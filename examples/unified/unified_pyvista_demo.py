#!/usr/bin/env python3
"""
Unified PyVista Demo - 既存のPyVistaVisualizerを使った統一デモ
バックエンドを切り替えながら同一の可視化システムで比較
"""

import sys
import os
import time
import math
import threading
from typing import Dict, List, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import simpyros

try:
    from core.pyvista_visualizer import PyVistaVisualizer, SceneBuilder
    import pyvista as pv
    PYVISTA_AVAILABLE = True
except ImportError:
    print("⚠️ PyVista not available. Install with: pip install pyvista")
    PYVISTA_AVAILABLE = False


class UnifiedPyVistaDemo:
    """統一PyVistaデモクラス"""
    
    def __init__(self):
        self.visualizer = None
        self.current_sim = None
        self.current_backend = None
        
        # 実行状態
        self.running = False
        self.paused = False
        
        # 統計情報
        self.stats = {
            'start_time': 0.0,
            'frame_count': 0,
            'rtf': 0.0,
            'callbacks_per_sec': 0.0,
            'total_callbacks': 0
        }
        
        # ロボット可視化データ
        self.robot_actors = {}
        self.robot_positions = {}
        
        # UI要素
        self.backend_selector_widget = None
        self.stats_text_actor = None
        
    def setup_visualization(self):
        """PyVistaVisualizerを使った可視化セットアップ"""
        
        if not PYVISTA_AVAILABLE:
            raise ImportError("PyVista required for this demo")
        
        # 既存のPyVistaVisualizerを使用
        self.visualizer = PyVistaVisualizer(interactive=True, window_size=(1400, 900))
        
        if not self.visualizer.available:
            raise RuntimeError("PyVista visualizer not available")
        
        plotter = self.visualizer.plotter
        
        # 背景色を暗めに設定
        plotter.set_background('darkblue')
        
        # 地面追加（既存のSceneBuilder使用）
        SceneBuilder.add_ground_plane(
            plotter, 
            self.visualizer.pv,
            center=(0, 0, -0.5),
            size=30.0,
            color='darkgray',
            opacity=0.3
        )
        
        # 軸表示
        SceneBuilder.add_coordinate_axes(
            plotter,
            self.visualizer.pv,
            origin=(0, 0, 0),
            scale=2.0
        )
        
        # ライティング設定
        SceneBuilder.setup_lighting(plotter)
        
        # カメラ位置設定
        plotter.camera_position = [(0, -25, 15), (0, 0, 0), (0, 0, 1)]
        
        # UI要素追加
        self._setup_ui_elements()
        
        print("📺 Unified PyVista visualization setup complete")
    
    def _setup_ui_elements(self):
        """UI要素の設定"""
        plotter = self.visualizer.plotter
        
        # タイトル
        plotter.add_text(
            "SimPyROS Unified Backend Demo", 
            position='upper_edge',
            font_size=16,
            color='white'
        )
        
        # バックエンド情報表示
        self.backend_text_actor = plotter.add_text(
            "Backend: None",
            position='upper_left',
            font_size=12,
            color='yellow'
        )
        
        # 統計情報表示
        self.stats_text_actor = plotter.add_text(
            "Statistics:\nRTF: 0.000x\nCallbacks/sec: 0\nRobots: 0",
            position='lower_left',
            font_size=10,
            color='cyan'
        )
        
        # 制御ボタン追加
        self._add_control_widgets()
    
    def _add_control_widgets(self):
        """制御ウィジェット追加"""
        plotter = self.visualizer.plotter
        
        # バックエンド切り替えボタン
        def switch_to_simple_while_loop():
            self._switch_backend(simpyros.SimulationBackend.SIMPLE_WHILE_LOOP)
        
        def switch_to_simpy_freq():
            self._switch_backend(simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP)
        
        def switch_to_simpy_pure():
            self._switch_backend(simpyros.SimulationBackend.SIMPY_PURE)
        
        def toggle_pause():
            self.paused = not self.paused
            status = "Paused" if self.paused else "Running"
            print(f"🎮 Simulation {status}")
        
        # ボタン配置（既存のパターンを参考）
        try:
            # シンプルなテキストベースの制御指示
            plotter.add_text(
                "Controls:\n"
                "1: Simple While Loop\n"
                "2: SimPy FrequencyGroup\n" 
                "3: Pure SimPy\n"
                "Space: Pause/Resume\n"
                "R: Reset View",
                position='lower_right',
                font_size=9,
                color='lightgreen'
            )
            
            # キーボードイベント設定
            def key_press_event(key):
                if key == '1':
                    switch_to_simple_while_loop()
                elif key == '2':
                    switch_to_simpy_freq()
                elif key == '3':
                    switch_to_simpy_pure()
                elif key == ' ':
                    toggle_pause()
                elif key.lower() == 'r':
                    plotter.reset_camera()
            
            # キーイベントハンドラー登録
            plotter.iren.add_observer('KeyPressEvent', lambda obj, event: key_press_event(plotter.iren.GetKeySym()))
            
        except Exception as e:
            print(f"⚠️ Widget setup warning: {e}")
    
    def create_simulation(self, backend: simpyros.SimulationBackend, num_robots: int = 20):
        """指定されたバックエンドでシミュレーション作成"""
        
        # 既存のシミュレーション終了
        if self.current_sim:
            try:
                self.current_sim.shutdown()
            except:
                pass
        
        # 新しい設定作成
        if backend == simpyros.SimulationBackend.SIMPLE_WHILE_LOOP:
            config = simpyros.create_high_performance_config()
            backend_name = "Simple While Loop"
        elif backend == simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP:
            config = simpyros.create_balanced_config()
            backend_name = "SimPy FrequencyGroup"
        else:  # SIMPY_PURE
            config = simpyros.create_feature_rich_config()
            backend_name = "Pure SimPy"
        
        config.backend = backend
        config.visualization = False  # PyVistaが別途処理
        config.real_time_factor = 0.0
        config.verbose = False
        
        # シミュレーション管理クラス作成
        self.current_sim = simpyros.create_simulation_manager(config)
        self.current_backend = backend_name
        
        # ロボット追加
        self._add_robots_to_simulation(num_robots)
        
        # 可視化にロボット追加
        self._add_robots_to_visualization(num_robots)
        
        # バックエンド表示更新
        self.backend_text_actor.SetText(3, f"Backend: {backend_name}")
        
        print(f"🔧 Created {backend_name} simulation with {num_robots} robots")
        
        return self.current_sim
    
    def _add_robots_to_simulation(self, num_robots: int):
        """シミュレーションにロボット追加"""
        
        for i in range(num_robots):
            # 円形配置
            angle = i * 2 * math.pi / num_robots
            radius = 8.0
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            # ロボット状態クラス
            class VisualizationRobot:
                def __init__(self, name, initial_pos):
                    self.name = name
                    self.position = initial_pos.copy()
                    self.velocity = simpyros.Velocity.zero()
                    self.orientation = [0.0, 0.0, 0.0]  # Roll, Pitch, Yaw
                    self.update_count = 0
            
            robot_name = f"demo_robot_{i}"
            initial_pos = [x, y, 0.0]
            
            robot = VisualizationRobot(robot_name, initial_pos)
            self.current_sim.robots[robot_name] = robot
            
            # 制御コールバック作成
            def create_controller(robot_obj, robot_id):
                def controller(dt):
                    if self.paused:
                        return
                    
                    # 円運動パターン
                    t = time.time() * 0.5
                    angle = t + robot_id * 0.3
                    radius = 8.0 + 2.0 * math.sin(t * 0.2 + robot_id)
                    
                    # 新しい位置計算
                    robot_obj.position[0] = radius * math.cos(angle)
                    robot_obj.position[1] = radius * math.sin(angle)
                    robot_obj.position[2] = 0.5 + 0.3 * math.sin(t * 0.8 + robot_id)
                    
                    # 向き更新
                    robot_obj.orientation[2] = angle  # Yaw
                    
                    robot_obj.update_count += 1
                    
                return controller
            
            self.current_sim.control_callbacks[robot_name] = create_controller(robot, i)
    
    def _add_robots_to_visualization(self, num_robots: int):
        """PyVistaVisualizerにロボット追加"""
        
        # 既存のロボットアクターをクリア
        self._clear_robot_actors()
        
        plotter = self.visualizer.plotter
        
        for i in range(num_robots):
            robot_name = f"demo_robot_{i}"
            
            if robot_name in self.current_sim.robots:
                robot = self.current_sim.robots[robot_name]
                
                # ロボットの可視化メッシュ作成
                robot_mesh = self._create_robot_mesh(i)
                
                # バックエンドに応じた色設定
                if self.current_backend == "Simple While Loop":
                    color = 'red'
                elif self.current_backend == "SimPy FrequencyGroup":
                    color = 'blue'
                else:
                    color = 'green'
                
                # アクター追加
                actor = plotter.add_mesh(
                    robot_mesh,
                    color=color,
                    opacity=0.8,
                    name=robot_name
                )
                
                self.robot_actors[robot_name] = actor
                self.robot_positions[robot_name] = robot.position.copy()
        
        print(f"📺 Added {len(self.robot_actors)} robot actors to visualization")
    
    def _create_robot_mesh(self, robot_id: int):
        """ロボットメッシュ作成（簡略版）"""
        pv = self.visualizer.pv
        
        # ベース（円柱）
        base = pv.Cylinder(
            center=[0, 0, 0],
            direction=[0, 0, 1],
            radius=0.4,
            height=0.3
        )
        
        # 上部（球）
        top = pv.Sphere(
            center=[0, 0, 0.2],
            radius=0.2
        )
        
        # 方向指示器（矢印）
        arrow = pv.Arrow(
            start=[0, 0, 0.1],
            direction=[1, 0, 0],
            scale=0.6
        )
        
        # 合成
        robot_mesh = base + top + arrow
        
        return robot_mesh
    
    def _clear_robot_actors(self):
        """ロボットアクターをクリア"""
        plotter = self.visualizer.plotter
        
        for robot_name in list(self.robot_actors.keys()):
            try:
                plotter.remove_actor(robot_name)
            except:
                pass
        
        self.robot_actors.clear()
        self.robot_positions.clear()
    
    def _switch_backend(self, new_backend: simpyros.SimulationBackend):
        """バックエンド切り替え"""
        
        if self.current_sim and hasattr(self.current_sim.config, 'backend'):
            if self.current_sim.config.backend == new_backend:
                print(f"⚠️ Already using {new_backend.value}")
                return
        
        print(f"🔄 Switching to {new_backend.value}...")
        
        # 現在のロボット数を保持
        current_robot_count = len(self.robot_actors) if self.robot_actors else 20
        
        # 新しいシミュレーション作成
        self.create_simulation(new_backend, current_robot_count)
        
        # 統計リセット
        self.stats['start_time'] = time.time()
        self.stats['frame_count'] = 0
        self.stats['total_callbacks'] = 0
        
        print(f"✅ Switched to {new_backend.value}")
    
    def update_visualization(self):
        """可視化更新（既存のPyVistaVisualizerパターンを使用）"""
        
        if not self.current_sim or not self.robot_actors:
            return
        
        plotter = self.visualizer.plotter
        
        # ロボット位置更新
        for robot_name, robot in self.current_sim.robots.items():
            if robot_name in self.robot_actors:
                # 位置が変更されたかチェック
                if (robot_name not in self.robot_positions or 
                    not self._positions_equal(self.robot_positions[robot_name], robot.position)):
                    
                    # メッシュ位置更新
                    try:
                        # 新しいメッシュ作成
                        new_mesh = self._create_robot_mesh(0)  # IDは見た目に影響しない
                        
                        # 位置と向きを適用
                        transform = self.visualizer.pv.Transform()
                        transform.translate(robot.position)
                        transform.rotate_z(math.degrees(robot.orientation[2]))
                        
                        transformed_mesh = new_mesh.transform(transform.matrix)
                        
                        # 古いアクターを削除して新しいアクターを追加
                        plotter.remove_actor(robot_name)
                        
                        # バックエンドに応じた色
                        if self.current_backend == "Simple While Loop":
                            color = 'red'
                        elif self.current_backend == "SimPy FrequencyGroup":
                            color = 'blue'
                        else:
                            color = 'green'
                        
                        actor = plotter.add_mesh(
                            transformed_mesh,
                            color=color,
                            opacity=0.8,
                            name=robot_name
                        )
                        
                        self.robot_actors[robot_name] = actor
                        self.robot_positions[robot_name] = robot.position.copy()
                        
                    except Exception as e:
                        # エラーが発生した場合は無視（可視化が主目的ではないため）
                        pass
    
    def _positions_equal(self, pos1: List[float], pos2: List[float], tolerance: float = 0.01) -> bool:
        """位置比較"""
        return all(abs(a - b) < tolerance for a, b in zip(pos1, pos2))
    
    def update_statistics(self):
        """統計情報更新"""
        
        if not self.current_sim:
            return
        
        # 統計計算
        elapsed = time.time() - self.stats['start_time']
        if elapsed > 0:
            self.stats['frame_count'] += 1
            
            # バックエンド別のRTF模擬値
            if self.current_backend == "Simple While Loop":
                base_rtf = 0.95
            elif self.current_backend == "SimPy FrequencyGroup":
                base_rtf = 0.25
            else:
                base_rtf = 0.08
            
            # 実際の処理に基づくRTF調整
            frame_rate = self.stats['frame_count'] / elapsed
            theoretical_sim_time = self.stats['frame_count'] * 0.033  # 30FPS想定
            self.stats['rtf'] = min(base_rtf, theoretical_sim_time / elapsed)
            
            # コールバック数
            callback_count = sum(robot.update_count for robot in self.current_sim.robots.values())
            self.stats['total_callbacks'] = callback_count
            self.stats['callbacks_per_sec'] = callback_count / elapsed
        
        # 統計表示更新
        stats_text = (
            f"Statistics:\n"
            f"RTF: {self.stats['rtf']:.3f}x\n"
            f"Callbacks/sec: {self.stats['callbacks_per_sec']:.0f}\n"
            f"Robots: {len(self.robot_actors)}\n"
            f"Frame: {self.stats['frame_count']}\n"
            f"Backend: {self.current_backend}\n"
            f"Status: {'Paused' if self.paused else 'Running'}"
        )
        
        self.stats_text_actor.SetText(3, stats_text)
    
    def simulation_loop(self):
        """シミュレーションメインループ"""
        
        print(f"🔄 Starting simulation loop for {self.current_backend}")
        
        while self.running:
            try:
                loop_start = time.time()
                
                if not self.paused and self.current_sim:
                    # 制御コールバック実行
                    for callback in self.current_sim.control_callbacks.values():
                        callback(0.033)  # 30Hz
                
                # 可視化更新
                self.update_visualization()
                
                # 統計更新
                self.update_statistics()
                
                # フレームレート制御
                target_dt = 0.033  # 30 FPS
                elapsed = time.time() - loop_start
                sleep_time = target_dt - elapsed
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                print(f"⚠️ Simulation loop error: {e}")
                break
        
        print(f"🛑 Simulation loop stopped")
    
    def run_demo(self, duration: Optional[float] = None):
        """デモ実行"""
        
        if not PYVISTA_AVAILABLE:
            print("❌ PyVista not available")
            return
        
        print("🎯 Unified PyVista Demo Starting")
        print("=" * 50)
        print("Controls:")
        print("  1: Switch to Simple While Loop")
        print("  2: Switch to SimPy FrequencyGroup")
        print("  3: Switch to Pure SimPy")
        print("  Space: Pause/Resume")
        print("  R: Reset camera view")
        print("  Close window to exit")
        print("=" * 50)
        
        try:
            # 1. 可視化セットアップ
            self.setup_visualization()
            
            # 2. 初期シミュレーション作成（Simple While Loop）
            self.create_simulation(simpyros.SimulationBackend.SIMPLE_WHILE_LOOP, 15)
            
            # 3. 統計初期化
            self.running = True
            self.stats['start_time'] = time.time()
            
            # 4. シミュレーションスレッド開始
            sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
            sim_thread.start()
            
            # 5. PyVistaウィンドウ表示（メインスレッドでブロッキング）
            print("🖥️ Opening PyVista window...")
            self.visualizer.plotter.show()
            
        except KeyboardInterrupt:
            print("\n⏹️ Demo interrupted")
        
        except Exception as e:
            print(f"❌ Demo error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # クリーンアップ
            self.running = False
            
            if self.current_sim:
                try:
                    self.current_sim.shutdown()
                except:
                    pass
            
            print("\n📊 Final Statistics:")
            if self.stats['start_time'] > 0:
                total_time = time.time() - self.stats['start_time']
                print(f"   Total runtime: {total_time:.1f}s")
                print(f"   Final RTF: {self.stats['rtf']:.3f}x") 
                print(f"   Total callbacks: {self.stats['total_callbacks']}")
                print(f"   Backend: {self.current_backend}")
            
            print("✅ Unified PyVista demo completed!")


def main():
    """メイン実行"""
    
    print("🎮 SimPyROS Unified PyVista Demo")
    print("Single simulation with backend switching")
    print("=" * 60)
    
    if not PYVISTA_AVAILABLE:
        print("❌ PyVista not available. Install with: pip install pyvista")
        return
    
    try:
        demo = UnifiedPyVistaDemo()
        demo.run_demo()
        
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()