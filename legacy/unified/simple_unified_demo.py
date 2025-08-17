#!/usr/bin/env python3
"""
Simple Unified Demo - 基本的な統一可視化デモ（エラー修正版）
"""

import sys
import os
import time
import math
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import simpyros

try:
    from core.pyvista_visualizer import PyVistaVisualizer, SceneBuilder
    import pyvista as pv
    PYVISTA_AVAILABLE = True
except ImportError:
    print("⚠️ PyVista not available")
    PYVISTA_AVAILABLE = False


class SimpleUnifiedDemo:
    """シンプルな統一デモ"""
    
    def __init__(self):
        self.visualizer = None
        self.current_sim = None
        self.current_backend = simpyros.SimulationBackend.SIMPLE_WHILE_LOOP
        self.running = False
        self.paused = False
        
        # ロボット可視化
        self.robot_actors = {}
        self.robot_meshes = {}
        
        # UI要素
        self.backend_text = None
        self.stats_text = None
    
    def setup_visualization(self):
        """可視化セットアップ"""
        
        if not PYVISTA_AVAILABLE:
            return False
        
        # 既存のPyVistaVisualizerを使用
        self.visualizer = PyVistaVisualizer(interactive=True, window_size=(1200, 800))
        
        if not self.visualizer.available:
            return False
        
        plotter = self.visualizer.plotter
        
        # シンプルなシーン設定
        plotter.set_background('darkblue')
        
        # 地面追加
        SceneBuilder.add_ground_plane(
            plotter, self.visualizer.pv,
            center=(0, 0, -0.5),
            size=20.0,
            color='gray',
            opacity=0.5
        )
        
        # 座標軸追加
        SceneBuilder.add_coordinate_axes(
            plotter, self.visualizer.pv,
            length=2.0,
            origin=(0, 0, 0)
        )
        
        # カメラ設定
        plotter.camera_position = [(0, -20, 10), (0, 0, 0), (0, 0, 1)]
        
        # UI追加
        self._setup_simple_ui()
        
        print("📺 Simple visualization setup complete")
        return True
    
    def _setup_simple_ui(self):
        """シンプルUI設定"""
        plotter = self.visualizer.plotter
        
        # タイトル
        plotter.add_text(
            "SimPyROS Simple Unified Demo",
            position='upper_edge',
            font_size=14,
            color='white'
        )
        
        # バックエンド表示
        self.backend_text = plotter.add_text(
            f"Backend: {self.current_backend.value}",
            position='upper_left',
            font_size=11,
            color='yellow'
        )
        
        # 統計表示
        self.stats_text = plotter.add_text(
            "Statistics: Initializing...",
            position='lower_left',
            font_size=10,
            color='cyan'
        )
        
        # 制御ガイド
        plotter.add_text(
            "Controls:\n1: Simple While Loop\n2: SimPy FrequencyGroup\n3: Pure SimPy\nSpace: Pause",
            position='lower_right',
            font_size=9,
            color='lightgreen'
        )
        
        # キーボードイベント
        self._setup_keyboard()
    
    def _setup_keyboard(self):
        """キーボード制御設定"""
        
        def handle_key(key):
            if key == '1':
                self._switch_backend(simpyros.SimulationBackend.SIMPLE_WHILE_LOOP)
            elif key == '2':
                self._switch_backend(simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP)
            elif key == '3':
                self._switch_backend(simpyros.SimulationBackend.SIMPY_PURE)
            elif key == ' ':
                self.paused = not self.paused
                print(f"🎮 {'Paused' if self.paused else 'Resumed'}")
        
        try:
            self.visualizer.plotter.iren.add_observer(
                'KeyPressEvent',
                lambda obj, event: handle_key(self.visualizer.plotter.iren.GetKeySym())
            )
        except Exception as e:
            print(f"⚠️ Keyboard setup warning: {e}")
    
    def create_simulation(self, backend: simpyros.SimulationBackend):
        """シミュレーション作成"""
        
        # 設定作成
        if backend == simpyros.SimulationBackend.SIMPLE_WHILE_LOOP:
            config = simpyros.create_high_performance_config()
        elif backend == simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP:
            config = simpyros.create_balanced_config()
        else:
            config = simpyros.create_feature_rich_config()
        
        config.backend = backend
        config.visualization = False
        config.verbose = False
        
        # 既存終了
        if self.current_sim:
            self.current_sim.shutdown()
        
        # 新規作成
        self.current_sim = simpyros.create_simulation_manager(config)
        self.current_backend = backend
        
        # ロボット追加
        self._create_simple_robots()
        
        # UI更新
        self.backend_text.SetText(3, f"Backend: {backend.value}")
        
        print(f"🔧 Created {backend.value} simulation")
    
    def _create_simple_robots(self):
        """シンプルなロボット作成"""
        
        num_robots = 10
        
        for i in range(num_robots):
            # 円形配置
            angle = i * 2 * math.pi / num_robots
            radius = 5.0
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            # ロボットクラス
            class SimpleRobot:
                def __init__(self, name, pos):
                    self.name = name
                    self.position = pos
                    self.orientation = [0, 0, 0]
                    self.update_count = 0
                    self.robot_id = i
            
            robot_name = f"simple_robot_{i}"
            robot = SimpleRobot(robot_name, [x, y, 0.0])
            
            self.current_sim.robots[robot_name] = robot
            
            # 制御コールバック
            def create_controller(robot_obj):
                def controller(dt):
                    if self.paused:
                        return
                    
                    # シンプルな円運動
                    t = time.time() * 0.5
                    angle = t + robot_obj.robot_id * 0.3
                    radius = 4.0 + math.sin(t) * 1.0
                    
                    robot_obj.position[0] = radius * math.cos(angle)
                    robot_obj.position[1] = radius * math.sin(angle)
                    robot_obj.position[2] = 0.2 + 0.1 * math.sin(t + robot_obj.robot_id)
                    robot_obj.orientation[2] = angle
                    robot_obj.update_count += 1
                
                return controller
            
            self.current_sim.control_callbacks[robot_name] = create_controller(robot)
        
        # 可視化に追加
        self._add_robots_to_visualization()
    
    def _add_robots_to_visualization(self):
        """ロボットを可視化に追加"""
        
        # 既存をクリア
        for name in list(self.robot_actors.keys()):
            try:
                self.visualizer.plotter.remove_actor(name)
            except:
                pass
        self.robot_actors.clear()
        
        # 新規追加
        for robot_name, robot in self.current_sim.robots.items():
            mesh = self._create_simple_robot_mesh()
            
            # バックエンド別の色
            if self.current_backend == simpyros.SimulationBackend.SIMPLE_WHILE_LOOP:
                color = 'red'
            elif self.current_backend == simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP:
                color = 'blue'
            else:
                color = 'green'
            
            # 位置設定
            positioned_mesh = mesh.translate(robot.position, inplace=False)
            
            # アクター追加
            actor = self.visualizer.plotter.add_mesh(
                positioned_mesh,
                color=color,
                opacity=0.8,
                name=robot_name
            )
            
            self.robot_actors[robot_name] = actor
            self.robot_meshes[robot_name] = mesh
        
        print(f"📺 Added {len(self.robot_actors)} robots to visualization")
    
    def _create_simple_robot_mesh(self):
        """シンプルなロボットメッシュ作成"""
        pv = self.visualizer.pv
        
        # ベース
        base = pv.Cylinder(
            center=[0, 0, 0.1],
            direction=[0, 0, 1],
            radius=0.3,
            height=0.2
        )
        
        # 上部
        top = pv.Sphere(center=[0, 0, 0.25], radius=0.15)
        
        # 方向矢印
        arrow = pv.Arrow(
            start=[0, 0, 0.2],
            direction=[1, 0, 0],
            scale=0.4
        )
        
        return base + top + arrow
    
    def _switch_backend(self, new_backend: simpyros.SimulationBackend):
        """バックエンド切り替え"""
        
        if self.current_backend == new_backend:
            print(f"⚠️ Already using {new_backend.value}")
            return
        
        print(f"🔄 Switching to {new_backend.value}...")
        
        # 新しいシミュレーション作成
        self.create_simulation(new_backend)
        
        print(f"✅ Switched to {new_backend.value}")
    
    def update_visualization(self):
        """可視化更新"""
        
        if not self.current_sim or not self.robot_actors:
            return
        
        # ロボット位置更新
        for robot_name, robot in self.current_sim.robots.items():
            if robot_name in self.robot_actors:
                try:
                    # 新しいメッシュで更新
                    new_mesh = self.robot_meshes[robot_name].copy()
                    positioned_mesh = new_mesh.translate(robot.position, inplace=False)
                    
                    # 回転適用
                    if hasattr(robot, 'orientation'):
                        yaw_deg = math.degrees(robot.orientation[2])
                        positioned_mesh = positioned_mesh.rotate_z(yaw_deg, inplace=False)
                    
                    # アクター更新
                    self.visualizer.plotter.remove_actor(robot_name)
                    
                    # 色設定
                    if self.current_backend == simpyros.SimulationBackend.SIMPLE_WHILE_LOOP:
                        color = 'red'
                    elif self.current_backend == simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP:
                        color = 'blue'
                    else:
                        color = 'green'
                    
                    actor = self.visualizer.plotter.add_mesh(
                        positioned_mesh,
                        color=color,
                        opacity=0.8,
                        name=robot_name
                    )
                    
                    self.robot_actors[robot_name] = actor
                    
                except Exception as e:
                    # 可視化エラーは無視
                    pass
    
    def update_statistics(self):
        """統計更新"""
        
        if not self.current_sim:
            return
        
        try:
            total_updates = sum(robot.update_count for robot in self.current_sim.robots.values())
            
            stats_text = (
                f"Statistics:\n"
                f"Backend: {self.current_backend.value}\n"
                f"Robots: {len(self.robot_actors)}\n"
                f"Updates: {total_updates}\n"
                f"Status: {'Paused' if self.paused else 'Running'}"
            )
            
            self.stats_text.SetText(3, stats_text)
            
        except Exception as e:
            print(f"⚠️ Stats update error: {e}")
    
    def simulation_loop(self):
        """シミュレーションループ"""
        
        print("🔄 Starting simulation loop")
        
        while self.running:
            try:
                if self.current_sim and not self.paused:
                    # コールバック実行
                    for callback in self.current_sim.control_callbacks.values():
                        callback(0.033)
                
                # 可視化更新
                self.update_visualization()
                
                # 統計更新
                self.update_statistics()
                
                time.sleep(0.033)  # 30 FPS
                
            except Exception as e:
                print(f"⚠️ Simulation loop error: {e}")
                break
        
        print("🛑 Simulation loop stopped")
    
    def run_demo(self):
        """デモ実行"""
        
        if not PYVISTA_AVAILABLE:
            print("❌ PyVista not available")
            return
        
        print("🎯 Simple Unified PyVista Demo")
        print("=" * 50)
        print("Controls:")
        print("  1: Simple While Loop (Red)")
        print("  2: SimPy FrequencyGroup (Blue)")
        print("  3: Pure SimPy (Green)")
        print("  Space: Pause/Resume")
        print("=" * 50)
        
        try:
            # 1. 可視化セットアップ
            if not self.setup_visualization():
                print("❌ Visualization setup failed")
                return
            
            # 2. 初期シミュレーション
            self.create_simulation(simpyros.SimulationBackend.SIMPLE_WHILE_LOOP)
            
            # 3. シミュレーションループ開始
            self.running = True
            sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
            sim_thread.start()
            
            # 4. 可視化表示（ブロッキング）
            print("🖥️ Opening PyVista window...")
            self.visualizer.plotter.show()
            
        except KeyboardInterrupt:
            print("\n⏹️ Demo interrupted")
        
        except Exception as e:
            print(f"❌ Demo error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self.running = False
            if self.current_sim:
                self.current_sim.shutdown()
            print("✅ Simple demo completed!")


def main():
    """メイン実行"""
    
    print("🎮 SimPyROS Simple Unified Demo")
    print("Basic backend switching with PyVista")
    print("=" * 50)
    
    if not PYVISTA_AVAILABLE:
        print("❌ PyVista not available. Install with: pip install pyvista")
        return
    
    try:
        demo = SimpleUnifiedDemo()
        demo.run_demo()
        
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()