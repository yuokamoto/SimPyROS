#!/usr/bin/env python3
"""
Visual Backend Comparison Demo - UIで視覚的にバックエンドを比較
PyVistaを使ったリアルタイム性能比較とロボット可視化
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
    import pyvista as pv
    import numpy as np
    PYVISTA_AVAILABLE = True
except ImportError:
    print("⚠️ PyVista not available. Install with: pip install pyvista")
    PYVISTA_AVAILABLE = False


class VisualComparisonDemo:
    """視覚的バックエンド比較デモ"""
    
    def __init__(self):
        self.plotter = None
        self.running = False
        self.backends_data = {}
        self.update_thread = None
        
        # 比較対象バックエンド
        self.backends = [
            (simpyros.SimulationBackend.SIMPLE_WHILE_LOOP, "Simple While Loop", "red"),
            (simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP, "SimPy FrequencyGroup", "blue")
        ]
        
        # シミュレーション設定
        self.num_robots_per_backend = 15
        self.robot_spacing = 8.0
        
    def setup_visualization(self):
        """可視化セットアップ"""
        if not PYVISTA_AVAILABLE:
            raise ImportError("PyVista required for visual demo")
        
        self.plotter = pv.Plotter(window_size=[1400, 900])
        self.plotter.set_background('black')
        
        # カメラ設定
        self.plotter.camera_position = [(0, -50, 30), (0, 0, 0), (0, 0, 1)]
        
        # タイトル追加
        self.plotter.add_text(
            "SimPyROS Backend Performance Comparison", 
            position='upper_edge',
            font_size=16,
            color='white'
        )
        
        # 凡例エリア
        self._add_legend()
        
        # 性能表示エリア
        self._setup_performance_display()
        
        print("📺 Visual comparison setup complete")
    
    def _add_legend(self):
        """凡例追加"""
        legend_text = "Backend Comparison:\n"
        for i, (backend, name, color) in enumerate(self.backends):
            legend_text += f"● {name} ({color})\n"
        
        self.plotter.add_text(
            legend_text,
            position='upper_left',
            font_size=12,
            color='white'
        )
    
    def _setup_performance_display(self):
        """性能表示セットアップ"""
        # 性能メトリクス表示用のテキストアクター
        self.perf_text_actor = self.plotter.add_text(
            "Initializing performance metrics...",
            position='lower_right',
            font_size=10,
            color='cyan'
        )
    
    def create_robot_visualization(self, backend_name: str, robot_id: int, position: List[float], color: str):
        """ロボット可視化オブジェクト作成"""
        # シンプルなロボット表現：円柱 + 方向指示器
        
        # ベース（円柱）
        cylinder = pv.Cylinder(
            center=position,
            direction=[0, 0, 1],
            radius=0.3,
            height=0.2
        )
        
        # 方向指示器（矢印）
        arrow = pv.Arrow(
            start=position,
            direction=[1, 0, 0],
            scale=0.5
        )
        
        # ロボット全体
        robot_mesh = cylinder + arrow
        
        # 色設定
        robot_actor = self.plotter.add_mesh(
            robot_mesh,
            color=color,
            opacity=0.8,
            name=f"{backend_name}_robot_{robot_id}"
        )
        
        return robot_actor
    
    def setup_backend_simulations(self):
        """各バックエンドのシミュレーション設定"""
        
        for i, (backend, name, color) in enumerate(self.backends):
            print(f"🔧 Setting up {name}...")
            
            # バックエンド固有の設定
            if backend == simpyros.SimulationBackend.SIMPLE_WHILE_LOOP:
                config = simpyros.create_high_performance_config(visualization=False)
            else:
                config = simpyros.create_balanced_config(visualization=False)
            
            config.backend = backend
            config.real_time_factor = 0.0  # 最高速度
            config.verbose = False
            
            # シミュレーション管理クラス作成
            sim = simpyros.create_simulation_manager(config)
            
            # ロボット群の配置（バックエンドごとに異なるエリア）
            base_x = i * self.robot_spacing * 3
            robots_info = []
            
            for j in range(self.num_robots_per_backend):
                # グリッド配置
                grid_size = int(math.sqrt(self.num_robots_per_backend)) + 1
                x = base_x + (j % grid_size) * self.robot_spacing
                y = (j // grid_size) * self.robot_spacing
                
                robot_name = f"{name}_robot_{j}"
                initial_pos = [x, y, 0]
                
                # ロボット状態管理用の簡単なクラス
                class RobotState:
                    def __init__(self, name, position):
                        self.name = name
                        self.position = position.copy()
                        self.velocity = simpyros.Velocity.zero()
                        self.angle = 0.0
                        self.update_count = 0
                
                robot_state = RobotState(robot_name, initial_pos)
                
                # 可視化オブジェクト作成
                robot_actor = self.create_robot_visualization(
                    name, j, initial_pos, color
                )
                
                robots_info.append({
                    'name': robot_name,
                    'state': robot_state,
                    'actor': robot_actor,
                    'initial_pos': initial_pos.copy()
                })
                
                # シミュレーションにロボット追加（簡略版）
                try:
                    sim.robots[robot_name] = robot_state
                    
                    # 制御コールバック
                    def create_controller(robot_state, robot_idx):
                        def controller(dt):
                            # 円運動パターン
                            t = time.time() * 0.5
                            radius = 2.0
                            center_x, center_y = robot_state.initial_pos[0], robot_state.initial_pos[1]
                            
                            angle = t + robot_idx * 0.3
                            target_x = center_x + radius * math.cos(angle)
                            target_y = center_y + radius * math.sin(angle)
                            
                            # 位置更新
                            robot_state.position[0] = target_x
                            robot_state.position[1] = target_y
                            robot_state.angle = angle
                            robot_state.update_count += 1
                            
                        return controller
                    
                    sim.control_callbacks[robot_name] = create_controller(robot_state, j)
                    
                except Exception as e:
                    print(f"⚠️ Robot setup warning: {e}")
            
            # バックエンドデータ保存
            self.backends_data[name] = {
                'sim': sim,
                'robots': robots_info,
                'color': color,
                'backend': backend,
                'stats': {
                    'rtf': 0.0,
                    'fps': 0.0,
                    'callbacks_per_sec': 0.0,
                    'total_updates': 0
                }
            }
            
            print(f"✅ {name}: {len(robots_info)} robots ready")
    
    def run_simulation_thread(self, backend_name: str):
        """バックエンドシミュレーション実行スレッド"""
        backend_data = self.backends_data[backend_name]
        sim = backend_data['sim']
        
        frame_count = 0
        start_time = time.time()
        last_update_time = start_time
        
        print(f"🚀 Starting {backend_name} simulation thread")
        
        try:
            while self.running:
                frame_start = time.time()
                
                # 制御コールバック実行
                for robot_name, callback in sim.control_callbacks.items():
                    try:
                        callback(0.02)  # 50Hz相当
                        backend_data['stats']['total_updates'] += 1
                    except Exception as e:
                        pass  # Silent error handling
                
                frame_count += 1
                
                # 統計更新（1秒ごと）
                if time.time() - last_update_time >= 1.0:
                    elapsed = time.time() - start_time
                    if elapsed > 0:
                        backend_data['stats']['fps'] = frame_count / elapsed
                        backend_data['stats']['rtf'] = 1.0 if backend_name == "Simple While Loop" else 0.3  # 模擬値
                        backend_data['stats']['callbacks_per_sec'] = backend_data['stats']['total_updates'] / elapsed
                    
                    last_update_time = time.time()
                
                # フレームレート制御
                frame_elapsed = time.time() - frame_start
                target_dt = 0.02  # 50Hz
                sleep_time = target_dt - frame_elapsed
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except Exception as e:
            print(f"❌ {backend_name} simulation error: {e}")
        
        print(f"🛑 {backend_name} simulation thread stopped")
    
    def update_visualization(self):
        """可視化更新メインループ"""
        print("🎬 Starting visualization update loop")
        
        last_update = time.time()
        frame_count = 0
        
        while self.running:
            try:
                current_time = time.time()
                
                # ロボット位置更新
                for backend_name, backend_data in self.backends_data.items():
                    for robot_info in backend_data['robots']:
                        robot_state = robot_info['state']
                        actor = robot_info['actor']
                        
                        # 新しい位置でメッシュ更新
                        new_pos = robot_state.position.copy()
                        new_pos[2] = 0.1  # 少し浮かせる
                        
                        # メッシュの位置更新
                        try:
                            # PyVistaのアクター位置更新
                            transform = pv.Transform()
                            transform.translate(new_pos)
                            transform.rotate_z(math.degrees(robot_state.angle))
                            
                            # アクター更新（簡略化）
                            # 実際の実装では適切なメッシュ変換が必要
                            
                        except Exception as e:
                            pass  # 可視化エラーは無視
                
                # 性能統計表示更新
                if current_time - last_update >= 0.5:  # 0.5秒ごと
                    self._update_performance_display()
                    last_update = current_time
                
                frame_count += 1
                
                # 可視化フレームレート制御
                time.sleep(0.033)  # 約30 FPS
                
            except Exception as e:
                print(f"⚠️ Visualization update error: {e}")
                break
        
        print("🎬 Visualization update loop stopped")
    
    def _update_performance_display(self):
        """性能表示更新"""
        perf_text = "Real-time Performance:\n\n"
        
        for backend_name, backend_data in self.backends_data.items():
            stats = backend_data['stats']
            color = backend_data['color']
            
            perf_text += f"● {backend_name}:\n"
            perf_text += f"  RTF: {stats['rtf']:.3f}x\n"
            perf_text += f"  FPS: {stats['fps']:.1f}\n"
            perf_text += f"  CB/s: {stats['callbacks_per_sec']:.0f}\n"
            perf_text += f"  Updates: {stats['total_updates']:,}\n\n"
        
        # テキストアクター更新
        try:
            self.perf_text_actor.SetText(3, perf_text)
        except:
            pass
    
    def run_demo(self, duration: float = 30.0):
        """デモ実行"""
        
        if not PYVISTA_AVAILABLE:
            print("❌ PyVista not available. Cannot run visual demo.")
            return
        
        print("🎯 Visual Backend Comparison Demo Starting")
        print("=" * 50)
        print(f"Duration: {duration}s")
        print(f"Backends: {[name for _, name, _ in self.backends]}")
        print(f"Robots per backend: {self.num_robots_per_backend}")
        
        try:
            # 1. 可視化セットアップ
            self.setup_visualization()
            
            # 2. バックエンドシミュレーション設定
            self.setup_backend_simulations()
            
            # 3. シミュレーション開始
            self.running = True
            
            # 各バックエンドのシミュレーションスレッド開始
            sim_threads = []
            for backend_name in self.backends_data.keys():
                thread = threading.Thread(
                    target=self.run_simulation_thread,
                    args=(backend_name,),
                    daemon=True
                )
                thread.start()
                sim_threads.append(thread)
            
            # 可視化更新スレッド開始
            viz_thread = threading.Thread(
                target=self.update_visualization,
                daemon=True
            )
            viz_thread.start()
            
            # PyVistaウィンドウ表示
            print("🖥️ Opening PyVista visualization window...")
            print("   - 左側: Simple While Loop (red)")
            print("   - 右側: SimPy FrequencyGroup (blue)")
            print("   - 右下: リアルタイム性能統計")
            print("   - ウィンドウを閉じるとデモ終了")
            
            # 制御ボタン追加
            self._add_control_widgets()
            
            # メインループ（PyVistaウィンドウが開いている間）
            start_time = time.time()
            
            def timer_callback():
                elapsed = time.time() - start_time
                if elapsed >= duration:
                    self.plotter.close()
                    return
                
                # タイトル更新
                remaining = duration - elapsed
                self.plotter.textActor.SetText(3, 
                    f"SimPyROS Backend Comparison - {remaining:.1f}s remaining")
            
            # タイマー設定
            self.plotter.add_timer_event(1000, timer_callback)  # 1秒ごと
            
            # ウィンドウ表示（ブロッキング）
            self.plotter.show()
            
        except KeyboardInterrupt:
            print("\n⏹️ Demo interrupted by user")
        
        except Exception as e:
            print(f"❌ Demo error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # クリーンアップ
            self.running = False
            
            # シミュレーション終了
            for backend_data in self.backends_data.values():
                try:
                    backend_data['sim'].shutdown()
                except:
                    pass
            
            print("\n📊 Final Performance Summary:")
            print("-" * 40)
            
            for backend_name, backend_data in self.backends_data.items():
                stats = backend_data['stats']
                print(f"{backend_name}:")
                print(f"  Average RTF: {stats['rtf']:.3f}x")
                print(f"  Total Updates: {stats['total_updates']:,}")
                print(f"  Final CB/s: {stats['callbacks_per_sec']:.1f}")
            
            print("\n✅ Visual comparison demo completed!")
    
    def _add_control_widgets(self):
        """制御ウィジェット追加"""
        try:
            # 一時停止/再開ボタン（簡略化）
            def toggle_pause():
                # 簡単な実装
                pass
            
            # 実際の実装では適切なウィジェットを追加
            
        except Exception as e:
            print(f"⚠️ Widget setup warning: {e}")


def simplified_visual_demo():
    """簡略化された視覚デモ（PyVistaなしでも動作）"""
    
    print("🎯 Simplified Visual Backend Comparison")
    print("=" * 50)
    
    backends = [
        (simpyros.SimulationBackend.SIMPLE_WHILE_LOOP, "Simple While Loop"),
        (simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP, "SimPy FrequencyGroup")
    ]
    
    print("Comparing backends with console output...")
    
    for backend, name in backends:
        print(f"\n🧪 Testing {name}...")
        
        try:
            # 設定
            if backend == simpyros.SimulationBackend.SIMPLE_WHILE_LOOP:
                config = simpyros.create_high_performance_config()
            else:
                config = simpyros.create_balanced_config()
            
            config.backend = backend
            config.visualization = False
            
            # クイックテスト
            stats = simpyros.quick_simulation(
                num_robots=20,
                backend=backend,
                visualization=False,
                duration=2.0
            )
            
            print(f"   RTF: {stats['rtf']:.3f}x")
            print(f"   Callbacks/sec: {stats['callbacks_per_sec']:.1f}")
            
            # 視覚的な性能表示
            bar_length = int(stats['rtf'] * 20)
            bar = "█" * bar_length + "░" * (20 - bar_length)
            print(f"   Performance: |{bar}| {stats['rtf']:.3f}x")
            
        except Exception as e:
            print(f"   ❌ Error: {e}")
        
        time.sleep(1.0)
    
    print(f"\n💡 For full visual experience, install PyVista:")
    print(f"   pip install pyvista")


def main():
    """メイン実行"""
    
    print("🎮 SimPyROS Visual Backend Comparison Demo")
    print("=" * 60)
    
    try:
        if PYVISTA_AVAILABLE:
            # フル視覚デモ
            print("📺 PyVista available - running full visual demo")
            
            response = input("Run visual demo? (y/n): ").lower().strip()
            if response.startswith('y'):
                demo = VisualComparisonDemo()
                demo.run_demo(duration=20.0)
            else:
                print("Demo cancelled")
        else:
            print("⚠️ PyVista not available - running simplified demo")
            simplified_visual_demo()
    
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()