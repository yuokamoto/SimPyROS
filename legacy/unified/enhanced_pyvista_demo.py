#!/usr/bin/env python3
"""
Enhanced PyVista Demo - 既存ライブラリを最大活用した統一可視化デモ
UnifiedPyVistaVisualizerとSimPyROS統一インターフェースの完全統合
"""

import sys
import os
import time
import math
import threading
from typing import Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import simpyros

try:
    from core.unified_pyvista_visualizer import UnifiedPyVistaVisualizer, create_unified_visualizer
    PYVISTA_AVAILABLE = True
except ImportError:
    print("⚠️ PyVista or unified visualizer not available")
    PYVISTA_AVAILABLE = False


class EnhancedPyVistaDemo:
    """拡張PyVistaデモ - 既存ライブラリ最大活用"""
    
    def __init__(self):
        self.visualizer = None
        self.current_sim = None
        self.simulation_thread = None
        self.running = False
        
        # デモ設定
        self.num_robots = 25
        self.robot_patterns = ['circle', 'spiral', 'figure8', 'grid']
        self.current_pattern = 0
        
    def setup_demo(self):
        """デモセットアップ"""
        
        if not PYVISTA_AVAILABLE:
            raise ImportError("PyVista unified visualizer not available")
        
        # 1. 統一ビジュアライザー作成
        self.visualizer = create_unified_visualizer(interactive=True)
        
        if not self.visualizer:
            raise RuntimeError("Failed to create unified visualizer")
        
        # 2. 統一シーン設定
        if not self.visualizer.setup_unified_scene():
            raise RuntimeError("Failed to setup unified scene")
        
        # 3. バックエンド切り替えコールバック登録
        self.visualizer.add_backend_switch_callback(self._on_backend_switch)
        
        print("🎯 Enhanced PyVista demo setup complete")
        
    def create_initial_simulation(self):
        """初期シミュレーション作成"""
        
        # Simple While Loopで開始
        config = simpyros.create_high_performance_config()
        config.visualization = False  # 統一ビジュアライザーが処理
        
        self.current_sim = simpyros.create_simulation_manager(config)
        
        # ビジュアライザーにアタッチ
        self.visualizer.attach_simulation_manager(self.current_sim)
        
        # ロボット追加
        self._create_demo_robots()
        
        print(f"🚀 Initial simulation created with {self.num_robots} robots")
        
    def _create_demo_robots(self):
        """デモ用ロボット作成"""
        
        for i in range(self.num_robots):
            # 初期位置（円形配置）
            angle = i * 2 * math.pi / self.num_robots
            radius = 10.0
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            # ロボット状態クラス（統一インターフェース互換）
            class EnhancedRobot:
                def __init__(self, name, initial_pos):
                    self.name = name
                    self.position = initial_pos.copy()
                    self.orientation = [0.0, 0.0, 0.0]  # Roll, Pitch, Yaw
                    self.velocity = simpyros.Velocity.zero()
                    self.update_count = 0
                    self.target_position = initial_pos.copy()
                    self.robot_id = int(name.split('_')[-1])
            
            robot_name = f"enhanced_robot_{i}"
            initial_pos = [x, y, 0.0]
            
            robot = EnhancedRobot(robot_name, initial_pos)
            self.current_sim.robots[robot_name] = robot
            
            # 動的制御コールバック
            self.current_sim.control_callbacks[robot_name] = self._create_enhanced_controller(robot)
    
    def _create_enhanced_controller(self, robot):
        """拡張制御コールバック作成"""
        
        def enhanced_controller(dt):
            if getattr(self.current_sim, 'paused', False):
                return
            
            current_time = time.time()
            robot_id = robot.robot_id
            
            # パターンに応じた運動
            pattern = self.robot_patterns[self.current_pattern % len(self.robot_patterns)]
            
            if pattern == 'circle':
                # 円運動
                angle = current_time * 0.5 + robot_id * 0.3
                radius = 8.0 + 2.0 * math.sin(current_time * 0.2)
                robot.position[0] = radius * math.cos(angle)
                robot.position[1] = radius * math.sin(angle)
                robot.position[2] = 0.5 + 0.3 * math.sin(current_time + robot_id)
                robot.orientation[2] = angle
                
            elif pattern == 'spiral':
                # スパイラル運動
                t = current_time * 0.3 + robot_id * 0.1
                radius = 5.0 + t * 0.5
                angle = t * 2.0
                robot.position[0] = radius * math.cos(angle)
                robot.position[1] = radius * math.sin(angle)
                robot.position[2] = 0.2 + 0.1 * math.sin(t * 3)
                robot.orientation[2] = angle
                
            elif pattern == 'figure8':
                # 8の字運動
                t = current_time * 0.4 + robot_id * 0.2
                scale = 6.0
                robot.position[0] = scale * math.sin(t)
                robot.position[1] = scale * math.sin(t) * math.cos(t)
                robot.position[2] = 0.3 + 0.2 * math.sin(t * 2)
                robot.orientation[2] = math.atan2(
                    math.cos(t) * math.cos(t) - math.sin(t) * math.sin(t),
                    math.cos(t) + math.sin(t) * math.cos(t)
                )
                
            else:  # grid
                # グリッド上での協調運動
                grid_size = int(math.sqrt(self.num_robots)) + 1
                grid_x = robot_id % grid_size
                grid_y = robot_id // grid_size
                
                base_x = (grid_x - grid_size/2) * 3.0
                base_y = (grid_y - grid_size/2) * 3.0
                
                wave_offset = current_time * 0.8 + robot_id * 0.1
                robot.position[0] = base_x + math.sin(wave_offset) * 0.5
                robot.position[1] = base_y + math.cos(wave_offset) * 0.5
                robot.position[2] = 0.2 + 0.3 * math.sin(wave_offset)
                robot.orientation[2] = wave_offset
            
            robot.update_count += 1
        
        return enhanced_controller
    
    def _on_backend_switch(self, new_backend: simpyros.SimulationBackend):
        """バックエンド切り替えコールバック"""
        
        print(f"🔄 Switching to {new_backend.value}...")
        
        try:
            # 現在のロボット状態を保存
            robot_states = {}
            if self.current_sim:
                for name, robot in self.current_sim.robots.items():
                    robot_states[name] = {
                        'position': robot.position.copy(),
                        'orientation': robot.orientation.copy() if hasattr(robot, 'orientation') else [0, 0, 0],
                        'robot_id': getattr(robot, 'robot_id', 0)
                    }
            
            # 新しいシミュレーション作成
            if new_backend == simpyros.SimulationBackend.SIMPLE_WHILE_LOOP:
                config = simpyros.create_high_performance_config()
            elif new_backend == simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP:
                config = simpyros.create_balanced_config()
            else:  # SIMPY_PURE
                config = simpyros.create_feature_rich_config()
            
            config.backend = new_backend
            config.visualization = False
            
            # 古いシミュレーション終了
            if self.current_sim:
                self.current_sim.shutdown()
            
            # 新しいシミュレーション作成
            self.current_sim = simpyros.create_simulation_manager(config)
            
            # ビジュアライザー更新
            self.visualizer.clear_robots()
            self.visualizer.attach_simulation_manager(self.current_sim)
            
            # ロボット状態復元
            self._restore_robots(robot_states)
            
            print(f"✅ Successfully switched to {new_backend.value}")
            
            # 運動パターン変更
            self.current_pattern = (self.current_pattern + 1) % len(self.robot_patterns)
            new_pattern = self.robot_patterns[self.current_pattern]
            print(f"🎨 Changed motion pattern to: {new_pattern}")
            
        except Exception as e:
            print(f"❌ Backend switch failed: {e}")
            import traceback
            traceback.print_exc()
    
    def _restore_robots(self, robot_states: dict):
        """ロボット状態復元"""
        
        for name, state in robot_states.items():
            # ロボットクラス作成
            class RestoredRobot:
                def __init__(self, name, state):
                    self.name = name
                    self.position = state['position']
                    self.orientation = state['orientation']
                    self.velocity = simpyros.Velocity.zero()
                    self.update_count = 0
                    self.robot_id = state['robot_id']
                    self.target_position = state['position'].copy()
            
            robot = RestoredRobot(name, state)
            self.current_sim.robots[name] = robot
            
            # 制御コールバック復元
            self.current_sim.control_callbacks[name] = self._create_enhanced_controller(robot)
    
    def start_simulation_loop(self):
        """シミュレーションループ開始"""
        
        self.running = True
        
        # シミュレーションスレッド開始
        self.simulation_thread = threading.Thread(
            target=self._enhanced_simulation_loop,
            daemon=True
        )
        self.simulation_thread.start()
        
        # 可視化ループ開始
        self.visualizer.start_visualization_loop()
        
        print("🔄 Enhanced simulation loop started")
        
    def _enhanced_simulation_loop(self):
        """拡張シミュレーションループ"""
        
        print(f"🚀 Enhanced simulation loop running")
        
        while self.running:
            try:
                if self.current_sim and not getattr(self.current_sim, 'paused', False):
                    # 制御コールバック実行
                    for callback in self.current_sim.control_callbacks.values():
                        callback(0.033)  # 30Hz
                
                # 追加の演出効果
                self._update_scene_effects()
                
                time.sleep(0.033)  # 30Hz制御
                
            except Exception as e:
                print(f"⚠️ Enhanced simulation loop error: {e}")
                break
        
        print(f"🛑 Enhanced simulation loop stopped")
    
    def _update_scene_effects(self):
        """シーン演出効果更新"""
        
        try:
            # 時間ベースの色変化など
            current_time = time.time()
            
            # 地面の色をゆっくり変化（オプション）
            # 実装は省略 - PyVistaの詳細操作が必要
            
        except Exception as e:
            # 演出エラーは無視
            pass
    
    def run_enhanced_demo(self, duration: Optional[float] = None):
        """拡張デモ実行"""
        
        if not PYVISTA_AVAILABLE:
            print("❌ PyVista not available for enhanced demo")
            return
        
        print("🎯 SimPyROS Enhanced PyVista Demo")
        print("=" * 60)
        print("Features:")
        print("  📺 Existing PyVistaVisualizer integration") 
        print("  🔄 Real-time backend switching")
        print("  🎨 Dynamic motion patterns")
        print("  🎮 Interactive keyboard controls")
        print("  📊 Live performance statistics")
        print()
        print("Controls:")
        print("  1️⃣: Simple While Loop (Red robots)")
        print("  2️⃣: SimPy FrequencyGroup (Blue robots)")
        print("  3️⃣: Pure SimPy (Green robots)")
        print("  Space: Pause/Resume")
        print("  R: Reset camera")
        print("  Q: Quit")
        print("=" * 60)
        
        try:
            # 1. デモセットアップ
            self.setup_demo()
            
            # 2. 初期シミュレーション作成
            self.create_initial_simulation()
            
            # 3. シミュレーションループ開始
            self.start_simulation_loop()
            
            # 4. メイン可視化ループ（ブロッキング）
            print("🖥️ Opening enhanced PyVista window...")
            self.visualizer.show_unified()
            
        except KeyboardInterrupt:
            print("\n⏹️ Enhanced demo interrupted")
        
        except Exception as e:
            print(f"❌ Enhanced demo error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # クリーンアップ
            self.running = False
            
            if self.visualizer:
                self.visualizer.stop_visualization_loop()
            
            if self.current_sim:
                try:
                    self.current_sim.shutdown()
                except:
                    pass
            
            print("✅ Enhanced PyVista demo completed!")


def main():
    """メイン実行"""
    
    print("🎮 SimPyROS Enhanced PyVista Demo")
    print("Leveraging existing PyVistaVisualizer with unified interface")
    print("=" * 70)
    
    if not PYVISTA_AVAILABLE:
        print("❌ PyVista or unified visualizer not available")
        print("💡 Install with: pip install pyvista")
        return
    
    try:
        demo = EnhancedPyVistaDemo()
        demo.run_enhanced_demo()
        
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()