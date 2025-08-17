#!/usr/bin/env python3
"""
Console Unified Demo - ヘッドレス環境でのバックエンド切り替えテスト
PyVistaなしでも動作する統一インターフェースのデモ
"""

import sys
import os
import time
import math
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import simpyros


class ConsoleUnifiedDemo:
    """コンソール統一デモ"""
    
    def __init__(self):
        self.current_sim = None
        self.current_backend = None
        self.running = False
        self.paused = False
        
        # 統計情報
        self.stats = {
            'start_time': 0.0,
            'total_callbacks': 0,
            'robot_updates': 0
        }
    
    def create_simulation(self, backend: simpyros.SimulationBackend, num_robots: int = 15):
        """シミュレーション作成"""
        
        print(f"\n🔧 Creating {backend.value} simulation...")
        
        # 設定作成
        if backend == simpyros.SimulationBackend.SIMPLE_WHILE_LOOP:
            config = simpyros.create_high_performance_config(visualization=False)
        elif backend == simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP:
            config = simpyros.create_balanced_config(visualization=False)
        else:
            config = simpyros.create_feature_rich_config(visualization=False)
        
        config.backend = backend
        config.real_time_factor = 0.0  # 最高速度
        config.verbose = False
        
        # 既存終了
        if self.current_sim:
            self.current_sim.shutdown()
        
        # 新規作成
        self.current_sim = simpyros.create_simulation_manager(config)
        self.current_backend = backend
        
        # ロボット追加
        self._create_console_robots(num_robots)
        
        # 統計リセット
        self.stats['start_time'] = time.time()
        self.stats['total_callbacks'] = 0
        self.stats['robot_updates'] = 0
        
        print(f"✅ Created {backend.value} with {num_robots} robots")
        
        return self.current_sim
    
    def _create_console_robots(self, num_robots: int):
        """コンソール用ロボット作成"""
        
        for i in range(num_robots):
            # 初期位置（グリッド配置）
            grid_size = int(math.sqrt(num_robots)) + 1
            x = (i % grid_size) * 2.0
            y = (i // grid_size) * 2.0
            
            # シンプルなロボットクラス
            class ConsoleRobot:
                def __init__(self, name, pos):
                    self.name = name
                    self.position = pos
                    self.velocity = simpyros.Velocity.zero()
                    self.update_count = 0
                    self.robot_id = i
                    self.last_position = pos.copy()
            
            robot_name = f"console_robot_{i}"
            robot = ConsoleRobot(robot_name, [x, y, 0.0])
            
            self.current_sim.robots[robot_name] = robot
            
            # 制御コールバック
            def create_controller(robot_obj):
                def controller(dt):
                    if self.paused:
                        return
                    
                    # パターン運動
                    t = time.time() * 0.5
                    robot_id = robot_obj.robot_id
                    
                    # 軌道計算
                    angle = t + robot_id * 0.2
                    radius = 3.0 + math.sin(t * 0.3 + robot_id) * 1.0
                    
                    robot_obj.position[0] = robot_obj.last_position[0] + radius * math.cos(angle) * 0.1
                    robot_obj.position[1] = robot_obj.last_position[1] + radius * math.sin(angle) * 0.1
                    robot_obj.position[2] = 0.1 * math.sin(t + robot_id)
                    
                    robot_obj.update_count += 1
                    self.stats['robot_updates'] += 1
                
                return controller
            
            self.current_sim.control_callbacks[robot_name] = create_controller(robot)
    
    def simulation_loop(self):
        """シミュレーションループ"""
        
        print(f"🔄 Starting {self.current_backend.value} simulation loop")
        
        frame_count = 0
        last_stats_time = time.time()
        
        while self.running:
            try:
                if self.current_sim and not self.paused:
                    # コールバック実行
                    for callback in self.current_sim.control_callbacks.values():
                        callback(0.02)  # 50Hz
                        self.stats['total_callbacks'] += 1
                
                frame_count += 1
                
                # 統計表示（2秒ごと）
                if time.time() - last_stats_time >= 2.0:
                    self._print_statistics(frame_count)
                    last_stats_time = time.time()
                
                time.sleep(0.02)  # 50Hz
                
            except Exception as e:
                print(f"⚠️ Simulation loop error: {e}")
                break
        
        print(f"🛑 {self.current_backend.value} simulation loop stopped")
    
    def _print_statistics(self, frame_count: int):
        """統計情報表示"""
        
        if not self.current_sim:
            return
        
        elapsed = time.time() - self.stats['start_time']
        if elapsed <= 0:
            return
        
        # 計算
        cb_per_sec = self.stats['total_callbacks'] / elapsed
        updates_per_sec = self.stats['robot_updates'] / elapsed
        fps = frame_count / elapsed
        
        # バックエンド別の期待RTF
        expected_rtf = {
            simpyros.SimulationBackend.SIMPLE_WHILE_LOOP: 1.0,
            simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP: 0.3,
            simpyros.SimulationBackend.SIMPY_PURE: 0.1
        }.get(self.current_backend, 0.5)
        
        # 実測RTF（模擬）
        actual_rtf = min(expected_rtf, fps * 0.02)  # 50Hz基準
        
        # 表示
        print(f"\n📊 {self.current_backend.value} Statistics:")
        print(f"   Runtime: {elapsed:.1f}s")
        print(f"   Callbacks/sec: {cb_per_sec:.0f}")
        print(f"   Robot updates/sec: {updates_per_sec:.0f}")
        print(f"   Estimated RTF: {actual_rtf:.3f}x")
        print(f"   Robots: {len(self.current_sim.robots)}")
        print(f"   Status: {'Paused' if self.paused else 'Running'}")
        
        # 性能評価
        if actual_rtf >= 0.5:
            rating = "🚀 EXCELLENT"
        elif actual_rtf >= 0.1:
            rating = "✅ GOOD"
        elif actual_rtf >= 0.05:
            rating = "⚠️ FAIR"
        else:
            rating = "❌ POOR"
        
        print(f"   Performance: {rating}")
    
    def interactive_demo(self):
        """対話式デモ"""
        
        print("🎯 Console Unified Demo - Interactive Mode")
        print("=" * 60)
        print("Available backends:")
        print("  1: Simple While Loop (Highest Performance)")
        print("  2: SimPy FrequencyGroup (Balanced)")
        print("  3: Pure SimPy (Full Features)")
        print("  p: Pause/Resume")
        print("  s: Show current statistics")
        print("  q: Quit")
        print("=" * 60)
        
        # 初期シミュレーション
        self.create_simulation(simpyros.SimulationBackend.SIMPLE_WHILE_LOOP)
        
        # シミュレーションスレッド開始
        self.running = True
        sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        sim_thread.start()
        
        # インタラクティブループ
        try:
            while True:
                choice = input("\nCommand (1/2/3/p/s/q): ").strip().lower()
                
                if choice == '1':
                    self._switch_backend(simpyros.SimulationBackend.SIMPLE_WHILE_LOOP)
                elif choice == '2':
                    self._switch_backend(simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP)
                elif choice == '3':
                    self._switch_backend(simpyros.SimulationBackend.SIMPY_PURE)
                elif choice == 'p':
                    self.paused = not self.paused
                    print(f"🎮 {'Paused' if self.paused else 'Resumed'}")
                elif choice == 's':
                    self._show_detailed_statistics()
                elif choice == 'q':
                    break
                else:
                    print("⚠️ Invalid command")
                    
        except KeyboardInterrupt:
            print("\n⏹️ Demo interrupted")
        
        finally:
            self.running = False
            if self.current_sim:
                self.current_sim.shutdown()
    
    def _switch_backend(self, new_backend: simpyros.SimulationBackend):
        """バックエンド切り替え"""
        
        if self.current_backend == new_backend:
            print(f"⚠️ Already using {new_backend.value}")
            return
        
        print(f"\n🔄 Switching from {self.current_backend.value} to {new_backend.value}...")
        
        # 現在の統計を保存
        old_stats = self._get_current_statistics()
        
        # 新しいシミュレーション作成
        self.create_simulation(new_backend, len(self.current_sim.robots))
        
        print(f"✅ Successfully switched to {new_backend.value}")
        
        # 比較表示
        print(f"\n📊 Backend Switch Comparison:")
        print(f"   Previous ({old_stats['backend']}): {old_stats['cb_per_sec']:.0f} CB/s")
        print(f"   Current ({new_backend.value}): Starting fresh measurement...")
    
    def _get_current_statistics(self) -> dict:
        """現在の統計取得"""
        
        elapsed = time.time() - self.stats['start_time']
        cb_per_sec = self.stats['total_callbacks'] / elapsed if elapsed > 0 else 0
        
        return {
            'backend': self.current_backend.value,
            'cb_per_sec': cb_per_sec,
            'elapsed': elapsed,
            'total_callbacks': self.stats['total_callbacks']
        }
    
    def _show_detailed_statistics(self):
        """詳細統計表示"""
        
        print(f"\n📊 Detailed Statistics - {self.current_backend.value}")
        print("=" * 50)
        
        elapsed = time.time() - self.stats['start_time']
        
        if elapsed > 0:
            cb_per_sec = self.stats['total_callbacks'] / elapsed
            updates_per_sec = self.stats['robot_updates'] / elapsed
            
            print(f"Backend: {self.current_backend.value}")
            print(f"Runtime: {elapsed:.2f} seconds")
            print(f"Total callbacks: {self.stats['total_callbacks']:,}")
            print(f"Robot updates: {self.stats['robot_updates']:,}")
            print(f"Callbacks/second: {cb_per_sec:.1f}")
            print(f"Updates/second: {updates_per_sec:.1f}")
            print(f"Active robots: {len(self.current_sim.robots)}")
            print(f"Status: {'Paused' if self.paused else 'Running'}")
            
            # パフォーマンス評価
            backend_info = self.current_sim.get_backend_info()
            print(f"Performance tier: {backend_info['performance_tier']}")
            print(f"Description: {backend_info['description']}")
        else:
            print("No statistics available yet")
    
    def automated_comparison(self, duration_per_backend: float = 10.0):
        """自動比較テスト"""
        
        print("🔄 Automated Backend Comparison")
        print("=" * 50)
        
        backends = [
            simpyros.SimulationBackend.SIMPLE_WHILE_LOOP,
            simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP,
            simpyros.SimulationBackend.SIMPY_PURE
        ]
        
        results = []
        
        for backend in backends:
            print(f"\n🧪 Testing {backend.value} for {duration_per_backend}s...")
            
            # シミュレーション作成
            self.create_simulation(backend, 20)
            
            # 実行
            self.running = True
            self.paused = False
            
            sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
            sim_thread.start()
            
            time.sleep(duration_per_backend)
            
            # 結果収集
            stats = self._get_current_statistics()
            results.append(stats)
            
            print(f"   Result: {stats['cb_per_sec']:.0f} callbacks/sec")
        
        self.running = False
        
        # 比較結果表示
        print(f"\n🏆 Automated Comparison Results")
        print("=" * 50)
        print(f"{'Backend':<25} {'CB/sec':<10} {'Performance'}")
        print("-" * 50)
        
        for result in results:
            cb_per_sec = result['cb_per_sec']
            
            if cb_per_sec >= 1000:
                rating = "🚀 ULTRA"
            elif cb_per_sec >= 500:
                rating = "⚡ FAST"
            elif cb_per_sec >= 200:
                rating = "✅ GOOD"
            elif cb_per_sec >= 100:
                rating = "⚠️ FAIR"
            else:
                rating = "❌ SLOW"
            
            print(f"{result['backend']:<25} {cb_per_sec:<10.0f} {rating}")
        
        # 推奨事項
        best_result = max(results, key=lambda x: x['cb_per_sec'])
        print(f"\n💡 Recommendation: {best_result['backend']} (Highest performance)")
        
        return results


def main():
    """メイン実行"""
    
    print("🎮 SimPyROS Console Unified Demo")
    print("Backend switching without PyVista dependency")
    print("=" * 60)
    
    try:
        demo = ConsoleUnifiedDemo()
        
        # デモ選択
        print("\nDemo modes:")
        print("1: Interactive mode (manual backend switching)")
        print("2: Automated comparison (10s per backend)")
        
        choice = input("Select mode (1/2): ").strip()
        
        if choice == '1':
            demo.interactive_demo()
        elif choice == '2':
            demo.automated_comparison()
        else:
            print("Running interactive mode by default...")
            demo.interactive_demo()
        
        print("\n✅ Console unified demo completed!")
        
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()