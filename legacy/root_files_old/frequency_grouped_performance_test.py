#!/usr/bin/env python3
"""
Frequency-Grouped Performance Test - SimPyROS

Implements frequency-based process grouping optimization where robots with the 
same update frequency share the same SimPy process and yield, dramatically 
reducing scheduling overhead.

Key Innovation: Instead of N processes (one per robot), we have F processes 
(one per unique frequency), where F << N for most scenarios.

Usage:
    python frequency_grouped_performance_test.py
"""

import sys
import os
import time
import math
from collections import defaultdict
from typing import Dict, List, Callable, Any

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose

# ==============================
# 周波数グループ化設定
# ==============================
NUM_ROBOTS = 100
FREQUENCY_GROUPS = [5.0, 10.0, 15.0, 20.0]  # 使用する周波数
DURATION = 10.0

class FrequencyGroupedSimulation:
    """周波数でグループ化されたシミュレーション"""
    
    def __init__(self, config: SimulationConfig):
        self.sim = SimulationManager(config)
        
        # 周波数別グループ管理
        self.frequency_groups: Dict[float, List] = defaultdict(list)
        self.group_controllers: Dict[float, List[Callable]] = defaultdict(list)
        self.group_processes: Dict[float, Any] = {}
        
        # パフォーマンス追跡
        self.total_callbacks = 0
        self.frequency_callback_counts: Dict[float, int] = defaultdict(int)
    
    def add_robot_to_frequency_group(self, robot_name: str, robot_instance, 
                                   controller: Callable, frequency: float):
        """ロボットを周波数グループに追加"""
        
        # ロボットをグループに追加
        robot_info = {
            'name': robot_name,
            'instance': robot_instance,
            'controller': controller
        }
        self.frequency_groups[frequency].append(robot_info)
        
        print(f"   Added {robot_name} to {frequency}Hz group (total: {len(self.frequency_groups[frequency])})")
    
    def create_grouped_process(self, frequency: float):
        """指定周波数の統合プロセスを作成"""
        
        robots_in_group = self.frequency_groups[frequency]
        if not robots_in_group:
            return
        
        print(f"🔄 Creating grouped process for {frequency}Hz ({len(robots_in_group)} robots)")
        
        def grouped_frequency_process():
            """統合された周波数プロセス - 同じ周波数の全ロボットを一括処理"""
            
            dt = 1.0 / frequency
            
            while True:
                # 同じ周波数の全ロボットを一度に処理
                for robot_info in robots_in_group:
                    try:
                        robot_info['controller'](dt)
                        self.total_callbacks += 1
                        self.frequency_callback_counts[frequency] += 1
                    except Exception as e:
                        print(f"❌ Controller error for {robot_info['name']}: {e}")
                
                # 一回のyieldで全ロボット処理完了
                yield self.sim.env.timeout(dt)
        
        # プロセスをSimPy環境に登録
        process = self.sim.env.process(grouped_frequency_process())
        self.group_processes[frequency] = process
        
        return process
    
    def start_all_grouped_processes(self):
        """全周波数グループのプロセスを開始"""
        
        print(f"🚀 Starting frequency-grouped processes...")
        active_frequencies = list(self.frequency_groups.keys())
        
        print(f"Frequency groups:")
        total_robots = 0
        for freq in sorted(active_frequencies):
            count = len(self.frequency_groups[freq])
            total_robots += count
            print(f"   {freq:5.1f} Hz: {count:3d} robots")
        
        print(f"Process optimization:")
        print(f"   Traditional: {total_robots} processes (1 per robot)")
        print(f"   Grouped: {len(active_frequencies)} processes (1 per frequency)")
        print(f"   Reduction: {total_robots - len(active_frequencies)} processes ({(1 - len(active_frequencies)/total_robots)*100:.1f}%)")
        
        # 各周波数のプロセスを作成・開始
        for frequency in active_frequencies:
            self.create_grouped_process(frequency)
        
        return len(active_frequencies)
    
    def get_performance_stats(self):
        """パフォーマンス統計を取得"""
        return {
            'total_callbacks': self.total_callbacks,
            'frequency_counts': dict(self.frequency_callback_counts),
            'active_frequencies': list(self.frequency_groups.keys()),
            'robots_per_frequency': {freq: len(robots) for freq, robots in self.frequency_groups.items()},
            'total_processes': len(self.group_processes),
            'total_robots': sum(len(robots) for robots in self.frequency_groups.values())
        }
    
    def run(self, duration: float):
        """シミュレーション実行"""
        return self.sim.run(duration=duration)
    
    def shutdown(self):
        """シャットダウン"""
        try:
            self.sim.shutdown()
        except:
            pass

def create_frequency_grouped_test(num_robots=NUM_ROBOTS):
    """周波数グループ化テストを作成"""
    
    print(f"🎯 Frequency-Grouped Performance Test")
    print(f"{'='*60}")
    print(f"Robots: {num_robots}")
    print(f"Frequency groups: {FREQUENCY_GROUPS}")
    print(f"Optimization: Group robots by frequency into shared processes")
    print(f"{'='*60}")
    
    # 高速化設定
    config = SimulationConfig(
        visualization=False,  # ヘッドレスで最高性能
        update_rate=50.0,     # ベース更新レート
        real_time_factor=0.0  # 最高速度
    )
    
    grouped_sim = FrequencyGroupedSimulation(config)
    
    try:
        print(f"🏗️ Creating {num_robots} robots with frequency grouping...")
        
        # ロボット作成と周波数グループへの配置
        robots_created = 0
        
        for i in range(num_robots):
            # グリッド配置
            grid_size = int(math.ceil(math.sqrt(num_robots)))
            x = (i % grid_size) * 2.0
            y = (i // grid_size) * 2.0
            
            robot_name = f"robot_{i:03d}"
            
            # 通常のロボット作成（unified_process=Falseで個別プロセス作成を回避）
            robot = grouped_sim.sim.add_robot_from_urdf(
                name=robot_name,
                urdf_path="examples/robots/mobile_robot.urdf",
                initial_pose=Pose(x=x, y=y, z=0),
                joint_update_rate=1.0,  # 低頻度でロボット内部処理を軽量化
                unified_process=True  # 個別のロボットプロセスは作成
            )
            
            # 周波数を循環的に割り当て
            frequency = FREQUENCY_GROUPS[i % len(FREQUENCY_GROUPS)]
            
            # 効率的なコントローラ作成
            def create_optimized_controller(robot_name, robot_id, frequency):
                # Velocityオブジェクトをキャッシュ
                velocity_cache = {
                    'forward': Velocity(linear_x=0.6, angular_z=0),
                    'left': Velocity(linear_x=0.4, angular_z=0.5),
                    'right': Velocity(linear_x=0.4, angular_z=-0.5),
                    'stop': Velocity(linear_x=0, angular_z=0)
                }
                
                def optimized_controller(dt):
                    # 高速パターン選択（ビット演算）
                    t = grouped_sim.sim.get_sim_time()
                    pattern = (robot_id + int(t * frequency)) & 3  # % 4 の高速版
                    
                    # キャッシュされたvelocityを使用
                    if pattern == 0:
                        velocity = velocity_cache['forward']
                    elif pattern == 1:
                        velocity = velocity_cache['left']
                    elif pattern == 2:
                        velocity = velocity_cache['right']
                    else:
                        velocity = velocity_cache['stop']
                    
                    grouped_sim.sim.set_robot_velocity(robot_name, velocity)
                
                return optimized_controller
            
            controller = create_optimized_controller(robot_name, i, frequency)
            
            # ロボットを周波数グループに追加（SimulationManager の callback は使わない）
            grouped_sim.add_robot_to_frequency_group(robot_name, robot, controller, frequency)
            
            robots_created += 1
            
            # 進捗表示
            if num_robots > 50 and (i + 1) % (num_robots//5) == 0:
                print(f"   Created {i+1}/{num_robots} robots...")
        
        print(f"✅ All {robots_created} robots created and grouped!")
        
        # 周波数グループ化プロセスを開始
        active_processes = grouped_sim.start_all_grouped_processes()
        
        print(f"\n🚀 Starting optimized simulation...")
        
        # システムリソース監視
        try:
            import psutil
            process = psutil.Process()
            start_memory = process.memory_info().rss / 1024 / 1024
            start_threads = len(process.threads())
            print(f"System state: {start_memory:.1f}MB memory, {start_threads} threads")
        except ImportError:
            start_memory = start_threads = 0
        
        # 実行時間測定
        wall_start = time.time()
        
        # シミュレーション実行
        grouped_sim.run(duration=DURATION)
        
        # 結果計算
        wall_elapsed = time.time() - wall_start
        sim_time = grouped_sim.sim.get_sim_time()
        stats = grouped_sim.get_performance_stats()
        
        # 最終システム状態
        try:
            end_memory = process.memory_info().rss / 1024 / 1024
            end_threads = len(process.threads())
        except:
            end_memory = end_threads = 0
        
        print(f"\n🏆 FREQUENCY-GROUPED PERFORMANCE RESULTS")
        print(f"{'='*60}")
        
        print(f"Scale & Architecture:")
        print(f"   Total robots: {stats['total_robots']}")
        print(f"   Frequency groups: {len(stats['active_frequencies'])}")
        print(f"   Active processes: {stats['total_processes']}")
        print(f"   Process reduction: {stats['total_robots'] - stats['total_processes']} "
              f"({(1 - stats['total_processes']/stats['total_robots'])*100:.1f}%)")
        
        print(f"\nPerformance:")
        print(f"   Simulation time: {sim_time:.2f}s")
        print(f"   Wall clock time: {wall_elapsed:.2f}s")
        print(f"   Speed multiplier: {sim_time/wall_elapsed:.1f}x")
        print(f"   Total callbacks: {stats['total_callbacks']:,}")
        print(f"   Callback rate: {stats['total_callbacks']/wall_elapsed:.0f} callbacks/sec")
        print(f"   Per-robot rate: {stats['total_callbacks']/wall_elapsed/stats['total_robots']:.1f} Hz")
        
        print(f"\nFrequency Distribution:")
        for freq in sorted(stats['active_frequencies']):
            robot_count = stats['robots_per_frequency'][freq]
            callback_count = stats['frequency_counts'][freq]
            expected_callbacks = freq * sim_time * robot_count
            efficiency = callback_count / expected_callbacks * 100 if expected_callbacks > 0 else 0
            print(f"   {freq:5.1f} Hz: {robot_count:3d} robots, {callback_count:6,} callbacks ({efficiency:.1f}% efficiency)")
        
        if start_memory > 0:
            print(f"\nResource Usage:")
            print(f"   Memory: {start_memory:.1f} → {end_memory:.1f}MB (Δ{end_memory-start_memory:+.1f})")
            print(f"   Threads: {start_threads} → {end_threads} (Δ{end_threads-start_threads:+d})")
        
        # パフォーマンス評価
        per_robot_hz = stats['total_callbacks'] / wall_elapsed / stats['total_robots']
        process_efficiency = stats['total_callbacks'] / stats['total_processes'] / wall_elapsed
        
        if per_robot_hz > 20.0:
            rating = "🚀 ULTRA-FAST"
        elif per_robot_hz > 15.0:
            rating = "⚡ EXCELLENT"
        elif per_robot_hz > 10.0:
            rating = "✅ VERY GOOD"
        elif per_robot_hz > 5.0:
            rating = "✅ GOOD"
        else:
            rating = "⚠️ NEEDS OPTIMIZATION"
        
        print(f"\n{rating}: {per_robot_hz:.1f} Hz per robot")
        print(f"Process efficiency: {process_efficiency:.1f} callbacks/process/sec")
        
        return {
            'num_robots': stats['total_robots'],
            'num_processes': stats['total_processes'],
            'total_callbacks': stats['total_callbacks'],
            'simulation_time': sim_time,
            'wall_time': wall_elapsed,
            'per_robot_hz': per_robot_hz,
            'process_reduction_percent': (1 - stats['total_processes']/stats['total_robots'])*100,
            'process_efficiency': process_efficiency
        }
        
    except Exception as e:
        print(f"❌ Frequency-grouped test failed: {e}")
        import traceback
        traceback.print_exc()
        return None
        
    finally:
        grouped_sim.shutdown()

def compare_architectures():
    """従来手法と周波数グループ化手法の比較"""
    
    print(f"🏁 ARCHITECTURE COMPARISON")
    print(f"{'='*80}")
    
    test_configs = [
        ("Traditional Individual", 50, False),
        ("Unified Process", 50, True),  
        ("Frequency-Grouped", 50, "grouped"),
        ("Large Scale Grouped", 100, "grouped"),
    ]
    
    results = []
    
    for name, num_robots, architecture in test_configs:
        print(f"\n⏳ Testing {name} ({num_robots} robots)...")
        
        try:
            if architecture == "grouped":
                # 周波数グループ化テスト
                global NUM_ROBOTS
                original_count = NUM_ROBOTS
                NUM_ROBOTS = num_robots
                result = create_frequency_grouped_test(num_robots)
                NUM_ROBOTS = original_count
            else:
                # 従来手法のテスト
                result = run_traditional_test(num_robots, architecture)
            
            if result:
                results.append((name, result))
                print(f"✅ {name}: {result['per_robot_hz']:.1f} Hz/robot")
            
            time.sleep(2.0)
            
        except KeyboardInterrupt:
            print("\n⏹️ Testing interrupted")
            break
        except Exception as e:
            print(f"❌ {name} failed: {e}")
    
    # 比較結果の表示
    if len(results) >= 2:
        print(f"\n🏆 ARCHITECTURE COMPARISON RESULTS")
        print(f"{'='*80}")
        print(f"{'Architecture':<25} {'Robots':<7} {'Processes':<10} {'Hz/Robot':<10} {'Efficiency':<12}")
        print("-"*80)
        
        for name, result in results:
            processes = result.get('num_processes', result['num_robots'])
            efficiency = result.get('process_efficiency', result['per_robot_hz'])
            print(f"{name:<25} {result['num_robots']:<7} {processes:<10} "
                  f"{result['per_robot_hz']:<10.1f} {efficiency:<12.1f}")
        
        # 最適化効果の計算
        if len(results) >= 3:
            traditional = next((r[1] for r in results if "Traditional" in r[0]), None)
            grouped = next((r[1] for r in results if "Frequency-Grouped" in r[0]), None)
            
            if traditional and grouped:
                performance_improvement = (grouped['per_robot_hz'] / traditional['per_robot_hz'] - 1) * 100
                process_reduction = grouped.get('process_reduction_percent', 0)
                
                print(f"\n🎯 FREQUENCY-GROUPING BENEFITS:")
                print(f"   Performance improvement: {performance_improvement:+.1f}%")
                print(f"   Process reduction: {process_reduction:.1f}%")
                print(f"   Scalability: Better with larger robot counts")

def run_traditional_test(num_robots, unified_process):
    """従来手法での比較テスト"""
    
    config = SimulationConfig(
        visualization=False,
        update_rate=50.0,
        real_time_factor=0.0
    )
    
    sim = SimulationManager(config)
    commands = 0
    
    try:
        # ロボット作成
        for i in range(num_robots):
            robot = sim.add_robot_from_urdf(
                f"robot_{i}",
                "examples/robots/mobile_robot.urdf",
                Pose((i%10)*2, (i//10)*2, 0),
                unified_process=unified_process
            )
            
            def controller(dt, robot_id=i):
                nonlocal commands
                commands += 1
                velocity = Velocity(linear_x=0.5, angular_z=0.2)
                sim.set_robot_velocity(f"robot_{robot_id}", velocity)
            
            frequency = FREQUENCY_GROUPS[i % len(FREQUENCY_GROUPS)]
            sim.set_robot_control_callback(f"robot_{i}", controller, frequency=frequency)
        
        start = time.time()
        sim.run(duration=DURATION)
        elapsed = time.time() - start
        
        return {
            'num_robots': num_robots,
            'total_callbacks': commands,
            'wall_time': elapsed,
            'per_robot_hz': commands / elapsed / num_robots
        }
        
    finally:
        try:
            sim.shutdown()
        except:
            pass

def main():
    """メイン実行"""
    print("🎯 Frequency-Grouped SimPyROS Performance Test")
    print("Revolutionary optimization: Group robots by frequency into shared processes")
    
    try:
        # 単体の周波数グループ化テスト
        print("\n1️⃣ Primary Frequency-Grouped Test")
        create_frequency_grouped_test(NUM_ROBOTS)
        
        time.sleep(3)
        
        # アーキテクチャ比較
        print("\n2️⃣ Architecture Comparison")
        compare_architectures()
        
        print(f"\n💡 KEY OPTIMIZATION:")
        print(f"   Traditional: N processes (1 per robot)")
        print(f"   Frequency-Grouped: F processes (1 per frequency)")  
        print(f"   Where F << N, dramatically reducing SimPy overhead")
        print(f"   Perfect for scenarios with repeated frequencies!")
        
    except KeyboardInterrupt:
        print("\n⏹️ Testing interrupted")
    except Exception as e:
        print(f"❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()