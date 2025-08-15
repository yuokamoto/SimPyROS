#!/usr/bin/env python3
"""
Performance Profiling Tool for SimPyROS

Identifies bottlenecks and optimization opportunities in the simulation.

Usage:
    python profile_performance.py
"""

import sys
import os
import time
import cProfile
import pstats
import io
from contextlib import contextmanager

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose

@contextmanager
def profile_context(description="Profile"):
    """プロファイリングコンテキストマネージャ"""
    print(f"🔍 Starting {description}...")
    
    profiler = cProfile.Profile()
    start_time = time.time()
    
    profiler.enable()
    try:
        yield profiler
    finally:
        profiler.disable()
        elapsed = time.time() - start_time
        
        print(f"✅ {description} completed in {elapsed:.2f}s")
        
        # 結果をバッファに出力
        buffer = io.StringIO()
        stats = pstats.Stats(profiler, stream=buffer)
        stats.sort_stats('cumulative')
        stats.print_stats(20)  # 上位20関数
        
        print(f"\n📊 Top functions in {description}:")
        print(buffer.getvalue())

def profile_robot_creation(num_robots=50):
    """ロボット作成のプロファイリング"""
    with profile_context("Robot Creation"):
        config = SimulationConfig(
            visualization=False,
            update_rate=50.0,
            real_time_factor=0.0
        )
        
        sim = SimulationManager(config)
        
        try:
            robots = []
            for i in range(num_robots):
                robot = sim.add_robot_from_urdf(
                    f"robot_{i:03d}",
                    "examples/robots/mobile_robot.urdf",
                    Pose(x=i*2, y=0, z=0),
                    unified_process=True
                )
                robots.append(robot)
            
            return len(robots)
            
        finally:
            try:
                sim.shutdown()
            except:
                pass

def profile_simulation_execution(num_robots=20, duration=5.0):
    """シミュレーション実行のプロファイリング"""
    config = SimulationConfig(
        visualization=False,
        update_rate=100.0,
        real_time_factor=0.0
    )
    
    sim = SimulationManager(config)
    commands_executed = 0
    
    try:
        print(f"Setting up {num_robots} robots for execution profiling...")
        
        # ロボット作成
        robots = []
        for i in range(num_robots):
            robot = sim.add_robot_from_urdf(
                f"robot_{i:03d}",
                "examples/robots/mobile_robot.urdf", 
                Pose(x=(i%10)*2, y=(i//10)*2, z=0),
                unified_process=True
            )
            robots.append((f"robot_{i:03d}", robot, i))
        
        # コントローラ設定
        def create_controller(robot_name, robot_id):
            def controller(dt):
                nonlocal commands_executed
                commands_executed += 1
                
                # シンプルな制御
                t = sim.get_sim_time()
                velocity = Velocity(
                    linear_x=0.5,
                    angular_z=0.3 * (1 if robot_id % 2 == 0 else -1)
                )
                sim.set_robot_velocity(robot_name, velocity)
            
            return controller
        
        for robot_name, robot_instance, robot_id in robots:
            controller = create_controller(robot_name, robot_id)
            sim.set_robot_control_callback(robot_name, controller, frequency=15.0)
        
        # 実行をプロファイリング
        with profile_context("Simulation Execution"):
            sim.run(duration=duration)
        
        return commands_executed
        
    finally:
        try:
            sim.shutdown()
        except:
            pass

def analyze_memory_usage(num_robots=30):
    """メモリ使用量の分析"""
    try:
        import psutil
        import gc
    except ImportError:
        print("⚠️ psutil not available for memory analysis")
        return
    
    print(f"🧠 Memory Usage Analysis ({num_robots} robots)")
    print("="*50)
    
    process = psutil.Process()
    
    # 初期状態
    gc.collect()
    initial_memory = process.memory_info().rss / 1024 / 1024
    print(f"Initial memory: {initial_memory:.1f} MB")
    
    config = SimulationConfig(
        visualization=False,
        update_rate=50.0,
        real_time_factor=0.0
    )
    
    sim = SimulationManager(config)
    
    try:
        # SimulationManager作成後
        after_sim_memory = process.memory_info().rss / 1024 / 1024
        print(f"After SimulationManager: {after_sim_memory:.1f} MB (+{after_sim_memory-initial_memory:.1f})")
        
        # ロボット作成中のメモリ追跡
        robots = []
        for i in range(num_robots):
            robot = sim.add_robot_from_urdf(
                f"robot_{i:03d}",
                "examples/robots/mobile_robot.urdf",
                Pose(x=i*2, y=0, z=0),
                unified_process=True
            )
            robots.append(robot)
            
            # 10台ごとにメモリチェック
            if (i + 1) % 10 == 0:
                current_memory = process.memory_info().rss / 1024 / 1024
                print(f"After {i+1:2d} robots: {current_memory:.1f} MB (+{current_memory-after_sim_memory:.1f})")
        
        # 最終メモリ使用量
        final_memory = process.memory_info().rss / 1024 / 1024
        per_robot_memory = (final_memory - after_sim_memory) / num_robots
        
        print(f"\nMemory Summary:")
        print(f"   Total increase: {final_memory - initial_memory:.1f} MB")
        print(f"   Per robot: {per_robot_memory:.2f} MB")
        print(f"   Base overhead: {after_sim_memory - initial_memory:.1f} MB")
        
        # ガベージコレクション効果
        gc.collect()
        after_gc_memory = process.memory_info().rss / 1024 / 1024
        gc_savings = final_memory - after_gc_memory
        print(f"   GC savings: {gc_savings:.1f} MB")
        
    finally:
        try:
            sim.shutdown()
        except:
            pass

def benchmark_different_configurations():
    """異なる設定でのベンチマーク比較"""
    print(f"🏁 Configuration Benchmark Comparison")
    print("="*60)
    
    configs = [
        ("Low Freq", 20, 5.0, 3.0),    # 20台, 5Hz, 3秒
        ("Medium", 30, 10.0, 3.0),     # 30台, 10Hz, 3秒
        ("High Freq", 40, 20.0, 3.0),  # 40台, 20Hz, 3秒
        ("Many Robots", 60, 10.0, 3.0), # 60台, 10Hz, 3秒
    ]
    
    results = []
    
    for name, num_robots, frequency, duration in configs:
        print(f"\n⏳ Testing {name} ({num_robots} robots @ {frequency}Hz)...")
        
        try:
            start_time = time.time()
            commands = run_benchmark_test(num_robots, frequency, duration)
            elapsed = time.time() - start_time
            
            commands_per_sec = commands / elapsed if elapsed > 0 else 0
            per_robot_hz = commands_per_sec / num_robots
            
            result = {
                'name': name,
                'robots': num_robots,
                'frequency': frequency,
                'duration': duration,
                'commands': commands,
                'elapsed': elapsed,
                'commands_per_sec': commands_per_sec,
                'per_robot_hz': per_robot_hz
            }
            
            results.append(result)
            print(f"   {commands:,} commands in {elapsed:.2f}s = {per_robot_hz:.1f} Hz/robot")
            
        except Exception as e:
            print(f"   ❌ Failed: {e}")
    
    # 結果比較
    if results:
        print(f"\n🏆 BENCHMARK RESULTS")
        print("="*70)
        print(f"{'Config':<15} {'Robots':<7} {'Freq':<6} {'Commands/s':<12} {'Hz/Robot':<10}")
        print("-"*70)
        
        for result in results:
            print(f"{result['name']:<15} {result['robots']:<7} {result['frequency']:<6.1f} "
                  f"{result['commands_per_sec']:<12.0f} {result['per_robot_hz']:<10.1f}")
        
        # 最高パフォーマンスの特定
        best = max(results, key=lambda x: x['per_robot_hz'])
        print(f"\n🌟 Best performance: {best['name']} at {best['per_robot_hz']:.1f} Hz/robot")

def run_benchmark_test(num_robots, frequency, duration):
    """ベンチマーク用の単純テスト"""
    config = SimulationConfig(
        visualization=False,
        update_rate=max(50.0, frequency * 2),  # 周波数の2倍以上
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
                Pose(x=(i%10)*2, y=(i//10)*2, z=0),
                unified_process=True
            )
            
            def controller(dt, robot_id=i):
                nonlocal commands
                commands += 1
                
                velocity = Velocity(
                    linear_x=0.5,
                    angular_z=0.2 * (-1 if robot_id % 2 else 1)
                )
                sim.set_robot_velocity(f"robot_{robot_id}", velocity)
            
            sim.set_robot_control_callback(f"robot_{i}", controller, frequency=frequency)
        
        # 実行
        sim.run(duration=duration)
        return commands
        
    finally:
        try:
            sim.shutdown()
        except:
            pass

def main():
    """メインプロファイリング実行"""
    print("🔍 SimPyROS Performance Profiling Suite")
    print("Analyzing performance bottlenecks and optimization opportunities")
    print("="*80)
    
    try:
        # 1. ロボット作成のプロファイリング
        print("\n1️⃣ Robot Creation Profiling")
        robots_created = profile_robot_creation(30)
        print(f"Successfully created {robots_created} robots")
        
        time.sleep(2)
        
        # 2. シミュレーション実行のプロファイリング
        print("\n2️⃣ Simulation Execution Profiling")
        commands = profile_simulation_execution(15, 4.0)
        print(f"Executed {commands:,} commands")
        
        time.sleep(2)
        
        # 3. メモリ使用量分析
        print("\n3️⃣ Memory Usage Analysis")
        analyze_memory_usage(25)
        
        time.sleep(2)
        
        # 4. 設定比較ベンチマーク
        print("\n4️⃣ Configuration Benchmark")
        benchmark_different_configurations()
        
        print(f"\n🎯 OPTIMIZATION RECOMMENDATIONS")
        print("="*50)
        print("Based on profiling results:")
        print("✅ Use unified_process=True for better efficiency")
        print("✅ Optimize control callback frequency vs. performance needs")
        print("✅ Monitor memory usage with large robot counts")
        print("✅ Consider batch processing for >50 robots")
        print("✅ Use headless mode for maximum performance")
        
    except KeyboardInterrupt:
        print("\n⏹️ Profiling interrupted")
    except Exception as e:
        print(f"❌ Profiling failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()