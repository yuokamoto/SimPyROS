#!/usr/bin/env python3
"""
Configurable Multi-Robot Performance Test - SimPyROS

Simple test script to evaluate performance with configurable robot count.

Usage:
    python configurable_performance_test.py
    
Modify NUM_ROBOTS variable to test different scales.
"""

import sys
import os
import time
import math

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose

# ==============================
# 設定可能パラメータ
# ==============================
NUM_ROBOTS = 20       # ロボット台数（1-100推奨）
VISUALIZATION = False  # True=可視化あり、False=ヘッドレス（高速）
DURATION = 10.0       # シミュレーション時間（秒）
UNIFIED_PROCESS = True # True=統合プロセス、False=従来マルチプロセス

def run_configurable_performance_test():
    """設定可能なパフォーマンステスト"""
    
    print(f"🚀 SimPyROS Performance Test")
    print(f"{'='*50}")
    print(f"Robot count: {NUM_ROBOTS}")
    print(f"Visualization: {'ON' if VISUALIZATION else 'OFF (Headless)'}")
    print(f"Architecture: {'Unified Event-Driven' if UNIFIED_PROCESS else 'Multi-Process Legacy'}")
    print(f"Duration: {DURATION}s")
    print(f"{'='*50}")
    
    # シミュレーション設定
    config = SimulationConfig(
        visualization=VISUALIZATION,
        update_rate=30.0,
        real_time_factor=1.0 if VISUALIZATION else 0.0  # 可視化時は1.0x、ヘッドレス時は最高速度
    )
    
    sim = SimulationManager(config)
    
    try:
        print(f"🏗️ Creating {NUM_ROBOTS} robots...")
        robots = []
        
        # グリッド配置の計算
        grid_size = int(math.ceil(math.sqrt(NUM_ROBOTS)))
        
        # ロボット作成
        for i in range(NUM_ROBOTS):
            x = (i % grid_size) * 2.0  # 2m間隔
            y = (i // grid_size) * 2.0
            
            robot_name = f"robot_{i:03d}"
            
            robot = sim.add_robot_from_urdf(
                name=robot_name,
                urdf_path="examples/robots/mobile_robot.urdf",  # 軽量なモバイルロボット
                initial_pose=Pose(x=x, y=y, z=0),
                joint_update_rate=15.0,
                unified_process=UNIFIED_PROCESS
            )
            robots.append((robot_name, robot, i))
            
            # 進捗表示
            if NUM_ROBOTS > 20 and (i + 1) % (NUM_ROBOTS//5) == 0:
                print(f"   Created {i+1}/{NUM_ROBOTS} robots...")
        
        print(f"✅ All {NUM_ROBOTS} robots created!")
        
        # パフォーマンス追跡
        total_callbacks = 0
        start_time = time.time()
        
        def create_simple_controller(robot_name, robot_id):
            """シンプルなロボット制御"""
            def controller(dt):
                nonlocal total_callbacks
                total_callbacks += 1
                
                t = sim.get_sim_time()
                
                # 3つの動作パターン
                pattern = robot_id % 3
                
                if pattern == 0:  # 円運動
                    velocity = Velocity(
                        linear_x=0.5,
                        angular_z=0.3 * math.sin(t * 0.5 + robot_id * 0.1)
                    )
                elif pattern == 1:  # 直進とターン
                    cycle = int(t + robot_id * 0.2) % 8
                    if cycle < 6:
                        velocity = Velocity(linear_x=0.6, angular_z=0)
                    else:
                        velocity = Velocity(linear_x=0, angular_z=0.5)
                else:  # ランダムウォーク
                    velocity = Velocity(
                        linear_x=0.4,
                        angular_z=0.4 * math.sin(t * 0.7 + robot_id * 0.3)
                    )
                
                sim.set_robot_velocity(robot_name, velocity)
            
            return controller
        
        # コントロールコールバック設定
        print("🎮 Setting up robot controllers...")
        for robot_name, robot_instance, robot_id in robots:
            controller = create_simple_controller(robot_name, robot_id)
            sim.set_robot_control_callback(robot_name, controller, frequency=10.0)
        
        print(f"🚀 Starting simulation...")
        print(f"Expected processes: {NUM_ROBOTS if UNIFIED_PROCESS else NUM_ROBOTS*4}")
        print(f"Running for {DURATION}s...")
        
        # システム情報記録
        try:
            import psutil
            process = psutil.Process()
            start_memory = process.memory_info().rss / 1024 / 1024
            start_threads = len(process.threads())
            print(f"Initial: {start_memory:.1f}MB memory, {start_threads} threads")
        except ImportError:
            start_memory = start_threads = 0
        
        # シミュレーション実行
        sim.run(duration=DURATION)
        
        # 結果計算
        elapsed_time = time.time() - start_time
        sim_time = sim.get_sim_time()
        
        # 終了システム状態
        try:
            end_memory = process.memory_info().rss / 1024 / 1024
            end_threads = len(process.threads())
        except:
            end_memory = end_threads = 0
        
        # 結果表示
        print(f"\n📊 PERFORMANCE RESULTS")
        print(f"{'='*40}")
        print(f"Simulation:")
        print(f"   Sim time: {sim_time:.2f}s")
        print(f"   Wall time: {elapsed_time:.2f}s")
        print(f"   Speed factor: {sim_time/elapsed_time:.2f}x")
        
        print(f"\nRobot Control:")
        print(f"   Total callbacks: {total_callbacks:,}")
        print(f"   Avg rate: {total_callbacks/elapsed_time:.1f} Hz")
        print(f"   Per-robot: {total_callbacks/elapsed_time/NUM_ROBOTS:.1f} Hz")
        
        if start_memory > 0:
            print(f"\nResources:")
            print(f"   Memory: {start_memory:.1f} → {end_memory:.1f}MB (Δ{end_memory-start_memory:+.1f})")
            print(f"   Threads: {start_threads} → {end_threads} (Δ{end_threads-start_threads:+d})")
        
        # パフォーマンス評価
        per_robot_hz = total_callbacks / elapsed_time / NUM_ROBOTS
        if per_robot_hz > 8.0:
            rating = "🌟 EXCELLENT"
        elif per_robot_hz > 5.0:
            rating = "✅ GOOD"  
        elif per_robot_hz > 3.0:
            rating = "⚠️ FAIR"
        else:
            rating = "❌ POOR"
            
        print(f"\n{rating}: {per_robot_hz:.1f} Hz per robot")
        print(f"Architecture: {'Unified Event-Driven' if UNIFIED_PROCESS else 'Multi-Process Legacy'}")
        
        # タイミング統計
        timing_stats = sim.get_timing_stats()
        if timing_stats:
            print(f"\nTiming Details:")
            for key, value in timing_stats.items():
                print(f"   {key}: {value}")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            sim.shutdown()
        except:
            pass

def main():
    """メイン実行関数"""
    print("🧪 Configurable Multi-Robot Performance Test")
    print(f"Edit NUM_ROBOTS variable to test different scales (current: {NUM_ROBOTS})")
    print()
    
    try:
        run_configurable_performance_test()
    except KeyboardInterrupt:
        print("\n⏹️ Test interrupted by user")
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
    
    print(f"\n✅ Performance test complete!")
    print(f"\n💡 To test different configurations:")
    print(f"   - Change NUM_ROBOTS (line 20) for different scales")
    print(f"   - Set VISUALIZATION=True for visual confirmation")
    print(f"   - Set UNIFIED_PROCESS=False to compare architectures")

if __name__ == "__main__":
    main()