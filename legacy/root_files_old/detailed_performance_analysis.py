#!/usr/bin/env python3
"""
詳細パフォーマンス分析 - 各処理の時間測定
"""

import sys
import os
import time
import gc

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose


def detailed_timing_analysis():
    """各処理ステップの詳細時間測定"""
    
    print("🔍 詳細パフォーマンス分析")
    print("=" * 50)
    
    # Step 1: 設定作成時間
    start = time.time()
    config = SimulationConfig(
        visualization=False,
        enable_frequency_grouping=False,  # まず従来方式で測定
        update_rate=100.0,  # 高頻度に設定
        real_time_factor=0.0  # 最高速度
    )
    config_time = time.time() - start
    print(f"⏱️ Config作成: {config_time*1000:.3f}ms")
    
    # Step 2: SimulationManager作成時間
    start = time.time()
    sim = SimulationManager(config)
    manager_time = time.time() - start
    print(f"⏱️ SimulationManager作成: {manager_time*1000:.3f}ms")
    
    callback_count = 0
    
    try:
        # Step 3: Robot作成時間
        start = time.time()
        robot = sim.add_robot_from_urdf(
            name="timing_robot",
            urdf_path="examples/robots/mobile_robot.urdf",
            initial_pose=Pose(x=0, y=0, z=0),
            joint_update_rate=50.0,  # 高頻度
            unified_process=True
        )
        robot_time = time.time() - start
        print(f"⏱️ Robot作成: {robot_time*1000:.3f}ms")
        
        # Step 4: コールバック設定時間
        start = time.time()
        def high_freq_callback(dt):
            nonlocal callback_count
            callback_count += 1
            # 軽量な処理のみ
            velocity = Velocity(linear_x=0.1, angular_z=0.0)
            sim.set_robot_velocity("timing_robot", velocity)
        
        sim.set_robot_control_callback("timing_robot", high_freq_callback, frequency=50.0)
        callback_setup_time = time.time() - start
        print(f"⏱️ コールバック設定: {callback_setup_time*1000:.3f}ms")
        
        # Step 5: シミュレーション実行時間（詳細測定）
        print(f"\n🚀 高頻度シミュレーション開始...")
        
        durations = [0.1, 0.5, 1.0, 2.0]  # 異なる実行時間で測定
        
        for duration in durations:
            gc.collect()  # ガベージコレクション実行
            
            callback_count = 0
            start_time = time.time()
            
            sim.run(duration=duration)
            
            elapsed_time = time.time() - start_time
            sim_time = sim.get_sim_time()
            
            print(f"\n📊 {duration}秒実行結果:")
            print(f"   実行時間: {elapsed_time*1000:.1f}ms")
            print(f"   シミュレーション時間: {sim_time:.3f}s")
            print(f"   速度倍率: {sim_time/elapsed_time:.1f}x")
            print(f"   コールバック数: {callback_count}")
            print(f"   期待コールバック数: {int(duration * 50)}")
            print(f"   コールバック効率: {callback_count/(duration*50)*100:.1f}%")
            print(f"   実効コールバック/秒: {callback_count/elapsed_time:.1f}")
            
            if callback_count < duration * 50 * 0.9:  # 90%未満なら問題
                print(f"   ⚠️ コールバック数不足 - 処理が重い可能性")
            else:
                print(f"   ✅ コールバック数正常")
            
            time.sleep(0.1)  # システム安定化
            
        return True
        
    except Exception as e:
        print(f"❌ 分析失敗: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        sim.shutdown()


def compare_architectures_detailed():
    """従来vs FrequencyGroup詳細比較"""
    
    print(f"\n🏁 アーキテクチャ詳細比較")
    print("=" * 50)
    
    test_configs = [
        ("Traditional", False, 20.0),
        ("FrequencyGroup", True, 20.0),
    ]
    
    results = []
    
    for name, use_frequency_grouping, frequency in test_configs:
        print(f"\n🧪 {name}モード テスト")
        print("-" * 30)
        
        config = SimulationConfig(
            visualization=False,
            enable_frequency_grouping=use_frequency_grouping,
            update_rate=50.0,
            real_time_factor=0.0
        )
        
        sim = SimulationManager(config)
        callback_count = 0
        
        try:
            # 複数ロボットで測定
            num_robots = 5
            
            robots_start = time.time()
            for i in range(num_robots):
                robot = sim.add_robot_from_urdf(
                    name=f"test_robot_{i}",
                    urdf_path="examples/robots/mobile_robot.urdf",
                    initial_pose=Pose(x=i*2.0, y=0, z=0),
                    joint_update_rate=frequency,
                    unified_process=True
                )
                
                def create_callback(robot_id):
                    def callback(dt):
                        nonlocal callback_count
                        callback_count += 1
                        velocity = Velocity(linear_x=0.1, angular_z=0.0)
                        sim.set_robot_velocity(f"test_robot_{robot_id}", velocity)
                    return callback
                
                sim.set_robot_control_callback(f"test_robot_{i}", create_callback(i), frequency=frequency)
            
            robots_time = time.time() - robots_start
            
            # シミュレーション実行
            duration = 1.0
            sim_start = time.time()
            
            sim.run(duration=duration)
            
            sim_elapsed = time.time() - sim_start
            sim_time = sim.get_sim_time()
            
            expected_callbacks = int(duration * frequency * num_robots)
            efficiency = callback_count / expected_callbacks * 100 if expected_callbacks > 0 else 0
            
            result = {
                'mode': name,
                'robots_creation_time': robots_time,
                'simulation_time': sim_elapsed,
                'sim_time': sim_time,
                'callback_count': callback_count,
                'expected_callbacks': expected_callbacks,
                'efficiency': efficiency,
                'callbacks_per_sec': callback_count / sim_elapsed if sim_elapsed > 0 else 0
            }
            
            results.append(result)
            
            print(f"   ロボット作成時間: {robots_time*1000:.1f}ms")
            print(f"   シミュレーション実行時間: {sim_elapsed*1000:.1f}ms")
            print(f"   コールバック数: {callback_count}/{expected_callbacks}")
            print(f"   コールバック効率: {efficiency:.1f}%")
            print(f"   コールバック/秒: {callback_count/sim_elapsed:.1f}")
            
            if efficiency < 90:
                print(f"   ⚠️ 効率低下 - 調査が必要")
            else:
                print(f"   ✅ 正常動作")
                
        except Exception as e:
            print(f"   ❌ {name}テスト失敗: {e}")
            
        finally:
            sim.shutdown()
            time.sleep(0.5)
    
    # 比較結果
    if len(results) == 2:
        traditional = results[0]
        frequency_group = results[1]
        
        print(f"\n📊 比較結果:")
        print(f"   Traditional効率: {traditional['efficiency']:.1f}%")
        print(f"   FrequencyGroup効率: {frequency_group['efficiency']:.1f}%")
        
        if frequency_group['efficiency'] > traditional['efficiency']:
            print(f"   ✅ FrequencyGroupの方が効率的")
        else:
            print(f"   ⚠️ FrequencyGroupで効率低下")


def main():
    """メイン実行"""
    try:
        # 詳細分析
        detailed_timing_analysis()
        
        # アーキテクチャ比較
        compare_architectures_detailed()
        
    except KeyboardInterrupt:
        print(f"\n⏹️ 分析中断")
    except Exception as e:
        print(f"\n❌ 分析失敗: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()