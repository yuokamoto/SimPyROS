#!/usr/bin/env python3
"""
パフォーマンス比較テスト - 修正前後の速度を比較
"""

import sys
import os
import time
import gc

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose


def performance_test(enable_frequency_grouping: bool, num_robots: int = 10, duration: float = 5.0):
    """パフォーマンステスト実行"""
    
    mode_name = "FrequencyGroup" if enable_frequency_grouping else "Traditional"
    print(f"🧪 {mode_name}モード パフォーマンステスト")
    print(f"   ロボット数: {num_robots}")
    print(f"   実行時間: {duration}秒")
    print("-" * 50)
    
    # メモリクリア
    gc.collect()
    
    # シミュレーション設定
    config = SimulationConfig(
        visualization=False,  # ヘッドレス
        enable_frequency_grouping=enable_frequency_grouping,
        update_rate=50.0,  # 高頻度更新
        real_time_factor=0.0  # 最高速度
    )
    
    sim = SimulationManager(config)
    callback_count = 0
    
    try:
        # ロボット作成開始時間
        creation_start = time.time()
        
        print(f"🏗️ {num_robots}台のロボットを作成中...")
        robots = []
        
        for i in range(num_robots):
            robot = sim.add_robot_from_urdf(
                name=f"perf_robot_{i}",
                urdf_path="examples/robots/mobile_robot.urdf",
                initial_pose=Pose(x=i*1.5, y=0, z=0),
                joint_update_rate=20.0,  # 統一周波数
                unified_process=True
            )
            robots.append(robot)
            
            # 簡単なコントローラ
            def create_controller(robot_id):
                def controller(dt):
                    nonlocal callback_count
                    callback_count += 1
                    velocity = Velocity(linear_x=0.2, angular_z=0.1)
                    sim.set_robot_velocity(f"perf_robot_{robot_id}", velocity)
                return controller
            
            sim.set_robot_control_callback(f"perf_robot_{i}", create_controller(i), frequency=20.0)
        
        creation_time = time.time() - creation_start
        print(f"✅ ロボット作成完了: {creation_time:.3f}秒")
        
        # シミュレーション実行
        print(f"🚀 {duration}秒間のシミュレーション開始...")
        sim_start = time.time()
        
        sim.run(duration=duration)
        
        # 結果計算
        sim_elapsed = time.time() - sim_start
        sim_time = sim.get_sim_time()
        
        # 統計表示
        print(f"📊 {mode_name}モード結果:")
        print(f"   作成時間: {creation_time:.3f}秒")
        print(f"   実行時間: {sim_elapsed:.3f}秒")
        print(f"   シミュレーション時間: {sim_time:.3f}秒")
        print(f"   速度倍率: {sim_time/sim_elapsed:.1f}x")
        print(f"   コールバック数: {callback_count:,}")
        print(f"   コールバック/秒: {callback_count/sim_elapsed:.0f}")
        print(f"   ロボット毎Hz: {callback_count/sim_elapsed/num_robots:.1f}")
        
        return {
            'mode': mode_name,
            'creation_time': creation_time,
            'simulation_time': sim_elapsed,
            'sim_time': sim_time,
            'speed_multiplier': sim_time/sim_elapsed if sim_elapsed > 0 else 0,
            'callback_count': callback_count,
            'callback_rate': callback_count/sim_elapsed if sim_elapsed > 0 else 0,
            'per_robot_hz': callback_count/sim_elapsed/num_robots if sim_elapsed > 0 else 0
        }
        
    except Exception as e:
        print(f"❌ {mode_name}テスト失敗: {e}")
        import traceback
        traceback.print_exc()
        return None
        
    finally:
        sim.shutdown()
        time.sleep(1.0)  # クリーンアップ待機


def main():
    """メインテスト"""
    print("🏁 FrequencyGroup修正前後パフォーマンス比較")
    print("=" * 60)
    
    num_robots = 10
    duration = 3.0  # 短時間で比較
    
    try:
        # テスト1: Traditional（従来方式）
        print("\n1️⃣ Traditional Individual Process Mode")
        result_traditional = performance_test(
            enable_frequency_grouping=False, 
            num_robots=num_robots, 
            duration=duration
        )
        
        time.sleep(2.0)  # システム安定化
        
        # テスト2: FrequencyGroup（修正後）
        print("\n2️⃣ FrequencyGroup Optimized Mode")
        result_grouped = performance_test(
            enable_frequency_grouping=True, 
            num_robots=num_robots, 
            duration=duration
        )
        
        # 比較結果
        if result_traditional and result_grouped:
            print(f"\n🏆 パフォーマンス比較結果")
            print("=" * 60)
            print(f"{'メトリック':<20} {'Traditional':<15} {'FrequencyGroup':<15} {'変化':<10}")
            print("-" * 60)
            
            metrics = [
                ('作成時間', 'creation_time', '秒'),
                ('実行時間', 'simulation_time', '秒'),
                ('速度倍率', 'speed_multiplier', 'x'),
                ('コールバック/秒', 'callback_rate', 'Hz'),
                ('ロボット毎Hz', 'per_robot_hz', 'Hz')
            ]
            
            for name, key, unit in metrics:
                trad_val = result_traditional[key]
                group_val = result_grouped[key]
                
                if trad_val > 0:
                    change = ((group_val / trad_val - 1) * 100)
                    change_str = f"{change:+.1f}%"
                    if 'time' in key and change < 0:  # 時間は短い方が良い
                        change_str += " ✅"
                    elif 'time' not in key and change > 0:  # レートは高い方が良い
                        change_str += " ✅"
                    else:
                        change_str += " ⚠️"
                else:
                    change_str = "N/A"
                
                print(f"{name:<20} {trad_val:<15.3f} {group_val:<15.3f} {change_str:<10}")
            
            # 総合評価
            if result_grouped['per_robot_hz'] > result_traditional['per_robot_hz']:
                print(f"\n✅ FrequencyGroup修正により性能向上")
            else:
                print(f"\n⚠️ FrequencyGroup修正により性能低下 - 調査が必要")
                
        else:
            print(f"\n❌ 比較テスト失敗")
            
    except KeyboardInterrupt:
        print(f"\n⏹️ テスト中断")
    except Exception as e:
        print(f"\n❌ テスト全体失敗: {e}")


if __name__ == "__main__":
    main()