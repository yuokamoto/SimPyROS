#!/usr/bin/env python3
"""
クイック診断テスト - 問題の特定
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose


def quick_test():
    """最小限のテスト"""
    print("🔍 クイック診断テスト")
    print("=" * 30)
    
    # FrequencyGroup無効で最小テスト
    config = SimulationConfig(
        visualization=False,
        enable_frequency_grouping=False,  # まず従来方式で
        update_rate=10.0,
        real_time_factor=0.0
    )
    
    sim = SimulationManager(config)
    
    try:
        print("🤖 1台のロボットを作成...")
        start = time.time()
        
        robot = sim.add_robot_from_urdf(
            name="test_robot",
            urdf_path="examples/robots/mobile_robot.urdf",
            initial_pose=Pose(x=0, y=0, z=0),
            joint_update_rate=10.0,
            unified_process=True
        )
        
        creation_time = time.time() - start
        print(f"✅ ロボット作成完了: {creation_time:.3f}秒")
        
        # コールバック無しで実行テスト
        print("🚀 コールバック無しで1秒実行...")
        start = time.time()
        
        sim.run(duration=1.0)
        
        run_time = time.time() - start
        sim_time = sim.get_sim_time()
        
        print(f"✅ 実行完了:")
        print(f"   実行時間: {run_time:.3f}秒")
        print(f"   シミュレーション時間: {sim_time:.3f}秒")
        print(f"   速度比: {sim_time/run_time:.1f}x")
        
        if run_time > 2.0:
            print("⚠️ 異常に遅い - 基本的な問題があります")
        else:
            print("✅ 基本動作は正常")
            
    except Exception as e:
        print(f"❌ テスト失敗: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        sim.shutdown()


def test_with_callbacks():
    """コールバック付きテスト"""
    print("\n🎮 コールバック付きテスト")
    print("=" * 30)
    
    config = SimulationConfig(
        visualization=False,
        enable_frequency_grouping=False,
        update_rate=10.0,
        real_time_factor=0.0
    )
    
    sim = SimulationManager(config)
    callback_count = 0
    
    try:
        robot = sim.add_robot_from_urdf(
            name="callback_robot",
            urdf_path="examples/robots/mobile_robot.urdf",
            initial_pose=Pose(x=0, y=0, z=0),
            joint_update_rate=10.0,
            unified_process=True
        )
        
        def simple_callback(dt):
            nonlocal callback_count
            callback_count += 1
            velocity = Velocity(linear_x=0.1, angular_z=0.0)
            sim.set_robot_velocity("callback_robot", velocity)
        
        sim.set_robot_control_callback("callback_robot", simple_callback, frequency=5.0)
        
        print("🚀 コールバック付きで1秒実行...")
        start = time.time()
        
        sim.run(duration=1.0)
        
        run_time = time.time() - start
        sim_time = sim.get_sim_time()
        
        print(f"✅ コールバック付き実行完了:")
        print(f"   実行時間: {run_time:.3f}秒")
        print(f"   コールバック数: {callback_count}")
        print(f"   コールバック/秒: {callback_count/run_time:.1f}")
        
        if callback_count < 3:
            print("⚠️ コールバックが少なすぎます")
        else:
            print("✅ コールバック動作正常")
            
    except Exception as e:
        print(f"❌ コールバックテスト失敗: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        sim.shutdown()


if __name__ == "__main__":
    quick_test()
    test_with_callbacks()