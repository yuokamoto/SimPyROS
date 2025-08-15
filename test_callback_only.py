#!/usr/bin/env python3
"""
Test ONLY callback execution without any robot movement
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig


def test_callback_only():
    """Test if callbacks are executed at all"""
    print("🧪 Testing callback execution only...")
    
    config = SimulationConfig(
        real_time_factor=1.0,
        visualization=False,  # No visualization to focus on callbacks
        enable_frequency_grouping=False
    )
    
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            name="test_robot",
            urdf_path="examples/robots/mobile_robot.urdf",
            unified_process=False
        )
        
        callback_count = 0
        
        def simple_callback(dt: float):
            nonlocal callback_count
            callback_count += 1
            print(f"📞 CALLBACK EXECUTED #{callback_count}: dt={dt:.4f}s")
            
            # Don't do anything else - just print
        
        print("🎮 Setting callback...")
        sim.set_robot_control_callback("test_robot", simple_callback, frequency=10.0)
        
        print("🚀 Running for 2 seconds...")
        sim.run(duration=2.0, auto_close=True)
        
        print(f"✅ Test completed. Total callbacks executed: {callback_count}")
        if callback_count == 0:
            print("❌ NO CALLBACKS WERE EXECUTED!")
        else:
            print(f"✅ Callbacks working! Expected ~20, got {callback_count}")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            sim.shutdown()
        except:
            pass


if __name__ == "__main__":
    test_callback_only()