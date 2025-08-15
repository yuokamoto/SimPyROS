#!/usr/bin/env python3
"""
Test visualization=True with callbacks after fix
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig


def test_visualization_callback():
    """Test if callbacks work with visualization=True after fix"""
    print("🧪 Testing visualization=True with callbacks...")
    
    config = SimulationConfig(
        real_time_factor=1.0,
        visualization=True,  # TEST: This should now work!
        enable_frequency_grouping=False,
        update_rate=10.0  # Lower rate for testing
    )
    
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            name="test_robot",
            urdf_path="examples/robots/mobile_robot.urdf",
            unified_process=False
        )
        
        callback_count = 0
        
        def test_callback(dt: float):
            nonlocal callback_count
            callback_count += 1
            print(f"✅ VISUALIZATION CALLBACK #{callback_count}: dt={dt:.4f}s")
            
            # Simple movement for visual confirmation
            from core.simulation_object import Velocity
            velocity = Velocity(linear_x=0.3, angular_z=0.2)
            sim.set_robot_velocity("test_robot", velocity)
        
        print("🎮 Setting callback with visualization=True...")
        sim.set_robot_control_callback("test_robot", test_callback, frequency=10.0)
        
        print("🚀 Running for 3 seconds with visualization...")
        sim.run(duration=3.0, auto_close=True)
        
        print(f"🎉 SUCCESS! Total callbacks with visualization: {callback_count}")
        if callback_count > 0:
            print("✅ Visualization + Callbacks working together!")
        else:
            print("❌ Still not working - callbacks not executed")
        
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
    test_visualization_callback()