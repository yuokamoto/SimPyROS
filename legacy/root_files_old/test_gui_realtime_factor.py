#!/usr/bin/env python3
"""
Test GUI Real-time Factor Control

Tests if GUI slider for real-time factor works properly
"""

import sys
import os
import time
import math

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose

def test_gui_realtime_factor():
    """Test GUI real-time factor control"""
    
    print("🎮 GUI Real-time Factor Test")
    print("This test will show visualization with real-time factor controls")
    print("Use the slider in the PyVista window to change speed")
    print("=" * 60)
    
    # Create configuration with visualization enabled and initial real-time factor
    config = SimulationConfig(
        real_time_factor=1.0,  # Start at 1.0x
        visualization=True,
        visualization_update_rate=30.0
    )
    
    sim = SimulationManager(config)
    
    try:
        # Add robot
        robot = sim.add_robot_from_urdf(
            name="test_robot",
            urdf_path="examples/robots/articulated_arm_robot.urdf",
            initial_pose=Pose(x=0, y=0, z=0),
            joint_update_rate=10.0,
            unified_process=True
        )
        
        frame_count = 0
        start_time = time.time()
        
        def control_callback(dt):
            nonlocal frame_count, start_time
            frame_count += 1
            
            # Print timing info every 50 frames
            if frame_count % 50 == 0:
                elapsed_real = time.time() - start_time
                sim_time = sim.get_sim_time()
                current_factor = sim.get_realtime_factor()
                
                print(f"Frame {frame_count}: sim_time={sim_time:.2f}s, "
                      f"real_time={elapsed_real:.2f}s, "
                      f"factor={current_factor:.2f}x")
            
            # Simple joint motion
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
            joint_names = [name for name in robot.get_joint_names() 
                          if robot.joints[name].joint_type.value != 'fixed']
            
            for i, joint_name in enumerate(joint_names[:2]):  # Only first 2 joints
                position = 0.4 * math.sin(t * 2 + i * math.pi / 4)
                sim.set_robot_joint_position("test_robot", joint_name, position)
        
        sim.set_robot_control_callback("test_robot", control_callback, frequency=10.0)
        
        print("🚀 Starting simulation with GUI controls...")
        print("📊 Real-time factor slider should be visible in the PyVista window")
        print("🎚️ Move the slider to change simulation speed")
        print("⏱️ Watch the console output for timing information")
        print("🔴 Close the window or press Ctrl+C to stop")
        print()
        
        # Run simulation without duration to allow manual testing
        sim.run()
        
    except KeyboardInterrupt:
        print("\n⏹️ Test interrupted by user")
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
    test_gui_realtime_factor()