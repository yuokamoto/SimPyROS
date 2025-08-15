#!/usr/bin/env python3
"""
Test mobile robot movement
"""

import sys
import os
import math
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose


def test_mobile_robot():
    """Test mobile robot movement"""
    print("🚗 Testing mobile robot movement...")
    
    # Create simple config
    config = SimulationConfig(
        real_time_factor=1.0,
        visualization=True,
        enable_frequency_grouping=False
    )
    
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            name="mobile_robot", 
            urdf_path="examples/robots/mobile_robot.urdf",
            joint_update_rate=50.0,
            unified_process=False
        )
        
        callback_count = 0
        
        def mobile_control(dt: float):
            nonlocal callback_count
            callback_count += 1
            
            if callback_count <= 10 or callback_count % 50 == 0:
                t = sim.get_sim_time()
                print(f"🔄 Mobile Callback #{callback_count}: t={t:.2f}s")
            
            # Move robot in circle
            t = sim.get_sim_time()
            linear_speed = 0.5
            angular_speed = 0.3
            
            velocity = Velocity(
                linear_x=linear_speed,
                angular_z=angular_speed
            )
            sim.set_robot_velocity("mobile_robot", velocity)
        
        sim.set_robot_control_callback("mobile_robot", mobile_control, frequency=50.0)
        sim.run(duration=3.0, auto_close=True)
        
        print(f"✅ Mobile robot test completed. Total callbacks: {callback_count}")
        
    except Exception as e:
        print(f"❌ Mobile robot test failed: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        try:
            sim.shutdown()
        except:
            pass


if __name__ == "__main__":
    test_mobile_robot()