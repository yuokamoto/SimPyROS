#!/usr/bin/env python3
"""
Test only mobile robot from basic_simulation.py
"""

import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose


def mobile_robot_example(unified_process=False):
    """Example 2: Mobile robot with auto-close"""
    print("🚗 Mobile Robot Example")
    print("=" * 40)
    
    # Explicitly set real-time factor to 1.0 for accurate timing
    from core.simulation_manager import SimulationConfig
    config = SimulationConfig(
        real_time_factor=1.0,  # Ensure 1:1 real-time synchronization
        visualization=False,  # TEST: Disable visualization to match working test
        enable_frequency_grouping=False  # Disable frequency grouping to test individual processes
    )
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            name="mobile_robot", 
            urdf_path="examples/robots/mobile_robot.urdf",
            unified_process=unified_process  # Use parameter from function call
        )
        
        # Debug: Mobile robot callback counter
        mobile_callback_count = 0
        
        def mobile_control(dt: float):
            """Move robot in circle"""
            nonlocal mobile_callback_count
            mobile_callback_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
            
            # Print debug info for first 10 calls, then every 50 calls
            if mobile_callback_count <= 10 or mobile_callback_count % 50 == 0:
                print(f"🚗 Mobile Callback #{mobile_callback_count}: t={t:.2f}s, dt={dt:.4f}s")
                if mobile_callback_count <= 3:
                    print(f"   Setting velocity: linear_x=0.5, angular_z=0.3")
            
            linear_speed = 0.5
            angular_speed = 0.3
            
            velocity = Velocity(
                linear_x=linear_speed,
                angular_z=angular_speed
            )
            sim.set_robot_velocity("mobile_robot", velocity)
        
        sim.set_robot_control_callback("mobile_robot", mobile_control, frequency=10.0)
        sim.run(duration=5.0, auto_close=True)
        
        print(f"✅ Mobile robot completed. Total callbacks: {mobile_callback_count}")
        
    except Exception as e:
        print(f"⚠️ Example error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            sim.shutdown()
        except:
            pass


if __name__ == "__main__":
    print("🧪 Testing mobile robot only...")
    mobile_robot_example(unified_process=False)