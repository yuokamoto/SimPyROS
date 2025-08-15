#!/usr/bin/env python3
"""
Debug test for robot movement issues
"""

import sys
import os
import math
import time

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose


def test_mobile_robot_debug():
    """Test mobile robot with debug output"""
    print("🚗 Testing Mobile Robot Example with Debug...")
    print("=" * 50)
    
    # Explicitly set real-time factor to 1.0 for accurate timing
    from core.simulation_manager import SimulationConfig
    config = SimulationConfig(
        real_time_factor=1.0,  # Ensure 1:1 real-time synchronization
        visualization=True,
        enable_frequency_grouping=False  # Disable frequency grouping to test individual processes
    )
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            name="mobile_robot", 
            urdf_path="examples/robots/mobile_robot.urdf",
            joint_update_rate=50.0,  # Set appropriate update rate for mobile robot
            unified_process=False  # Use independent processes for reliable callback execution
        )
        
        print(f"✅ Mobile robot added successfully")
        
        # Debug: Mobile robot callback counter
        mobile_callback_count = 0
        
        def mobile_control(dt: float):
            """Move robot in circle"""
            nonlocal mobile_callback_count
            mobile_callback_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
            
            # Print debug info for first 10 calls, then every 25 calls
            if mobile_callback_count <= 10 or mobile_callback_count % 25 == 0:
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
        
        print("🎮 Setting mobile robot control callback...")
        sim.set_robot_control_callback("mobile_robot", mobile_control, frequency=50.0)
        
        print("🚀 Starting 5-second mobile robot test...")
        sim.run(duration=5.0, auto_close=True)
        
        print(f"✅ Mobile robot test completed. Total callbacks: {mobile_callback_count}")
        
    except Exception as e:
        print(f"⚠️ Mobile robot test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            sim.shutdown()
        except:
            pass


def test_multi_robot_debug():
    """Test multi robot with debug output"""
    print("\n🤖🤖 Testing Multi-Robot Example with Debug...")  
    print("=" * 50)
    
    # Explicitly set real-time factor to 1.0 for accurate timing
    from core.simulation_manager import SimulationConfig
    config = SimulationConfig(
        real_time_factor=1.0,  # Ensure 1:1 real-time synchronization
        visualization=True,
        enable_frequency_grouping=False  # Disable frequency grouping to test individual processes
    )
    sim = SimulationManager(config)
    
    try:
        robot1 = sim.add_robot_from_urdf("robot1", "examples/robots/articulated_arm_robot.urdf", Pose(0, 1, 0, 0, 0, 0), joint_update_rate=50.0, unified_process=False)
        robot2 = sim.add_robot_from_urdf("robot2", "examples/robots/collision_robot.urdf", Pose(0, -1, 0, 0, 0, 0), joint_update_rate=50.0, unified_process=False)
        
        print(f"✅ Two robots added successfully")
        
        # Debug: Multi robot callback counters
        robot1_callback_count = 0
        robot2_callback_count = 0
        
        def control_robot1(dt):
            """Control first robot"""
            nonlocal robot1_callback_count
            robot1_callback_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
            
            # Print debug info for first few calls
            if robot1_callback_count <= 5 or robot1_callback_count % 25 == 0:
                print(f"🤖1 Robot1 Callback #{robot1_callback_count}: t={t:.2f}s, dt={dt:.4f}s")
            
            joint_names = [name for name in robot1.get_joint_names() 
                          if robot1.joints[name].joint_type.value != 'fixed']
            
            for i, joint_name in enumerate(joint_names):
                position = 0.3 * math.sin(t * 2 + i)
                sim.set_robot_joint_position("robot1", joint_name, position)
        
        def control_robot2(dt):
            """Control second robot"""
            nonlocal robot2_callback_count
            robot2_callback_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
            
            # Print debug info for first few calls
            if robot2_callback_count <= 5 or robot2_callback_count % 25 == 0:
                print(f"🤖2 Robot2 Callback #{robot2_callback_count}: t={t:.2f}s, dt={dt:.4f}s")
            
            joint_names = [name for name in robot2.get_joint_names() 
                          if robot2.joints[name].joint_type.value != 'fixed']
            
            for i, joint_name in enumerate(joint_names):
                position = 0.4 * math.cos(t * 1.5 + i * math.pi / 2)
                sim.set_robot_joint_position("robot2", joint_name, position)
        
        print("🎮 Setting multi robot control callbacks...")
        sim.set_robot_control_callback("robot1", control_robot1, frequency=50.0)
        sim.set_robot_control_callback("robot2", control_robot2, frequency=50.0)
        
        print("🚀 Starting 3-second multi robot test...")
        sim.run(duration=3.0, auto_close=True)
        
        print(f"✅ Multi robot test completed.")
        print(f"   Robot1 callbacks: {robot1_callback_count}")
        print(f"   Robot2 callbacks: {robot2_callback_count}")
        
    except Exception as e:
        print(f"⚠️ Multi robot test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            sim.shutdown()
        except:
            pass


def main():
    """Run debug tests"""
    print("🔍 Robot Movement Debug Tests")
    print("Testing individual examples with debug output...")
    print("=" * 60)
    
    # Test mobile robot
    test_mobile_robot_debug()
    
    # Pause between tests
    time.sleep(1.0)
    
    # Test multi robot
    test_multi_robot_debug()
    
    print("\n🎉 All debug tests completed!")


if __name__ == "__main__":
    main()