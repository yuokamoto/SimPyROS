#!/usr/bin/env python3
"""
Basic Simulation Example using SimulationManager

This example demonstrates the simplified interface for robot simulation.
From ~100 lines of setup code down to ~20 lines!

Usage:
    python basic_simulation.py
"""

import sys
import os
import math
import time

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.simulation_manager import SimulationManager
from core.simulation_object import Velocity, Pose


def simple_control_example():
    """Example 1: Simple joint control with auto-close"""
    print("🤖 Simple Control Example")
    print("=" * 40)
    
    sim = SimulationManager()
    
    try:
        robot = sim.add_robot_from_urdf(
            name="my_robot",
            urdf_path="examples/robots/articulated_arm_robot.urdf"
        )
        
        def my_control(dt: float):
            """Simple sinusoidal joint motion"""
            t = time.time()
            joint_names = [name for name in robot.get_joint_names() 
                          if robot.joints[name].joint_type.value != 'fixed']
            
            for i, joint_name in enumerate(joint_names):
                position = 0.5 * math.sin(t + i * math.pi / 3)
                sim.set_robot_joint_position("my_robot", joint_name, position)
        
        sim.set_robot_control_callback("my_robot", my_control, frequency=20.0)
        sim.run(duration=1.0, auto_close=True)
        
    except Exception as e:
        print(f"⚠️ Example error: {e}")
    finally:
        try:
            sim.shutdown()
        except:
            pass


def mobile_robot_example():
    """Example 2: Mobile robot with auto-close"""
    print("🚗 Mobile Robot Example")
    print("=" * 40)
    
    sim = SimulationManager()
    
    try:
        robot = sim.add_robot_from_urdf(
            name="mobile_robot", 
            urdf_path="examples/robots/mobile_robot.urdf"
        )
        
        def mobile_control(dt: float):
            """Move robot in circle"""
            t = time.time()
            linear_speed = 0.5
            angular_speed = 0.3
            
            velocity = Velocity(
                linear_x=linear_speed,
                angular_z=angular_speed * math.sin(t * 0.5)
            )
            sim.set_robot_velocity("mobile_robot", velocity)
        
        sim.set_robot_control_callback("mobile_robot", mobile_control, frequency=30.0)
        sim.run(duration=1.0, auto_close=True)
        
    except Exception as e:
        print(f"⚠️ Example error: {e}")
    finally:
        try:
            sim.shutdown()
        except:
            pass


def multi_robot_example():
    """Example 3: Multi-robot with auto-close"""
    print("🤖🤖 Multi-Robot Example")  
    print("=" * 40)
    
    sim = SimulationManager()
    
    try:
        robot1 = sim.add_robot_from_urdf("robot1", "examples/robots/articulated_arm_robot.urdf", Pose(0, 1, 0, 0, 0, 0))
        robot2 = sim.add_robot_from_urdf("robot2", "examples/robots/collision_robot.urdf", Pose(0, -1, 0, 0, 0, 0))
        
        def control_robot1(dt):
            """Control first robot"""
            t = time.time()
            joint_names = [name for name in robot1.get_joint_names() 
                          if robot1.joints[name].joint_type.value != 'fixed']
            
            for i, joint_name in enumerate(joint_names):
                position = 0.3 * math.sin(t * 2 + i)
                sim.set_robot_joint_position("robot1", joint_name, position)
        
        def control_robot2(dt):
            """Control second robot"""  
            t = time.time()
            joint_names = [name for name in robot2.get_joint_names() 
                          if robot2.joints[name].joint_type.value != 'fixed']
            
            for i, joint_name in enumerate(joint_names):
                position = 0.4 * math.cos(t * 1.5 + i * math.pi / 2)
                sim.set_robot_joint_position("robot2", joint_name, position)
        
        sim.set_robot_control_callback("robot1", control_robot1, frequency=15.0)
        sim.set_robot_control_callback("robot2", control_robot2, frequency=10.0)
        
        sim.run(duration=1.5, auto_close=True)
        
    except Exception as e:
        print(f"⚠️ Example error: {e}")
    finally:
        try:
            sim.shutdown()
        except:
            pass


def headless_example():
    """Example 4: Headless simulation (no visualization)"""
    print("\n🖥️ Headless Example")
    print("=" * 40)
    
    from core.simulation_manager import SimulationConfig
    
    # Create headless configuration
    config = SimulationConfig(
        visualization=False,  # No visualization
        update_rate=100.0     # High update rate
    )
    
    sim = SimulationManager(config)
    robot = sim.add_robot_from_urdf("headless_robot", "examples/robots/articulated_arm_robot.urdf")
    
    frame_count = 0
    
    def headless_control(dt):
        """High-frequency control for headless mode"""
        nonlocal frame_count
        frame_count += 1
        
        if frame_count % 100 == 0:  # Print every second at 100Hz
            print(f"🔄 Frame {frame_count}, dt={dt:.4f}s")
        
        # Simple joint motion
        t = time.time()  
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        for i, joint_name in enumerate(joint_names):
            position = 0.2 * math.sin(t * 3 + i)
            sim.set_robot_joint_position("headless_robot", joint_name, position)
    
    sim.set_robot_control_callback("headless_robot", headless_control, frequency=50.0)
    sim.run(duration=5.0)
    
    # Print final statistics
    info = sim.get_simulation_info()
    print(f"📊 Final stats: {info['frame_count']} frames, {info['average_fps']:.1f} FPS")
    print("✅ Headless example completed")


def main():
    """Run all examples with automatic progression"""
    print("🚀 SimPyROS Simplified Simulation Examples")
    print("This demonstrates the new SimulationManager interface")
    print("=" * 60)
    
    examples = [
        ("Simple Control", simple_control_example),
        ("Mobile Robot", mobile_robot_example),
        ("Multi-Robot", multi_robot_example),
        ("Headless Mode", headless_example)
    ]
    
    for i, (name, func) in enumerate(examples):
        try:
            print(f"\n▶️ Running: {name} ({i+1}/{len(examples)})")
            
            func()
            
            # Brief pause between examples
            if name != "Headless Mode":
                print(f"✅ {name} example completed. Next example starting soon...")
                time.sleep(2.0)
            else:
                time.sleep(0.5)  # Short pause for headless
            
        except KeyboardInterrupt:
            print(f"\n⏹️ {name} interrupted by user")
            break
        except Exception as e:
            print(f"❌ {name} failed: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n🎉 All examples completed!")
    print("\nKey benefits of the new interface:")
    print("  ✅ Automatic environment management") 
    print("  ✅ Built-in visualization integration")
    print("  ✅ Graceful shutdown handling")
    print("  ✅ Multi-robot support")
    print("  ✅ Headless mode support")


if __name__ == "__main__":
    main()