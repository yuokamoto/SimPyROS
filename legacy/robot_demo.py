#!/usr/bin/env python3
"""
Robot Class Demonstration
Shows advanced joint-level control and ROS 2 compatible interfaces
"""

import sys
import os
import time
import math
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import simpy
from robot import Robot, RobotParameters, create_robot_from_urdf, create_simple_arm_robot, create_mobile_robot
from simulation_object import ObjectType, Pose, Velocity


def basic_robot_demo():
    """Basic robot loading and information display"""
    print("🤖 Basic Robot Demo")
    print("=" * 50)
    
    # Create simulation environment
    env = simpy.Environment()
    
    # Create robot from URDF
    try:
        robot = create_simple_arm_robot(env, "demo_arm", Pose(x=0, y=0, z=0))
        robot.print_robot_info()
        
        # Run brief simulation to initialize
        env.run(until=0.1)
        
        print("✅ Robot created and initialized successfully")
        return robot
        
    except Exception as e:
        print(f"❌ Failed to create robot: {e}")
        return None


def joint_position_control_demo():
    """Demonstrate joint position control"""
    print("\n🎯 Joint Position Control Demo")
    print("=" * 50)
    
    env = simpy.Environment()
    robot = create_simple_arm_robot(env, "position_controlled_arm")
    
    if robot is None:
        print("❌ Failed to create robot")
        return
    
    def robot_controller():
        """Control robot joints with sinusoidal motion"""
        print("Starting joint position control...")
        
        # Get joint names
        joint_names = robot.get_joint_names()
        print(f"Controlling joints: {joint_names}")
        
        # Control loop
        for step in range(100):  # 10 seconds at 10Hz
            t = step * 0.1
            
            # Set sinusoidal joint positions
            for i, joint_name in enumerate(joint_names):
                if robot.joints[joint_name].joint_type.value != 'fixed':
                    # Different frequency for each joint
                    amplitude = 0.5 + i * 0.2
                    frequency = 0.5 + i * 0.3
                    target_pos = amplitude * math.sin(frequency * t)
                    robot.set_joint_position(joint_name, target_pos, max_velocity=1.0)
            
            yield env.timeout(0.1)
            
            # Print joint states every second
            if step % 10 == 0:
                print(f"Time: {t:.1f}s")
                joint_positions = robot.get_joint_positions()
                for name, pos in joint_positions.items():
                    print(f"  {name}: {pos:.3f} rad")
        
        print("✅ Position control demo completed")
    
    # Start controller
    env.process(robot_controller())
    env.run()


def joint_velocity_control_demo():
    """Demonstrate joint velocity control"""
    print("\n⚡ Joint Velocity Control Demo") 
    print("=" * 50)
    
    env = simpy.Environment()
    robot = create_simple_arm_robot(env, "velocity_controlled_arm")
    
    if robot is None:
        print("❌ Failed to create robot")
        return
    
    def robot_controller():
        """Control robot joints with varying velocities"""
        print("Starting joint velocity control...")
        
        joint_names = robot.get_joint_names()
        print(f"Controlling joints: {joint_names}")
        
        # Phase 1: Slow motion
        print("Phase 1: Slow constant velocities")
        for joint_name in joint_names:
            if robot.joints[joint_name].joint_type.value != 'fixed':
                robot.set_joint_velocity(joint_name, 0.2)  # 0.2 rad/s
        
        yield env.timeout(2.0)
        
        # Phase 2: Stop
        print("Phase 2: Stop all joints")
        robot.stop_all_joints()
        yield env.timeout(1.0)
        
        # Phase 3: Reverse direction
        print("Phase 3: Reverse direction")
        for joint_name in joint_names:
            if robot.joints[joint_name].joint_type.value != 'fixed':
                robot.set_joint_velocity(joint_name, -0.3)  # -0.3 rad/s
        
        yield env.timeout(2.0)
        
        # Phase 4: Variable velocities
        print("Phase 4: Variable velocities")
        for step in range(50):  # 5 seconds
            t = step * 0.1
            
            for i, joint_name in enumerate(joint_names):
                if robot.joints[joint_name].joint_type.value != 'fixed':
                    velocity = 0.5 * math.sin(t + i)
                    robot.set_joint_velocity(joint_name, velocity)
            
            yield env.timeout(0.1)
            
            if step % 10 == 0:
                velocities = robot.get_joint_velocities()
                print(f"Time: {t:.1f}s - Velocities: {velocities}")
        
        robot.stop_all_joints()
        print("✅ Velocity control demo completed")
    
    env.process(robot_controller())
    env.run()


def multi_joint_coordination_demo():
    """Demonstrate coordinated multi-joint control"""
    print("\n🔄 Multi-Joint Coordination Demo")
    print("=" * 50)
    
    env = simpy.Environment()
    robot = create_simple_arm_robot(env, "coordinated_arm")
    
    if robot is None:
        print("❌ Failed to create robot")
        return
    
    def robot_controller():
        """Demonstrate coordinated joint movements"""
        print("Starting coordinated joint control...")
        
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        if len(joint_names) < 2:
            print("❌ Need at least 2 movable joints for coordination demo")
            return
        
        print(f"Coordinating joints: {joint_names}")
        
        # Scenario 1: Sequential motion
        print("Scenario 1: Sequential motion")
        for i, joint_name in enumerate(joint_names):
            print(f"  Moving {joint_name}")
            target_pos = (-1) ** i * 0.8  # Alternate directions
            robot.set_joint_position(joint_name, target_pos, max_velocity=1.0)
            yield env.timeout(1.5)
            
            # Print current state
            positions = robot.get_joint_positions()
            print(f"  Current positions: {positions}")
        
        yield env.timeout(1.0)
        
        # Scenario 2: Synchronized motion
        print("Scenario 2: Synchronized motion")
        target_positions = {}
        for i, joint_name in enumerate(joint_names):
            target_positions[joint_name] = 0.5 * math.sin(i * math.pi / 4)
        
        print(f"  Target positions: {target_positions}")
        robot.set_joint_positions(target_positions, max_velocity=0.5)
        
        yield env.timeout(3.0)
        
        # Scenario 3: Circular coordination
        print("Scenario 3: Circular coordination pattern")
        for step in range(60):  # 6 seconds
            t = step * 0.1
            
            # Create circular pattern in joint space
            target_positions = {}
            for i, joint_name in enumerate(joint_names):
                phase_offset = i * 2 * math.pi / len(joint_names)
                amplitude = 0.4
                target_positions[joint_name] = amplitude * math.sin(t + phase_offset)
            
            robot.set_joint_positions(target_positions, max_velocity=2.0)
            yield env.timeout(0.1)
            
            if step % 20 == 0:
                current_pos = robot.get_joint_positions()
                print(f"  Time: {t:.1f}s - Positions: {current_pos}")
        
        # Return to home
        print("Returning to home position...")
        home_positions = {name: 0.0 for name in joint_names}
        robot.set_joint_positions(home_positions, max_velocity=1.0)
        yield env.timeout(2.0)
        
        print("✅ Coordination demo completed")
    
    env.process(robot_controller())
    env.run()


def robot_and_base_control_demo():
    """Demonstrate combined robot base movement and joint control"""
    print("\n🚁 Robot Base + Joint Control Demo")
    print("=" * 50)
    
    env = simpy.Environment()
    robot = create_mobile_robot(env, "mobile_manipulator")
    
    if robot is None:
        print("❌ Failed to create robot")
        return
    
    def robot_controller():
        """Control both robot base and joints simultaneously"""
        print("Starting combined base and joint control...")
        
        # Phase 1: Move base while controlling joints
        print("Phase 1: Moving base with joint motion")
        
        # Set base velocity (inherited from SimulationObject)
        robot.set_velocity(Velocity(linear_x=0.5, angular_z=0.2))
        
        # Control joints simultaneously
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        if joint_names:
            print(f"Moving joints: {joint_names}")
            # Simple joint motion while base moves
            for step in range(30):  # 3 seconds
                t = step * 0.1
                for joint_name in joint_names:
                    target_pos = 0.3 * math.sin(2 * t)
                    robot.set_joint_position(joint_name, target_pos)
                
                yield env.timeout(0.1)
                
                if step % 10 == 0:
                    print(f"  Time: {t:.1f}s - Base pose: {robot.pose}")
                    positions = robot.get_joint_positions()
                    print(f"    Joint positions: {positions}")
        else:
            yield env.timeout(3.0)
        
        # Phase 2: Stop base, continue joint control
        print("Phase 2: Stopped base, active joints")
        robot.set_velocity(Velocity.zero())
        
        yield env.timeout(2.0)
        
        # Phase 3: Teleport base, reset joints
        print("Phase 3: Teleport and reset")
        new_pose = Pose(x=2.0, y=1.0, z=0.0, yaw=math.pi/2)
        robot.teleport(new_pose)
        
        # Reset all joints
        if joint_names:
            home_positions = {name: 0.0 for name in joint_names}
            robot.set_joint_positions(home_positions)
        
        yield env.timeout(2.0)
        
        print(f"Final robot pose: {robot.pose}")
        final_positions = robot.get_joint_positions()
        print(f"Final joint positions: {final_positions}")
        
        print("✅ Combined control demo completed")
    
    env.process(robot_controller())
    env.run()


def ros2_compatibility_demo():
    """Demonstrate ROS 2 compatible interface usage"""
    print("\n🌉 ROS 2 Compatibility Interface Demo")
    print("=" * 50)
    
    env = simpy.Environment()
    robot = create_simple_arm_robot(env, "ros2_compatible_robot")
    
    if robot is None:
        print("❌ Failed to create robot")
        return
    
    def simulation_node():
        """Simulate ROS 2 node behavior"""
        print("Simulating ROS 2 node interactions...")
        
        # 1. Get robot information (equivalent to robot_description)
        robot_info = robot.get_robot_info()
        print("\n📋 Robot Description:")
        for key, value in robot_info.items():
            if not isinstance(value, (list, dict)):
                print(f"  {key}: {value}")
        
        yield env.timeout(0.1)
        
        # 2. Joint state monitoring (equivalent to /joint_states topic)
        print("\n📊 Joint State Monitoring:")
        for i in range(10):
            joint_states = robot.get_joint_states()
            print(f"Time: {i*0.1:.1f}s")
            for name, state in joint_states.items():
                print(f"  {name}: pos={state.position:.3f}, vel={state.velocity:.3f}, eff={state.effort:.3f}")
            yield env.timeout(0.1)
        
        # 3. Joint trajectory execution (equivalent to /joint_trajectory_controller)
        print("\n🎯 Joint Trajectory Execution:")
        
        # Define trajectory points (ROS 2 style)
        trajectory_points = [
            {'time': 0.0, 'positions': {'base_to_arm': 0.0, 'arm_to_end': 0.0}},
            {'time': 1.0, 'positions': {'base_to_arm': 0.5, 'arm_to_end': 0.3}},
            {'time': 2.0, 'positions': {'base_to_arm': -0.3, 'arm_to_end': 0.8}},
            {'time': 3.0, 'positions': {'base_to_arm': 0.0, 'arm_to_end': 0.0}},
        ]
        
        start_time = env.now
        for point in trajectory_points:
            target_time = start_time + point['time']
            
            # Wait until trajectory time
            while env.now < target_time:
                yield env.timeout(0.01)
            
            # Execute trajectory point
            print(f"Executing trajectory point at t={point['time']:.1f}s")
            robot.set_joint_positions(point['positions'], max_velocity=1.0)
            
            yield env.timeout(0.1)
        
        # 4. Service-like interface (equivalent to ROS 2 services)
        print("\n🔧 Service Interface Simulation:")
        
        # Get current state service
        current_state = {
            'joint_positions': robot.get_joint_positions(),
            'joint_velocities': robot.get_joint_velocities(),
            'base_pose': robot.pose,
            'robot_info': robot.get_robot_info()
        }
        
        print("Current robot state service response:")
        print(f"  Base pose: {current_state['base_pose']}")
        print(f"  Joint positions: {current_state['joint_positions']}")
        
        yield env.timeout(1.0)
        
        print("✅ ROS 2 compatibility demo completed")
    
    env.process(simulation_node())
    env.run()


def main():
    """Run all robot demos"""
    print("🤖 SimPyROS Robot Class Demonstration")
    print("=" * 60)
    
    # Run all demos
    try:
        basic_robot_demo()
        joint_position_control_demo()
        joint_velocity_control_demo()
        multi_joint_coordination_demo()
        robot_and_base_control_demo()
        ros2_compatibility_demo()
        
        print("\n🎉 All robot demos completed successfully!")
        print("\nNext steps:")
        print("- Integrate with PyVista visualization")
        print("- Add ROS 2 bridge for real robot control")
        print("- Implement advanced control algorithms")
        print("- Add physics simulation integration")
        
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()