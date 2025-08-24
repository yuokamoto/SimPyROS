#!/usr/bin/env python3
"""
Advanced SimPy Robot Behaviors Demo

Demonstrates the power of SimPy's event-driven architecture with:
- Independent robot subsystem processes
- Complex navigation behaviors
- Sensor-driven decision making
- Multi-robot coordination
- Event-driven task scheduling

This example showcases why the new architecture is superior to single-loop design.
"""

import sys
import os
import time
import numpy as np
import random
from typing import List, Dict, Optional

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.robot import Robot, create_robot_from_urdf
from core.simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from core.time_manager import TimeManager
import simpy.rt


def create_patrol_robot(sim_manager: SimulationManager, robot_name: str, patrol_points: List[Pose]) -> Robot:
    """Create a robot with autonomous patrol behavior"""
    
    # Create robot
    robot_urdf_path = "robots/articulated_arm_robot.urdf"
    initial_pose = patrol_points[0] if patrol_points else Pose()
    
    try:
        robot = sim_manager.add_robot_from_urdf(
            robot_name, 
            robot_urdf_path, 
            initial_pose=initial_pose,
            joint_update_frequency=50.0  # Lower frequency for smoother demo
        )
        
        # Set up patrol behavior
        def patrol_control(dt: float):
            """Control callback for patrol behavior"""
            if not hasattr(patrol_control, 'current_target_index'):
                patrol_control.current_target_index = 1  # Start with second point
                patrol_control.last_target_time = time.time()
            
            current_time = time.time()
            if current_time - patrol_control.last_target_time > 5.0:  # Change target every 5 seconds
                # Move to next patrol point
                patrol_control.current_target_index = (patrol_control.current_target_index + 1) % len(patrol_points)
                target_pose = patrol_points[patrol_control.current_target_index]
                
                print(f"🚁 {robot_name}: Patrolling to point {patrol_control.current_target_index}: {target_pose.position}")
                
                # Start navigation process to target
                robot.stop_navigation_process()  # Stop current navigation
                robot.start_navigation_process(target_pose)
                
                patrol_control.last_target_time = current_time
        
        # Set control callback
        sim_manager.set_robot_control_callback(robot_name, patrol_control, frequency=2.0)
        
        print(f"✅ Created patrol robot '{robot_name}' with {len(patrol_points)} patrol points")
        return robot
        
    except Exception as e:
        print(f"❌ Failed to create patrol robot '{robot_name}': {e}")
        return None


def create_search_robot(sim_manager: SimulationManager, robot_name: str, search_area: Dict) -> Robot:
    """Create a robot with search and exploration behavior"""
    
    robot_urdf_path = "robots/articulated_arm_robot.urdf"
    center = search_area.get('center', [0, 0, 0])
    initial_pose = Pose(x=center[0], y=center[1], z=center[2])
    
    try:
        robot = sim_manager.add_robot_from_urdf(
            robot_name,
            robot_urdf_path,
            initial_pose=initial_pose,
            joint_update_frequency=30.0
        )
        
        # Set up search behavior
        def search_control(dt: float):
            """Control callback for search behavior"""
            if not hasattr(search_control, 'last_search_time'):
                search_control.last_search_time = time.time()
                search_control.search_count = 0
            
            current_time = time.time()
            if current_time - search_control.last_search_time > 3.0:  # New search target every 3 seconds
                # Generate random search point within area
                radius = search_area.get('radius', 2.0)
                angle = random.uniform(0, 2 * np.pi)
                distance = random.uniform(0.5, radius)
                
                target_x = center[0] + distance * np.cos(angle)
                target_y = center[1] + distance * np.sin(angle)
                target_pose = Pose(x=target_x, y=target_y, z=center[2])
                
                search_control.search_count += 1
                print(f"🔍 {robot_name}: Search #{search_control.search_count} - exploring ({target_x:.1f}, {target_y:.1f})")
                
                # Start navigation to search point
                robot.stop_navigation_process()
                robot.start_navigation_process(target_pose)
                
                # Move arm joints for "scanning" behavior
                joint_positions = {
                    'joint_1': random.uniform(-1.0, 1.0),
                    'joint_2': random.uniform(-0.5, 0.5),
                    'joint_3': random.uniform(-0.5, 0.5)
                }
                robot.set_joint_positions(joint_positions)
                
                search_control.last_search_time = current_time
        
        sim_manager.set_robot_control_callback(robot_name, search_control, frequency=1.0)
        
        print(f"✅ Created search robot '{robot_name}' in area radius {search_area.get('radius', 2.0)}m")
        return robot
        
    except Exception as e:
        print(f"❌ Failed to create search robot '{robot_name}': {e}")
        return None


def create_follower_robot(sim_manager: SimulationManager, robot_name: str, leader_robot: Robot) -> Robot:
    """Create a robot that follows another robot"""
    
    robot_urdf_path = "robots/articulated_arm_robot.urdf"
    leader_pose = leader_robot.get_pose()
    # Start behind the leader
    follower_pose = Pose(x=leader_pose.x - 1.0, y=leader_pose.y - 1.0, z=leader_pose.z)
    
    try:
        robot = sim_manager.add_robot_from_urdf(
            robot_name,
            robot_urdf_path,
            initial_pose=follower_pose,
            joint_update_frequency=40.0
        )
        
        # Set up following behavior
        def follow_control(dt: float):
            """Control callback for following behavior"""
            leader_pose = leader_robot.get_pose()
            follower_pose = robot.get_pose()
            
            # Calculate distance to leader
            distance = np.linalg.norm(leader_pose.position - follower_pose.position)
            
            # Follow if leader is too far away
            if distance > 1.5:  # Follow if more than 1.5m away
                # Calculate follow position (1m behind leader)
                direction_to_leader = (leader_pose.position - follower_pose.position) / distance
                follow_position = leader_pose.position - direction_to_leader * 1.0
                follow_pose = Pose(x=follow_position[0], y=follow_position[1], z=follow_position[2])
                
                # Start navigation to follow position
                robot.stop_navigation_process()
                robot.start_navigation_process(follow_pose)
                
                print(f"🐕 {robot_name}: Following leader (distance: {distance:.1f}m)")
            elif distance < 0.8:  # Too close, back away
                robot.stop()
                print(f"🐕 {robot_name}: Too close to leader, stopping")
        
        sim_manager.set_robot_control_callback(robot_name, follow_control, frequency=5.0)
        
        print(f"✅ Created follower robot '{robot_name}' following '{leader_robot.robot_name}'")
        return robot
        
    except Exception as e:
        print(f"❌ Failed to create follower robot '{robot_name}': {e}")
        return None


def create_coordinated_demo():
    """Create a complex multi-robot coordination demo"""
    
    print("\n🚀 Starting Advanced SimPy Robot Behaviors Demo")
    print("=" * 60)
    
    # Create simulation with moderate real-time factor for observation
    config = SimulationConfig(
        visualization=True,
        real_time_factor=1.5,  # Slightly faster than real-time
        update_frequency=100.0,
        time_step=0.01
    )
    
    sim = SimulationManager(config)
    
    # Define patrol points for patrol robot
    patrol_points = [
        Pose(x=2.0, y=2.0, z=0.0),
        Pose(x=-2.0, y=2.0, z=0.0),
        Pose(x=-2.0, y=-2.0, z=0.0),
        Pose(x=2.0, y=-2.0, z=0.0)
    ]
    
    # Define search area for search robot
    search_area = {
        'center': [0.0, 0.0, 0.0],
        'radius': 1.5
    }
    
    try:
        # Create robots with different behaviors
        print("\n🤖 Creating robots with independent behaviors...")
        
        # 1. Patrol robot - follows predefined waypoints
        patrol_robot = create_patrol_robot(sim, "patrol_robot", patrol_points)
        
        # 2. Search robot - random exploration in area
        search_robot = create_search_robot(sim, "search_robot", search_area)
        
        # 3. Follower robot - follows the patrol robot
        if patrol_robot:
            follower_robot = create_follower_robot(sim, "follower_robot", patrol_robot)
        
        # Add some static obstacles
        obstacle1 = ObjectParameters(
            name="obstacle1",
            object_type=ObjectType.STATIC,
            initial_pose=Pose(x=1.0, y=0.0, z=0.0)
        )
        sim.add_object("obstacle1", SimulationObject(sim.env, obstacle1, sim.time_manager))
        
        obstacle2 = ObjectParameters(
            name="obstacle2", 
            object_type=ObjectType.STATIC,
            initial_pose=Pose(x=-1.0, y=1.0, z=0.0)
        )
        sim.add_object("obstacle2", SimulationObject(sim.env, obstacle2, sim.time_manager))
        
        print(f"\n📊 Demo Statistics:")
        print(f"   - Robots: {len(sim.robots)}")
        print(f"   - Objects: {len(sim.objects)}")
        print(f"   - Real-time factor: {config.real_time_factor}x")
        print(f"   - SimPy processes: Multiple independent (robots + sensors + navigation)")
        
        print(f"\n🎮 Demo Features:")
        print(f"   - Patrol Robot: Autonomous waypoint navigation")
        print(f"   - Search Robot: Random exploration with arm scanning")
        print(f"   - Follower Robot: Leader-following behavior")
        print(f"   - Independent SimPy processes for each robot subsystem")
        print(f"   - Event-driven behaviors using SimPy's power")
        print(f"   - Real-time visualization with interactive controls")
        
        print(f"\n▶️ Running simulation...")
        print(f"   Use visualization controls to:")
        print(f"   - Adjust real-time factor with slider")
        print(f"   - Pause/resume with play button")
        print(f"   - Reset with reset button")
        print(f"   - Close window or Ctrl+C to exit")
        print("=" * 60)
        
        # Run simulation
        sim.run(duration=60.0, auto_close=False)  # Run for 60 seconds
        
    except KeyboardInterrupt:
        print("\n🛑 Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Demo error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n📊 Final Statistics:")
        info = sim.get_simulation_info()
        print(f"   - Simulation time: {info.get('elapsed_time', 0):.1f}s")
        print(f"   - Average FPS: {info.get('average_fps', 0):.1f} Hz")
        if 'timing_stats' in info:
            stats = info['timing_stats']
            print(f"   - Sim time: {stats.get('sim_time', 0):.1f}s")
            print(f"   - Target speed: {stats.get('target_speed', 'N/A')}")
            print(f"   - Actual speed: {stats.get('actual_speed', 'N/A')}")
        
        print("\n✅ Advanced SimPy Demo Complete!")
        print("   This demo showcased SimPy's event-driven architecture")
        print("   with independent robot processes, sensor fusion, and")
        print("   complex autonomous behaviors - much more powerful")
        print("   than single-loop centralized management!")


if __name__ == "__main__":
    create_coordinated_demo()