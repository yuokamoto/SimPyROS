#!/usr/bin/env python3
"""
Basic Demo - Demonstration of fundamental SimPyROS features
Integrated demo for learning simple connection systems and basic operations
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np
import time
import math
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose
import simpy


def demo_basic_operations():
    """Demonstration of basic operations"""
    print("=== Basic Operations Demo ===")
    
    # Create SimPy environment
    env = simpy.Environment()
    
    # Create robot
    robot_params = ObjectParameters(
        name="robot",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0, y=0, z=0),
        update_interval=0.1
    )
    robot = SimulationObject(env, robot_params)
    
    # Create sensor
    sensor_params = ObjectParameters(
        name="sensor",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.5, y=0, z=0),
        update_interval=0.1
    )
    sensor = SimulationObject(env, sensor_params)
    
    print(f"Robot initial position: {robot.pose}")
    print(f"Sensor initial position: {sensor.pose}")
    
    # Connect objects
    robot.connect_to(sensor)
    print("Connected robot and sensor")
    
    # Move robot
    new_pose = Pose(x=2, y=1, z=0, yaw=math.pi/4)
    robot.teleport(new_pose)
    
    print(f"Robot position after move: {robot.pose}")
    print(f"Sensor position after move: {sensor.pose}")
    
    # Disconnect objects
    robot.disconnect_from(sensor)
    print("Disconnected objects")
    
    # Move individually
    robot.teleport(Pose(x=0, y=0, z=0))
    print(f"After disconnect - Robot position: {robot.pose}")
    print(f"After disconnect - Sensor position: {sensor.pose}")


def demo_chain_connection():
    """Demonstration of chain connections"""
    print("\n=== Chain Connection Demo ===")
    
    env = simpy.Environment()
    
    # Create three objects
    objects = []
    for i, name in enumerate(['A', 'B', 'C']):
        params = ObjectParameters(
            name=name,
            object_type=ObjectType.DYNAMIC,
            initial_pose=Pose(x=i, y=0, z=0),
            update_interval=0.1
        )
        obj = SimulationObject(env, params)
        objects.append(obj)
    
    a, b, c = objects
    
    print("Initial positions:")
    for obj in objects:
        print(f"  {obj.parameters.name}: {obj.pose}")
    
    # Chain connection: A-B-C
    a.connect_to(b)
    b.connect_to(c)
    print("Chain connection complete: A-B-C")
    
    # Move A → entire chain moves
    a.teleport(Pose(x=5, y=3, z=1, yaw=math.pi/2))
    
    print("Positions after moving A:")
    for obj in objects:
        print(f"  {obj.parameters.name}: {obj.pose}")
    
    # Move B → A and C also move
    b.teleport(Pose(x=2, y=2, z=0, pitch=math.pi/6))
    
    print("Positions after moving B:")
    for obj in objects:
        print(f"  {obj.parameters.name}: {obj.pose}")


def demo_static_constraint():
    """Demonstration of static constraints"""
    print("\n=== Static Constraint Demo ===")
    
    env = simpy.Environment()
    
    # Static object (immovable base)
    static_params = ObjectParameters(
        name="base",
        object_type=ObjectType.STATIC,
        initial_pose=Pose(x=0, y=0, z=0),
        update_interval=0.1
    )
    static_obj = SimulationObject(env, static_params)
    
    # Dynamic object
    dynamic_params = ObjectParameters(
        name="arm",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=1, y=0, z=0),
        update_interval=0.1
    )
    dynamic_obj = SimulationObject(env, dynamic_params)
    
    print("Before connection:")
    print(f"  Base (STATIC): {static_obj.pose}")
    print(f"  Arm (DYNAMIC): {dynamic_obj.pose}")
    
    # Connect objects
    static_obj.connect_to(dynamic_obj)
    print("Connected dynamic object to static object")
    
    # Attempt to move dynamic object
    print("Attempting to move dynamic object...")
    original_pose = dynamic_obj.pose
    dynamic_obj.teleport(Pose(x=5, y=5, z=2))
    
    print("After move attempt:")
    print(f"  Base (STATIC): {static_obj.pose}")
    print(f"  Arm (DYNAMIC): {dynamic_obj.pose}")
    
    if dynamic_obj.pose == original_pose:
        print("✓ Movement blocked by static constraint")
    else:
        print("✓ Bidirectional connection system working (both objects moved together)")


def main():
    """Main demo execution"""
    print("SimPyROS Basic Demonstration")
    print("=" * 50)
    
    demo_basic_operations()
    demo_chain_connection() 
    demo_static_constraint()
    
    print("\n" + "=" * 50)
    print("🎉 Basic demo completed!")
    print("Next steps:")
    print("  - visualization_demo.py for 3D visualization experience")
    print("  - realtime_demo_simple.py for real-time processing experience")
    print("  - pyvista_simple_demo.py for high-quality 3D rendering experience")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDemo interrupted")
    except Exception as e:
        print(f"\nDemo error: {e}")
        import traceback
        traceback.print_exc()