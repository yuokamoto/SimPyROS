import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

"""
Test script for visualization - creates a static plot and saves it as image
"""

import simpy
import math
import numpy as np
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from visualizer import Object3DVisualizer


def test_static_visualization():
    """Test static visualization and save plot"""
    
    print("Creating static visualization test...")
    
    # Create simulation environment
    env = simpy.Environment()
    
    # Create visualizer
    viz = Object3DVisualizer(figure_size=(12, 9), grid_size=6.0)
    
    # Create test objects with interesting poses
    robot = SimulationObject(env, ObjectParameters(
        name="robot",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=0.0, z=0.5, yaw=math.pi/4)  # 45 degrees
    ))
    
    sensor = SimulationObject(env, ObjectParameters(
        name="sensor",
        object_type=ObjectType.STATIC,
        initial_pose=Pose(x=2.0, y=1.0, z=1.2, roll=0.0, pitch=0.0, yaw=math.pi/2)  # 90 degrees
    ))
    
    target = SimulationObject(env, ObjectParameters(
        name="target",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=-1.5, y=2.0, z=0.8, yaw=-math.pi/3)  # -60 degrees
    ))
    
    # Static obstacle
    obstacle = SimulationObject(env, ObjectParameters(
        name="wall",
        object_type=ObjectType.STATIC,
        initial_pose=Pose(x=3.0, y=-1.0, z=2.0)
    ))
    
    # Connect robot to sensor
    robot.connect_to(sensor, Pose(x=2.0, y=1.0, z=0.7))
    
    # Add objects to visualizer with different styles
    viz.add_object(robot, color='blue', marker='o', size=200, show_trajectory=False)
    viz.add_object(sensor, color='green', marker='^', size=150, show_trajectory=False)
    viz.add_object(target, color='red', marker='s', size=120, show_trajectory=False) 
    viz.add_object(obstacle, color='gray', marker='x', size=250, show_trajectory=False)
    
    print("Objects created:")
    print(f"- Robot at: ({robot.pose.x:.1f}, {robot.pose.y:.1f}, {robot.pose.z:.1f}) yaw={math.degrees(robot.pose.yaw):.1f}°")
    print(f"- Sensor at: ({sensor.pose.x:.1f}, {sensor.pose.y:.1f}, {sensor.pose.z:.1f}) yaw={math.degrees(sensor.pose.yaw):.1f}°")
    print(f"- Target at: ({target.pose.x:.1f}, {target.pose.y:.1f}, {target.pose.z:.1f}) yaw={math.degrees(target.pose.yaw):.1f}°")
    print(f"- Wall at: ({obstacle.pose.x:.1f}, {obstacle.pose.y:.1f}, {obstacle.pose.z:.1f})")
    print(f"- Robot-Sensor connection: {robot.is_connected_to(sensor)}")
    
    # Update visualization
    viz._update_plot(0)
    
    # Save as image
    output_file = "test_visualization.png"
    viz.save_frame(output_file, dpi=150)
    print(f"Visualization saved as: {output_file}")
    
    # Print statistics
    stats = viz.get_statistics()
    print(f"Visualization stats: {stats}")
    
    return viz


def test_trajectory_visualization():
    """Test trajectory visualization"""
    
    print("\nCreating trajectory visualization test...")
    
    env = simpy.Environment()
    viz = Object3DVisualizer(grid_size=5.0)
    
    # Create moving object
    obj = SimulationObject(env, ObjectParameters(
        name="moving_object",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=0.0, z=1.0),
        update_interval=0.1
    ))
    
    viz.add_object(obj, color='purple', marker='o', size=100, show_trajectory=True)
    
    # Simulate circular motion manually
    print("Simulating circular trajectory...")
    radius = 2.0
    for i in range(50):
        angle = i * 0.2  # 0.2 radians per step
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = 1.0 + 0.5 * math.sin(angle * 2)  # Varying height
        
        new_pose = Pose(x=x, y=y, z=z, yaw=angle)
        obj.pose = new_pose
        
        # Force update trajectory
        viz._update_trajectories()
    
    # Update and save
    viz._update_plot(0)
    viz.save_frame("test_trajectory.png", dpi=150)
    print("Trajectory visualization saved as: test_trajectory.png")
    
    return viz


if __name__ == "__main__":
    try:
        # Test 1: Static visualization
        viz1 = test_static_visualization()
        
        # Test 2: Trajectory visualization  
        viz2 = test_trajectory_visualization()
        
        print("\nVisualization tests completed successfully!")
        print("Generated files:")
        print("- test_visualization.png (static objects with coordinate frames)")
        print("- test_trajectory.png (circular trajectory)")
        
    except Exception as e:
        print(f"Error during visualization test: {e}")
        import traceback
        traceback.print_exc()