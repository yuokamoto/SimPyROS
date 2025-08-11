#!/usr/bin/env python3
"""
PyVista Interactive Robot Demo
Focused demonstration of PyVista 3D visualization capabilities
Uses built-in geometric robot models to showcase PyVista features

Usage:
    python pyvista_robot_demo.py [duration] [robot_type]
    
Robot Types:
    - wheeled (default): Mobile robot with wheels and arm
    - basic: Simple robot with base, arm, and gripper  
    - quadcopter: Drone with propellers
    - humanoid: Simple humanoid robot
    
Examples:
    python pyvista_robot_demo.py 10                    # 10-second demo with wheeled robot
    python pyvista_robot_demo.py 15 basic              # Basic robot demo
    python pyvista_robot_demo.py 20 quadcopter         # Quadcopter demo
    python pyvista_robot_demo.py 12 humanoid           # Humanoid robot demo
"""

import sys
import os
import numpy as np
import time
import math

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.simulation_object import Pose
from core.pyvista_visualizer import create_interactive_visualizer, setup_basic_scene, AnimationController
from examples.pyvista.sample_robots import SampleRobotFactory

OUTPUT_FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'output')
os.makedirs(OUTPUT_FILE_PATH, exist_ok=True)

def interactive_demo(duration_seconds=10, robot_type='wheeled'):
    """Run interactive PyVista demo using built-in geometric robot models"""
    print(f"Starting {duration_seconds}s interactive PyVista demo...")
    print(f"Robot type: {robot_type}")
    
    # Create visualizer
    viz = create_interactive_visualizer()
    if not viz.available:
        print("PyVista not available - demo cannot run")
        return 0
    
    # Setup basic scene
    setup_basic_scene(viz)
    
    # Create animation controller
    controller = AnimationController(viz.plotter, viz.pv)
    
    # Create built-in geometric robot
    robot_mesh = SampleRobotFactory.create_robot_mesh(viz.pv, robot_type)
    
    if robot_mesh:
        controller.add_robot("main_robot", robot_mesh)
        print(f"✅ {robot_type.capitalize()} robot successfully added to visualization")
    else:
        print(f"❌ Failed to create {robot_type} robot mesh")
        return 0
    
    # Animation parameters
    start_time = time.time()
    frame_count = 0
    
    def animate():
        """Animation callback function"""
        nonlocal frame_count, start_time
        
        current_time = time.time() - start_time
        
        if current_time >= duration_seconds:
            return
            
        # Calculate robot motion (figure-8 pattern)
        t = current_time * 0.8
        x = 3 * math.sin(t)
        y = 1.5 * math.sin(2 * t)
        z = 1 + 0.3 * abs(math.sin(t * 2))
        yaw = t * 0.5
        pitch = 0.1 * math.sin(t * 3)
        
        # Create pose
        pose = Pose(x=x, y=y, z=z, yaw=yaw, pitch=pitch)
        
        # Update robot position (optimized - no mesh recreation)
        controller.update_robot_pose("main_robot", pose)
        
        # Add trajectory trail every few frames
        if frame_count % 10 == 0:
            controller.add_trajectory_trail("main_robot")
        
        frame_count += 1
        
        # Update display with info
        fps = frame_count / current_time if current_time > 0 else 0
        viz.plotter.add_text(
            f"PyVista Demo - {robot_type.capitalize()} Robot\n"
            f"Frame: {frame_count}, FPS: {fps:.1f}, Time: {current_time:.1f}s\n"
            f"Position: ({x:.1f}, {y:.1f}, {z:.1f})", 
            position='upper_left', font_size=10, name='info'
        )
    
    print("Interactive demo started - window should open")
    print("Close the window to end the demo")
    print("The robot will move in a figure-8 pattern")
    
    # Show the plot with animation
    viz.plotter.show(auto_close=False, interactive_update=True)
    
    # Manual animation loop
    try:
        while time.time() - start_time < duration_seconds:
            animate()
            viz.plotter.update()
            time.sleep(0.05)  # 20 FPS
    except KeyboardInterrupt:
        print("\\nDemo interrupted by user")
        
    print(f"Demo completed! Total frames: {frame_count}")
    return frame_count

def main():
    """Main demo selector for PyVista built-in robot types"""
    print("PyVista Robot Visualization Demo")
    print("Built-in geometric robot models showcase")
    print("=" * 50)
    
    # Parse command line arguments
    duration = 10.0
    robot_type = 'wheeled_robot'
    
    if len(sys.argv) > 1:
        try:
            duration = float(sys.argv[1])
        except ValueError:
            print("Invalid duration, using default (10 seconds)")
            duration = 10.0
    
    if len(sys.argv) > 2:
        robot_type = sys.argv[2].lower()
        valid_types = ['basic_robot', 'wheeled_robot', 'quadcopter']
        if robot_type not in valid_types:
            print(f"❌ Invalid robot type '{robot_type}', using 'wheeled_robot'")
            print(f"Valid types: {', '.join(valid_types)}")
            robot_type = 'wheeled_robot'

    # Display configuration
    print(f"Duration: {duration} seconds")
    print(f"Robot type: {robot_type}")
    print("Note: This will open a 3D visualization window")
    print("Available robot types: basic_robot, wheeled_robot, quadcopter")
    print()

    try:
        frames = interactive_demo(duration, robot_type)
    except Exception as e:
        print(f"❌ Interactive demo failed: {e}")

if __name__ == "__main__":
    main()