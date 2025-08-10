#!/usr/bin/env python3
"""
PyVista Interactive Robot Demo
Shows real-time robot visualization with PyVista using the shared visualizer module
Now supports URDF robot loading!

Usage:
    python pyvista_robot_demo.py [duration]                    # Built-in wheeled robot
    python pyvista_robot_demo.py [duration] [urdf_path]        # Load from URDF file
    
Examples:
    python pyvista_robot_demo.py 10                                    # 10-second demo with built-in robot
    python pyvista_robot_demo.py 15 ../robots/simple_robot.urdf        # Load simple robot URDF
    python pyvista_robot_demo.py 20 ../robots/mobile_robot.urdf        # Load mobile robot URDF
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np
import time
import math
from simulation_object import Pose
from pyvista_visualizer import create_interactive_visualizer, setup_basic_scene, create_robot_mesh, AnimationController

def interactive_demo(duration_seconds=10, urdf_path=None):
    """Run interactive PyVista demo using the shared visualizer with optional URDF support"""
    print(f"Starting {duration_seconds}s interactive PyVista demo...")
    if urdf_path:
        print(f"URDF file: {urdf_path}")
    else:
        print("Using built-in wheeled robot")
    
    # Create visualizer
    viz = create_interactive_visualizer()
    if not viz.available:
        print("PyVista not available - demo cannot run")
        return 0
    
    # Setup basic scene
    setup_basic_scene(viz)
    
    # Create animation controller
    controller = AnimationController(viz.plotter, viz.pv)
    
    # Create and add robot with URDF support
    robot_mesh = None
    if urdf_path and os.path.exists(urdf_path):
        print(f"Loading robot from URDF: {urdf_path}")
        robot_mesh = create_robot_mesh(viz, robot_type='urdf', urdf_path=urdf_path)
        
    # Fallback to built-in robot if URDF loading fails
    if robot_mesh is None:
        robot_type = 'wheeled'  # Default fallback
        if urdf_path:
            print("URDF loading failed, using built-in wheeled robot")
        robot_mesh = create_robot_mesh(viz, robot_type)
    
    if robot_mesh:
        controller.add_robot("main_robot", robot_mesh)
        print("Robot successfully added to visualization")
    else:
        print("Warning: Could not create robot mesh")
    
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
            f"Frame: {frame_count}, FPS: {fps:.1f}, Time: {current_time:.1f}s", 
            position='upper_left', font_size=12, name='info'
        )
        viz.plotter.add_text(
            f"Robot Position: ({x:.1f}, {y:.1f}, {z:.1f})", 
            position='upper_right', font_size=10, name='pos'
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
    
    viz.plotter.close()
    
    print(f"Demo completed! Total frames: {frame_count}")
    return frame_count

def screenshot_demo(urdf_path=None):
    """Create series of robot screenshots using the shared visualizer with optional URDF support"""
    print("Creating robot screenshot series...")
    if urdf_path:
        print(f"Using URDF: {urdf_path}")
    else:
        print("Using built-in wheeled robot")
    
    from pyvista_visualizer import create_headless_visualizer
    
    os.makedirs("../output", exist_ok=True)
    
    # Create poses for different robot configurations
    poses = []
    for i in range(8):
        angle = i * math.pi / 4
        x = 2 * math.cos(angle)
        y = 2 * math.sin(angle)
        z = 1 + 0.5 * math.sin(angle * 2)
        yaw = angle + math.pi/2
        poses.append(Pose(x=x, y=y, z=z, yaw=yaw))
    
    for i, pose in enumerate(poses):
        # Create headless visualizer for each screenshot
        viz = create_headless_visualizer()
        if not viz.available:
            print("PyVista not available for screenshots")
            return
        
        # Setup scene
        setup_basic_scene(viz)
        
        # Create and transform robot with URDF support
        robot = None
        if urdf_path and os.path.exists(urdf_path):
            robot = create_robot_mesh(viz, robot_type='urdf', urdf_path=urdf_path)
        
        # Fallback to built-in robot
        if robot is None:
            robot = create_robot_mesh(viz, 'wheeled')
            
        if robot:
            transform_matrix = pose.to_transformation_matrix()
            robot.transform(transform_matrix, inplace=True)
            viz.plotter.add_mesh(robot, color='blue', opacity=0.9)
        
        # Add target marker
        target = viz.pv.Sphere(center=[0, 0, 1], radius=0.2, phi_resolution=12, theta_resolution=12)
        viz.plotter.add_mesh(target, color='red', opacity=0.7)
        
        # Save screenshot
        filename = f"../output/pyvista_robot_{i:02d}.png"
        viz.plotter.screenshot(filename)
        viz.plotter.close()
        
        print(f"Saved: {filename}")
    
    print(f"✅ Screenshot series complete! 8 images saved in output/")

def main():
    """Main demo selector with URDF support"""
    print("PyVista Robot Visualization Demo")
    print("Using shared visualizer module with URDF support")
    print("=" * 50)
    
    # Parse command line arguments
    duration = 10.0
    urdf_path = None
    
    if len(sys.argv) > 1:
        try:
            duration = float(sys.argv[1])
        except ValueError:
            print("Invalid duration, using default (10 seconds)")
            duration = 10.0
    
    if len(sys.argv) > 2:
        urdf_path = sys.argv[2]
        if not os.path.exists(urdf_path):
            print(f"Warning: URDF file not found: {urdf_path}")
            print("Will use built-in robot instead")
            urdf_path = None
    
    # Display configuration
    print(f"Duration: {duration} seconds")
    if urdf_path:
        print(f"URDF file: {urdf_path}")
    else:
        print("Robot: Built-in wheeled robot")
    print("Note: This will open a 3D visualization window")
    print()
    
    try:
        # Try interactive demo first
        frames = interactive_demo(duration, urdf_path)
        if frames == 0:
            print("Interactive demo failed, trying screenshot demo...")
            screenshot_demo(urdf_path)
    except Exception as e:
        print(f"Interactive demo failed: {e}")
        print("Falling back to screenshot demo...")
        screenshot_demo(urdf_path)

if __name__ == "__main__":
    main()