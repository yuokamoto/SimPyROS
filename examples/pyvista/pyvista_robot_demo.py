#!/usr/bin/env python3
"""
PyVista Interactive Robot Demo
Shows real-time robot visualization with PyVista
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np
import time
import math

# Configure PyVista for headless operation BEFORE import
os.environ['PYVISTA_OFF_SCREEN'] = 'false'  # Enable interactive display
os.environ['PYVISTA_USE_PANEL'] = 'false'

import pyvista as pv
from simulation_object import Pose

def create_robot_mesh():
    """Create a detailed robot mesh"""
    # Robot base (main body)
    base = pv.Box(bounds=[-0.5, 0.5, -0.3, 0.3, 0, 0.2])
    
    # Robot arm (articulated)
    arm1 = pv.Cylinder(center=[0.3, 0, 0.4], direction=[0, 0, 1], 
                      radius=0.1, height=0.6, resolution=16)
    
    # End effector
    gripper = pv.Sphere(center=[0.3, 0, 0.8], radius=0.08, phi_resolution=16, theta_resolution=16)
    
    # Wheels
    wheel_positions = [[-0.4, -0.35, -0.1], [-0.4, 0.35, -0.1], 
                       [0.4, -0.35, -0.1], [0.4, 0.35, -0.1]]
    wheels = []
    for pos in wheel_positions:
        wheel = pv.Cylinder(center=pos, direction=[0, 1, 0], 
                           radius=0.12, height=0.05, resolution=12)
        wheels.append(wheel)
    
    # Combine all parts
    robot = base + arm1 + gripper
    for wheel in wheels:
        robot = robot + wheel
        
    return robot

def interactive_demo(duration_seconds=10):
    """Run interactive PyVista demo"""
    print(f"Starting {duration_seconds}s interactive PyVista demo...")
    
    # Create plotter
    plotter = pv.Plotter(window_size=(1200, 800))
    plotter.set_background('lightblue')
    
    # Create ground plane
    ground = pv.Plane(center=[0, 0, -0.2], direction=[0, 0, 1], 
                     i_size=10, j_size=10, i_resolution=20, j_resolution=20)
    plotter.add_mesh(ground, color='lightgray', opacity=0.6)
    
    # Add coordinate axes
    axes_length = 1.5
    x_axis = pv.Arrow(start=[0, 0, 0], direction=[1, 0, 0], scale=axes_length)
    y_axis = pv.Arrow(start=[0, 0, 0], direction=[0, 1, 0], scale=axes_length)
    z_axis = pv.Arrow(start=[0, 0, 0], direction=[0, 0, 1], scale=axes_length)
    
    plotter.add_mesh(x_axis, color='red')
    plotter.add_mesh(y_axis, color='green') 
    plotter.add_mesh(z_axis, color='blue')
    
    # Create initial robot
    robot = create_robot_mesh()
    robot_actor = plotter.add_mesh(robot, color='orange', opacity=0.9)
    
    # Add trajectory trail
    trajectory_points = []
    
    # Set camera
    plotter.camera_position = [(6, 6, 4), (0, 0, 1), (0, 0, 1)]
    
    # Animation parameters
    start_time = time.time()
    frame_count = 0
    
    
    # Set up animation callback
    def animate():
        """Animation callback function"""
        nonlocal frame_count, robot_actor, trajectory_points, start_time
        
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
        
        # Update robot position
        new_robot = create_robot_mesh()
        transform_matrix = pose.to_transformation_matrix()
        new_robot.transform(transform_matrix, inplace=True)
        
        # Remove old robot and add new one
        plotter.remove_actor(robot_actor)
        robot_actor = plotter.add_mesh(new_robot, color='orange', opacity=0.9)
        
        # Add to trajectory trail
        trajectory_points.append([x, y, z])
        if len(trajectory_points) > 50:  # Limit trail length
            trajectory_points.pop(0)
            
        # Draw trajectory trail
        if len(trajectory_points) > 2:
            trail = pv.Spline(np.array(trajectory_points), n_points=100)
            plotter.add_mesh(trail, color='yellow', line_width=3, name='trail')
        
        frame_count += 1
        
        # Update title with info
        fps = frame_count / current_time if current_time > 0 else 0
        plotter.add_text(f"Frame: {frame_count}, FPS: {fps:.1f}, Time: {current_time:.1f}s", 
                        position='upper_left', font_size=12, name='info')
        plotter.add_text(f"Robot Position: ({x:.1f}, {y:.1f}, {z:.1f})", 
                        position='upper_right', font_size=10, name='pos')
    
    print("Interactive demo started - window should open")
    print("Close the window to end the demo")
    print("The robot will move in a figure-8 pattern")
    
    # Show the plot with animation
    plotter.show(auto_close=False, interactive_update=True)
    
    # Manual animation loop
    try:
        while time.time() - start_time < duration_seconds:
            animate()
            plotter.update()
            time.sleep(0.05)  # 20 FPS
    except KeyboardInterrupt:
        print("\\nDemo interrupted by user")
    
    plotter.close()
    
    print(f"Demo completed! Total frames: {frame_count}")

def screenshot_demo():
    """Create series of robot screenshots"""
    print("Creating robot screenshot series...")
    
    os.makedirs("output", exist_ok=True)
    
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
        # Create off-screen plotter
        plotter = pv.Plotter(off_screen=True, window_size=(1024, 768))
        plotter.set_background('white')
        
        # Ground plane
        ground = pv.Plane(center=[0, 0, -0.2], direction=[0, 0, 1], 
                         i_size=8, j_size=8, i_resolution=16, j_resolution=16)
        plotter.add_mesh(ground, color='lightgray', opacity=0.4)
        
        # Create and transform robot
        robot = create_robot_mesh()
        transform_matrix = pose.to_transformation_matrix()
        robot.transform(transform_matrix, inplace=True)
        plotter.add_mesh(robot, color='blue', opacity=0.9)
        
        # Add target marker
        target = pv.Sphere(center=[0, 0, 1], radius=0.2, phi_resolution=12, theta_resolution=12)
        plotter.add_mesh(target, color='red', opacity=0.7)
        
        # Set camera
        plotter.camera_position = [(8, 8, 6), (0, 0, 1), (0, 0, 1)]
        
        # Save screenshot
        filename = f"output/pyvista_robot_{i:02d}.png"
        plotter.screenshot(filename)
        plotter.close()
        
        print(f"Saved: {filename}")
    
    print(f"✅ Screenshot series complete! 8 images saved in output/")

def main():
    """Main demo selector"""
    print("PyVista Robot Visualization Demo")
    print("=" * 40)
    
    if len(sys.argv) > 1:
        duration = float(sys.argv[1])
    else:
        duration = 10
    
    print(f"Running interactive demo for {duration} seconds...")
    print("Note: This will open a 3D visualization window")
    
    try:
        # Try interactive demo first
        interactive_demo(duration)
    except Exception as e:
        print(f"Interactive demo failed: {e}")
        print("Falling back to screenshot demo...")
        screenshot_demo()

if __name__ == "__main__":
    main()