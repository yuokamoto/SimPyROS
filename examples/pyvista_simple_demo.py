#!/usr/bin/env python3
"""
Simple PyVista Demo for Robot Visualization
Demonstrates basic PyVista functionality with fallback support
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

# Try to import PyVista with fallback
try:
    import pyvista as pv
    PYVISTA_AVAILABLE = True
    print("PyVista imported successfully")
except ImportError as e:
    PYVISTA_AVAILABLE = False
    print(f"PyVista not available: {e}")

from simulation_object import Pose
import math
import time


def create_simple_robot_mesh():
    """Create a simple robot mesh using basic PyVista shapes"""
    if not PYVISTA_AVAILABLE:
        return None
        
    try:
        # Robot base (box)
        base = pv.Box(bounds=[-0.5, 0.5, -0.3, 0.3, 0, 0.2])
        
        # Robot arm (cylinder)
        arm = pv.Cylinder(center=[0.3, 0, 0.4], direction=[0, 0, 1], 
                         radius=0.1, height=0.6)
        
        # End effector (sphere)
        gripper = pv.Sphere(center=[0.3, 0, 0.8], radius=0.08)
        
        # Combine meshes
        robot = base + arm + gripper
        
        print("Robot mesh created successfully")
        return robot
        
    except Exception as e:
        print(f"Error creating robot mesh: {e}")
        return None


def demonstrate_mesh_creation():
    """Demonstrate mesh creation without rendering"""
    print("\\n=== PyVista Mesh Creation Demo ===")
    
    if not PYVISTA_AVAILABLE:
        print("PyVista not available - skipping mesh demo")
        return False
        
    try:
        # Create basic shapes
        print("Creating basic shapes...")
        sphere = pv.Sphere(radius=1.0)
        print(f"Sphere: {sphere.n_points} points, {sphere.n_cells} cells")
        
        box = pv.Box()
        print(f"Box: {box.n_points} points, {box.n_cells} cells")
        
        cylinder = pv.Cylinder()
        print(f"Cylinder: {cylinder.n_points} points, {cylinder.n_cells} cells")
        
        # Create robot mesh
        print("\\nCreating robot mesh...")
        robot = create_simple_robot_mesh()
        if robot:
            print(f"Robot mesh: {robot.n_points} points, {robot.n_cells} cells")
            
        # Test transformations
        print("\\nTesting transformations...")
        pose = Pose(x=1, y=2, z=3, yaw=math.pi/4)
        transform_matrix = pose.to_transformation_matrix()
        print(f"Transform matrix shape: {transform_matrix.shape}")
        
        if robot:
            transformed_robot = robot.copy()
            transformed_robot.transform(transform_matrix, inplace=True)
            print("Robot transformation successful")
            
        print("\\n✅ Mesh creation and transformation successful!")
        return True
        
    except Exception as e:
        print(f"❌ Mesh demo error: {e}")
        return False


def test_headless_rendering():
    """Test headless rendering with screenshot"""
    print("\\n=== PyVista Headless Rendering Test ===")
    
    if not PYVISTA_AVAILABLE:
        print("PyVista not available - skipping rendering test")
        return False
        
    try:
        # Create plotter with off-screen rendering
        print("Creating off-screen plotter...")
        plotter = pv.Plotter(off_screen=True, window_size=(800, 600))
        plotter.set_background('white')
        
        # Create and add robot
        robot = create_simple_robot_mesh()
        if robot:
            plotter.add_mesh(robot, color='blue', opacity=0.8)
            print("Robot mesh added to plotter")
        
        # Add coordinate axes
        axes = pv.Arrow(start=[0, 0, 0], direction=[1, 0, 0], scale=1.0)
        plotter.add_mesh(axes, color='red')
        
        # Set camera position
        plotter.camera_position = [(3, 3, 3), (0, 0, 0.5), (0, 0, 1)]
        
        # Save screenshot
        os.makedirs("output", exist_ok=True)
        screenshot_path = "output/pyvista_test.png"
        plotter.screenshot(screenshot_path)
        plotter.close()
        
        print(f"✅ Screenshot saved: {screenshot_path}")
        return True
        
    except Exception as e:
        print(f"❌ Headless rendering error: {e}")
        return False


def robot_animation_poses():
    """Generate poses for robot animation"""
    poses = []
    for i in range(10):
        t = i * 0.1
        x = 2 * math.cos(t)
        y = 2 * math.sin(t)
        z = 1 + 0.2 * math.sin(2*t)
        yaw = t
        poses.append(Pose(x=x, y=y, z=z, yaw=yaw))
    return poses


def test_animation_sequence():
    """Test animation sequence by creating multiple screenshots"""
    print("\\n=== PyVista Animation Sequence Test ===")
    
    if not PYVISTA_AVAILABLE:
        print("PyVista not available - skipping animation test")
        return False
        
    try:
        os.makedirs("output", exist_ok=True)
        poses = robot_animation_poses()
        
        print(f"Creating {len(poses)} animation frames...")
        
        for i, pose in enumerate(poses):
            # Create plotter for each frame
            plotter = pv.Plotter(off_screen=True, window_size=(800, 600))
            plotter.set_background('white')
            
            # Create robot at specific pose
            robot = create_simple_robot_mesh()
            if robot:
                transform_matrix = pose.to_transformation_matrix()
                robot.transform(transform_matrix, inplace=True)
                plotter.add_mesh(robot, color='blue', opacity=0.8)
            
            # Add ground plane
            ground = pv.Plane(center=[0, 0, -0.1], direction=[0, 0, 1], 
                             i_size=8, j_size=8)
            plotter.add_mesh(ground, color='lightgray', opacity=0.5)
            
            # Set camera
            plotter.camera_position = [(6, 6, 4), (0, 0, 1), (0, 0, 1)]
            
            # Save frame
            frame_path = f"output/pyvista_frame_{i:03d}.png"
            plotter.screenshot(frame_path)
            plotter.close()
            
            print(f"Frame {i+1}/{len(poses)} saved")
            
        print(f"✅ Animation sequence complete! {len(poses)} frames saved")
        return True
        
    except Exception as e:
        print(f"❌ Animation sequence error: {e}")
        return False


def main():
    """Main demo function"""
    print("PyVista Robot Visualization - Simple Demo")
    print("=" * 50)
    
    success_count = 0
    total_tests = 3
    
    # Test 1: Mesh creation
    if demonstrate_mesh_creation():
        success_count += 1
        
    # Test 2: Headless rendering
    if test_headless_rendering():
        success_count += 1
        
    # Test 3: Animation sequence
    if test_animation_sequence():
        success_count += 1
    
    print("\\n" + "=" * 50)
    print(f"Demo Results: {success_count}/{total_tests} tests passed")
    
    if success_count == total_tests:
        print("🎉 All PyVista tests successful!")
        print("PyVista integration is working correctly.")
        print("\\nGenerated files:")
        print("  - output/pyvista_test.png (single robot)")
        print("  - output/pyvista_frame_*.png (animation sequence)")
    elif success_count > 0:
        print("⚠️ Partial success - some PyVista features working")
    else:
        print("❌ PyVista integration issues detected")
        print("Consider using matplotlib visualization instead")
        
    return success_count == total_tests


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\\nDemo interrupted by user")
    except Exception as e:
        print(f"\\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()