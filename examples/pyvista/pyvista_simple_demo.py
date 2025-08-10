#!/usr/bin/env python3
"""
Simple PyVista Demo for Robot Visualization
Demonstrates basic PyVista functionality using the shared visualizer module
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np
import math
import time
from simulation_object import Pose
from pyvista_visualizer import (
    create_headless_visualizer, 
    setup_basic_scene, 
    create_robot_mesh, 
    RobotMeshFactory
)

def demonstrate_mesh_creation():
    """Demonstrate mesh creation without rendering using the shared module"""
    print("\\n=== PyVista Mesh Creation Demo ===")
    
    # Create a minimal visualizer just to access PyVista
    viz = create_headless_visualizer()
    if not viz.available:
        print("PyVista not available - skipping mesh demo")
        return False
        
    try:
        # Create basic shapes
        print("Creating basic shapes...")
        sphere = viz.pv.Sphere(radius=1.0)
        print(f"Sphere: {sphere.n_points} points, {sphere.n_cells} cells")
        
        box = viz.pv.Box()
        print(f"Box: {box.n_points} points, {box.n_cells} cells")
        
        cylinder = viz.pv.Cylinder()
        print(f"Cylinder: {cylinder.n_points} points, {cylinder.n_cells} cells")
        
        # Create robot meshes using factory
        print("\\nCreating robot meshes...")
        basic_robot = RobotMeshFactory.create_basic_robot(viz.pv)
        if basic_robot:
            print(f"Basic robot: {basic_robot.n_points} points, {basic_robot.n_cells} cells")
            
        wheeled_robot = RobotMeshFactory.create_wheeled_robot(viz.pv)
        if wheeled_robot:
            print(f"Wheeled robot: {wheeled_robot.n_points} points, {wheeled_robot.n_cells} cells")
            
        quadcopter = RobotMeshFactory.create_quadcopter(viz.pv)
        if quadcopter:
            print(f"Quadcopter: {quadcopter.n_points} points, {quadcopter.n_cells} cells")
            
        # Test transformations
        print("\\nTesting transformations...")
        pose = Pose(x=1, y=2, z=3, yaw=math.pi/4)
        transform_matrix = pose.to_transformation_matrix()
        print(f"Transform matrix shape: {transform_matrix.shape}")
        
        if basic_robot:
            transformed_robot = basic_robot.copy()
            transformed_robot.transform(transform_matrix, inplace=True)
            print("Robot transformation successful")
            
        print("\\n✅ Mesh creation and transformation successful!")
        return True
        
    except Exception as e:
        print(f"❌ Mesh demo error: {e}")
        return False

def test_headless_rendering():
    """Test headless rendering with screenshot using the shared module"""
    print("\\n=== PyVista Headless Rendering Test ===")
    
    viz = create_headless_visualizer()
    if not viz.available:
        print("PyVista not available - skipping rendering test")
        return False
        
    try:
        print("Creating headless scene...")
        
        # Setup basic scene
        setup_basic_scene(viz)
        
        # Create and add robot
        robot = create_robot_mesh(viz, 'basic')
        if robot:
            viz.plotter.add_mesh(robot, color='blue', opacity=0.8)
            print("Robot mesh added to scene")
        
        # Save screenshot
        os.makedirs("../output", exist_ok=True)
        screenshot_path = "../output/pyvista_test.png"
        viz.plotter.screenshot(screenshot_path)
        viz.plotter.close()
        
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
    """Test animation sequence by creating multiple screenshots using shared module"""
    print("\\n=== PyVista Animation Sequence Test ===")
    
    # Test if visualizer is available
    test_viz = create_headless_visualizer()
    if not test_viz.available:
        print("PyVista not available - skipping animation test")
        test_viz.plotter.close()
        return False
    test_viz.plotter.close()
        
    try:
        os.makedirs("../output", exist_ok=True)
        poses = robot_animation_poses()
        
        print(f"Creating {len(poses)} animation frames...")
        
        for i, pose in enumerate(poses):
            # Create new visualizer for each frame
            viz = create_headless_visualizer()
            
            # Setup scene
            setup_basic_scene(viz)
            
            # Create robot at specific pose
            robot = create_robot_mesh(viz, 'wheeled')
            if robot:
                transform_matrix = pose.to_transformation_matrix()
                robot.transform(transform_matrix, inplace=True)
                viz.plotter.add_mesh(robot, color='blue', opacity=0.8)
            
            # Add target marker
            target = viz.pv.Sphere(center=[0, 0, 1], radius=0.2, phi_resolution=12, theta_resolution=12)
            viz.plotter.add_mesh(target, color='red', opacity=0.7)
            
            # Save frame
            frame_path = f"../output/pyvista_frame_{i:03d}.png"
            viz.plotter.screenshot(frame_path)
            viz.plotter.close()
            
            print(f"Frame {i+1}/{len(poses)} saved")
            
        print(f"✅ Animation sequence complete! {len(poses)} frames saved")
        return True
        
    except Exception as e:
        print(f"❌ Animation sequence error: {e}")
        return False

def test_robot_types():
    """Test different robot types using the factory"""
    print("\\n=== Robot Types Test ===")
    
    viz = create_headless_visualizer()
    if not viz.available:
        print("PyVista not available - skipping robot types test")
        return False
    
    robot_types = ['basic', 'wheeled', 'quadcopter']
    
    try:
        os.makedirs("../output", exist_ok=True)
        
        for i, robot_type in enumerate(robot_types):
            # Create new scene
            viz_scene = create_headless_visualizer()
            setup_basic_scene(viz_scene)
            
            # Create robot of specific type
            robot = create_robot_mesh(viz_scene, robot_type)
            if robot:
                viz_scene.plotter.add_mesh(robot, color='green', opacity=0.9)
                print(f"Created {robot_type} robot successfully")
            else:
                print(f"Failed to create {robot_type} robot")
                continue
            
            # Save screenshot
            filename = f"../output/pyvista_robot_type_{robot_type}.png"
            viz_scene.plotter.screenshot(filename)
            viz_scene.plotter.close()
            
            print(f"Saved: {filename}")
        
        viz.plotter.close()
        print(f"✅ Robot types test complete!")
        return True
        
    except Exception as e:
        print(f"❌ Robot types test error: {e}")
        viz.plotter.close()
        return False

def main():
    """Main demo function using the shared visualizer module"""
    print("PyVista Robot Visualization - Simple Demo")
    print("Using shared pyvista_visualizer module")
    print("=" * 50)
    
    success_count = 0
    total_tests = 4
    
    # Test 1: Mesh creation
    if demonstrate_mesh_creation():
        success_count += 1
        
    # Test 2: Headless rendering
    if test_headless_rendering():
        success_count += 1
        
    # Test 3: Animation sequence
    if test_animation_sequence():
        success_count += 1
    
    # Test 4: Robot types
    if test_robot_types():
        success_count += 1
    
    print("\\n" + "=" * 50)
    print(f"Demo Results: {success_count}/{total_tests} tests passed")
    
    if success_count == total_tests:
        print("🎉 All PyVista tests successful!")
        print("PyVista integration is working correctly.")
        print("\\nGenerated files:")
        print("  - ../output/pyvista_test.png (basic rendering)")
        print("  - ../output/pyvista_frame_*.png (animation sequence)")
        print("  - ../output/pyvista_robot_type_*.png (different robot types)")
    elif success_count > 0:
        print("⚠️ Partial success - some PyVista features working")
    else:
        print("❌ PyVista integration issues detected")
        print("Check PyVista installation and dependencies")
        
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