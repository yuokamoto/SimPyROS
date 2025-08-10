#!/usr/bin/env python3
"""
Simple PyVista Demo for Robot Visualization
Demonstrates basic PyVista functionality using the shared visualizer module
"""

import sys
import os
import numpy as np
import math
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from simulation_object import Pose
from pyvista_visualizer import (
    create_headless_visualizer, 
    setup_basic_scene
)
from sample_robots import SampleRobotFactory

OUTPUT_FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'output')
os.makedirs(OUTPUT_FILE_PATH, exist_ok=True)


def demonstrate_mesh_creation():
    """Demonstrate mesh creation without rendering using the shared module"""
    print("\n=== PyVista Mesh Creation Demo ===")
    
    # Create a minimal visualizer just to access PyVista
    viz = create_headless_visualizer()
    if not viz.available:
        print("PyVista not available - skipping mesh demo")
        return False
        
    try:
        setup_basic_scene(viz)

        def headless_screen_shot_save(mesh, postfix):
            actor = viz.plotter.add_mesh(mesh)
            screenshot_path = os.path.join(OUTPUT_FILE_PATH, f'pyvista_demonstrate_mesh_{postfix}.png')
            viz.plotter.screenshot(screenshot_path)
            viz.plotter.remove_actor(actor)


        # Create basic shapes
        print("Creating basic shapes...")

        sphere = viz.pv.Sphere(radius=1.0)
        headless_screen_shot_save(sphere, 'sphere')
        print(f"Sphere: {sphere.n_points} points, {sphere.n_cells} cells")
        
        box = viz.pv.Box()
        headless_screen_shot_save(box, 'box')
        print(f"Box: {box.n_points} points, {box.n_cells} cells")
        
        cylinder = viz.pv.Cylinder()
        headless_screen_shot_save(cylinder, 'cylinder')
        print(f"Cylinder: {cylinder.n_points} points, {cylinder.n_cells} cells")
        
        # Create robot meshes using factory
        print("\nCreating robot meshes...")
        def create_robot_and_save(robot_type):
            robot = SampleRobotFactory.create_robot_mesh(viz.pv, robot_type) 
            if robot:
                headless_screen_shot_save(robot, robot_type)
                print(f"{robot_type}: {robot.n_points} points, {robot.n_cells} cells")
            return robot
        basic_robot = create_robot_and_save('basic_robot')
        wheeled_robot = create_robot_and_save('wheeled_robot')
        quadcopter = create_robot_and_save('quadcopter')
            
        # Test transformations
        print("\nTesting transformations...")
        pose = Pose(x=1, y=2, z=3, yaw=math.pi/4)
        transform_matrix = pose.to_transformation_matrix()
        print(f"Transform matrix shape: {transform_matrix.shape}")
        
        if basic_robot:
            transformed_robot = basic_robot.copy()
            transformed_robot.transform(transform_matrix, inplace=True)
            headless_screen_shot_save(transformed_robot, 'transformed_robot')
            print("Robot transformation successful")
            
        print("\n✅ Mesh creation and transformation successful!")
        return True
        
    except Exception as e:
        print(f"❌ Mesh demo error: {e}")
        return False

def test_animation_sequence():
    """Test animation sequence by creating multiple screenshots using shared module"""
    print("\n=== PyVista Animation Sequence Test ===")
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
    # Test if visualizer is available
    test_viz = create_headless_visualizer()
    if not test_viz.available:
        print("PyVista not available - skipping animation test")
        test_viz.plotter.close()
        return False
    test_viz.plotter.close()
        
    try:
        poses = robot_animation_poses()
        
        print(f"Creating {len(poses)} animation frames...")
        
        for i, pose in enumerate(poses):
            # Create new visualizer for each frame
            viz = create_headless_visualizer()
            
            # Setup scene
            setup_basic_scene(viz)
            
            # Create robot at specific pose
            robot = SampleRobotFactory.create_wheeled_robot(viz.pv)
            if robot:
                transform_matrix = pose.to_transformation_matrix()
                robot.transform(transform_matrix, inplace=True)
                viz.plotter.add_mesh(robot, color='blue', opacity=0.8)
            
            # Add target marker
            target = viz.pv.Sphere(center=[0, 0, 1], radius=0.2, phi_resolution=12, theta_resolution=12)
            viz.plotter.add_mesh(target, color='red', opacity=0.7)
            
            # Save frame
            frame_path =  os.path.join(OUTPUT_FILE_PATH, f"pyvista_frame_{i:03d}.png")
            viz.plotter.screenshot(frame_path)
            viz.plotter.close()
            
            print(f"Frame {i+1}/{len(poses)} saved")
            
        print(f"✅ Animation sequence complete! {len(poses)} frames saved")
        return True
        
    except Exception as e:
        print(f"❌ Animation sequence error: {e}")
        return False


def main():
    """Main demo function using the shared visualizer module"""
    print("PyVista Robot Visualization - Simple Demo")
    print("Using shared pyvista_visualizer module")
    print("=" * 50)
    
    success_count = 0
    total_tests = 0
    
    # Test 1: Mesh creation and headless rendering
    total_tests += 1
    if demonstrate_mesh_creation():
        success_count += 1
        
    # Test 2: Animation sequence
    total_tests += 1
    if test_animation_sequence():
        success_count += 1
    
    
    print("\n" + "=" * 50)
    print(f"Demo Results: {success_count}/{total_tests} tests passed")
    
    if success_count == total_tests:
        print("🎉 All PyVista tests successful!")
        print("PyVista integration is working correctly.")
        print("\nGenerated files:")
        print("  - ../../output/pyvista_test.png (basic rendering)")
        print("  - ../../output/pyvista_frame_*.png (animation sequence)")
        print("  - ../../output/pyvista_robot_type_*.png (different robot types)")
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
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()