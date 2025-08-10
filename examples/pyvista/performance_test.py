#!/usr/bin/env python3
"""
PyVista Performance Test
Compare optimized vs non-optimized animation performance
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import time
import math
from simulation_object import Pose
from pyvista_visualizer import create_headless_visualizer, setup_basic_scene, create_robot_mesh, AnimationController

def test_optimized_performance(num_frames=100):
    """Test optimized animation performance"""
    print(f"Testing optimized performance with {num_frames} frames...")
    
    viz = create_headless_visualizer()
    if not viz.available:
        print("PyVista not available")
        return 0
    
    setup_basic_scene(viz)
    controller = AnimationController(viz.plotter, viz.pv)
    
    # Create robot once
    robot_mesh = create_robot_mesh(viz, 'wheeled')
    controller.add_robot("test_robot", robot_mesh)
    
    start_time = time.time()
    
    for i in range(num_frames):
        # Create different poses
        t = i * 0.1
        x = 2 * math.sin(t)
        y = 2 * math.cos(t)
        z = 1 + 0.2 * math.sin(t * 2)
        yaw = t
        
        pose = Pose(x=x, y=y, z=z, yaw=yaw)
        
        # Update using optimized method
        controller.update_robot_pose("test_robot", pose)
    
    end_time = time.time()
    viz.plotter.close()
    
    duration = end_time - start_time
    fps = num_frames / duration
    
    print(f"Optimized: {duration:.3f}s for {num_frames} frames = {fps:.1f} FPS")
    return fps

def test_fallback_performance(num_frames=100):
    """Test fallback (mesh recreation) performance"""
    print(f"Testing fallback performance with {num_frames} frames...")
    
    viz = create_headless_visualizer()
    if not viz.available:
        print("PyVista not available")
        return 0
    
    setup_basic_scene(viz)
    controller = AnimationController(viz.plotter, viz.pv)
    
    # Create robot once
    robot_mesh = create_robot_mesh(viz, 'wheeled')
    controller.add_robot("test_robot", robot_mesh)
    
    start_time = time.time()
    
    for i in range(num_frames):
        # Create different poses
        t = i * 0.1
        x = 2 * math.sin(t)
        y = 2 * math.cos(t)
        z = 1 + 0.2 * math.sin(t * 2)
        yaw = t
        
        pose = Pose(x=x, y=y, z=z, yaw=yaw)
        
        # Update using fallback method (mesh recreation)
        def mesh_creator():
            return create_robot_mesh(viz, 'wheeled')
            
        controller.update_robot_pose_fallback("test_robot", pose, mesh_creator)
    
    end_time = time.time()
    viz.plotter.close()
    
    duration = end_time - start_time
    fps = num_frames / duration
    
    print(f"Fallback: {duration:.3f}s for {num_frames} frames = {fps:.1f} FPS")
    return fps

def main():
    """Run performance comparison"""
    print("PyVista Animation Performance Test")
    print("=" * 40)
    
    # Test parameters
    test_frames = 50  # Reduced for quick testing
    
    # Run tests
    optimized_fps = test_optimized_performance(test_frames)
    fallback_fps = test_fallback_performance(test_frames)
    
    print("=" * 40)
    print("Performance Results:")
    print(f"Optimized method: {optimized_fps:.1f} FPS")
    print(f"Fallback method:  {fallback_fps:.1f} FPS")
    
    if optimized_fps > 0 and fallback_fps > 0:
        speedup = optimized_fps / fallback_fps
        print(f"Speedup: {speedup:.2f}x faster")
        
        if speedup > 2.0:
            print("🚀 Significant performance improvement!")
        elif speedup > 1.5:
            print("✅ Good performance improvement")
        else:
            print("⚠️ Modest performance improvement")
    
    print("\nRecommendation:")
    print("- Use update_robot_pose() for simple position/rotation changes")
    print("- Use update_robot_pose_fallback() only when robot shape changes")

if __name__ == "__main__":
    main()