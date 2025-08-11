#!/usr/bin/env python3
"""
Mesh Robot Examples using External Repositories

This example demonstrates loading and controlling robots with 3D meshes
from external repositories like TurtleBot3 and UR5.

Features:
- Automatic repository cloning
- 3D mesh visualization
- Simple robot control
- Performance optimization

Usage:
    python mesh_robots.py
    python mesh_robots.py --robot turtlebot3 --variant waffle_pi
    python mesh_robots.py --robot ur5 --variant ur5e
    python mesh_robots.py --setup-repos  # Setup repositories only
"""

import sys
import os
import time
import math
import argparse
from typing import Optional

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.simulation_manager import SimulationManager
from core.external_mesh_manager import ExternalMeshManager, setup_external_repositories
from core.robot import create_robot_from_urdf
from core.simulation_object import Velocity


def turtlebot3_demo(variant: str = 'waffle_pi', duration: float = 15.0):
    """TurtleBot3 mesh visualization and control demo"""
    print(f"🤖 TurtleBot3 Demo - Variant: {variant}")
    print("=" * 50)
    
    # Get URDF path from external repository
    mesh_manager = ExternalMeshManager()
    urdf_path = mesh_manager.get_urdf_path('turtlebot3', variant)
    
    if not urdf_path:
        print("❌ TurtleBot3 URDF not found. Setting up repository...")
        if not mesh_manager.clone_repository('turtlebot3'):
            print("❌ Failed to setup TurtleBot3 repository")
            return False
        urdf_path = mesh_manager.get_urdf_path('turtlebot3', variant)
    
    if not urdf_path:
        print(f"❌ TurtleBot3 variant '{variant}' not available")
        return False
    
    print(f"📁 URDF: {urdf_path}")
    
    # Create simulation
    sim = SimulationManager()
    
    try:
        # Add TurtleBot3 with mesh support
        robot = sim.add_robot_from_urdf(
            name="turtlebot3",
            urdf_path=urdf_path
        )
        
        print("✅ TurtleBot3 loaded with 3D meshes")
        
        # Mobile robot control
        def turtlebot_control(dt: float):
            """TurtleBot3 autonomous navigation pattern"""
            t = time.time()
            
            # Create interesting movement patterns
            phase = (t % 20) / 20  # 20-second cycle
            
            if phase < 0.25:
                # Forward movement
                velocity = Velocity(linear_x=0.3, angular_z=0.0)
            elif phase < 0.5:
                # Turn left
                velocity = Velocity(linear_x=0.1, angular_z=0.8)
            elif phase < 0.75:
                # Forward again
                velocity = Velocity(linear_x=0.3, angular_z=0.0)
            else:
                # Turn right
                velocity = Velocity(linear_x=0.1, angular_z=-0.8)
            
            sim.set_robot_velocity("turtlebot3", velocity)
            
            # Update title with robot state
            if hasattr(sim.visualizer, 'plotter'):
                pos = robot.get_pose().position
                sim.visualizer.plotter.title = (
                    f"TurtleBot3 {variant} - "
                    f"pos=({pos[0]:.1f}, {pos[1]:.1f}) - "
                    f"t={t:.1f}s"
                )
        
        # Set control callback
        sim.set_robot_control_callback("turtlebot3", turtlebot_control, frequency=20.0)
        
        # Run simulation
        print("🎮 Starting TurtleBot3 autonomous navigation...")
        return sim.run(duration=duration)
        
    except Exception as e:
        print(f"❌ TurtleBot3 demo failed: {e}")
        return False


def ur5_demo(variant: str = 'ur5', duration: float = 15.0):
    """UR5 robot arm mesh visualization and control demo"""
    print(f"🦾 UR5 Demo - Variant: {variant}")
    print("=" * 50)
    
    # Get URDF path from external repository
    mesh_manager = ExternalMeshManager()
    urdf_path = mesh_manager.get_urdf_path('ur5', variant)
    
    if not urdf_path:
        print("❌ UR5 URDF not found. Setting up repository...")
        if not mesh_manager.clone_repository('ur5'):
            print("❌ Failed to setup UR5 repository")
            return False
        urdf_path = mesh_manager.get_urdf_path('ur5', variant)
    
    if not urdf_path:
        print(f"❌ UR5 variant '{variant}' not available")
        return False
    
    print(f"📁 URDF: {urdf_path}")
    
    # Create simulation
    sim = SimulationManager()
    
    try:
        # Add UR5 with mesh support
        robot = sim.add_robot_from_urdf(
            name="ur5_arm",
            urdf_path=urdf_path
        )
        
        print("✅ UR5 loaded with 3D meshes")
        
        # Get movable joints
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        print(f"🎯 Movable joints: {joint_names}")
        
        # Robot arm control
        def ur5_control(dt: float):
            """UR5 coordinated joint motion"""
            t = time.time()
            
            # Create smooth, coordinated arm motions
            joint_positions = {}
            
            for i, joint_name in enumerate(joint_names):
                # Each joint has different motion pattern
                if i == 0:  # Base rotation
                    joint_positions[joint_name] = 0.8 * math.sin(0.3 * t)
                elif i == 1:  # Shoulder
                    joint_positions[joint_name] = 0.5 * math.sin(0.5 * t + math.pi/4)
                elif i == 2:  # Elbow
                    joint_positions[joint_name] = 0.7 * math.sin(0.4 * t + math.pi/2)
                elif i == 3:  # Wrist 1
                    joint_positions[joint_name] = 0.6 * math.sin(0.6 * t + 3*math.pi/4)
                elif i == 4:  # Wrist 2
                    joint_positions[joint_name] = 0.4 * math.sin(0.7 * t + math.pi)
                else:  # Wrist 3
                    joint_positions[joint_name] = 0.3 * math.sin(0.8 * t + 5*math.pi/4)
            
            # Set all joint positions at once
            sim.set_robot_joint_positions("ur5_arm", joint_positions, max_velocity=1.5)
            
            # Update title with joint info
            if hasattr(sim.visualizer, 'plotter'):
                current_positions = robot.get_joint_positions()
                joint_info = ", ".join([f"{name[:8]}={pos:.2f}" 
                                      for name, pos in list(current_positions.items())[:3]])
                sim.visualizer.plotter.title = (
                    f"UR5 {variant} - {joint_info} - t={t:.1f}s"
                )
        
        # Set control callback
        sim.set_robot_control_callback("ur5_arm", ur5_control, frequency=30.0)
        
        # Run simulation
        print("🎮 Starting UR5 coordinated motion...")
        return sim.run(duration=duration)
        
    except Exception as e:
        print(f"❌ UR5 demo failed: {e}")
        return False


def multi_mesh_robot_demo(duration: float = 20.0):
    """Demo with multiple mesh-based robots"""
    print("🤖🦾 Multi-Mesh Robot Demo")
    print("=" * 50)
    
    # Setup repositories
    print("📦 Setting up external repositories...")
    success = setup_external_repositories(['turtlebot3', 'ur5'])
    if not success:
        print("⚠️ Some repositories failed to setup, continuing anyway...")
    
    # Get URDF paths
    mesh_manager = ExternalMeshManager()
    turtlebot_urdf = mesh_manager.get_urdf_path('turtlebot3', 'waffle_pi')
    ur5_urdf = mesh_manager.get_urdf_path('ur5', 'ur5')
    
    if not turtlebot_urdf or not ur5_urdf:
        print("❌ Required robot URDFs not available")
        return False
    
    # Create simulation with both robots
    sim = SimulationManager()
    
    try:
        # Add TurtleBot3
        turtlebot = sim.add_robot_from_urdf("turtlebot3", turtlebot_urdf)
        
        # Add UR5 at different location
        from simulation_object import Pose
        ur5_pose = Pose(x=2.0, y=0.0, z=0.0)  # 2 meters away
        ur5 = sim.add_robot_from_urdf("ur5_arm", ur5_urdf)
        ur5.teleport(ur5_pose)
        
        print("✅ Both robots loaded successfully")
        
        # Control functions
        def turtlebot_control(dt):
            """TurtleBot moves in circle"""
            t = time.time()
            velocity = Velocity(
                linear_x=0.2,
                angular_z=0.3 * math.sin(t * 0.5)
            )
            sim.set_robot_velocity("turtlebot3", velocity)
        
        def ur5_control(dt):
            """UR5 performs slow coordinated motion"""
            t = time.time()
            ur5_joints = [name for name in ur5.get_joint_names() 
                         if ur5.joints[name].joint_type.value != 'fixed']
            
            positions = {}
            for i, joint_name in enumerate(ur5_joints[:3]):  # First 3 joints
                positions[joint_name] = 0.4 * math.sin(0.2 * t + i * math.pi/3)
            
            sim.set_robot_joint_positions("ur5_arm", positions)
        
        # Set control callbacks with different frequencies
        sim.set_robot_control_callback("turtlebot3", turtlebot_control, frequency=20.0)
        sim.set_robot_control_callback("ur5_arm", ur5_control, frequency=15.0)
        
        # Run simulation
        print("🎮 Starting multi-robot mesh simulation...")
        return sim.run(duration=duration)
        
    except Exception as e:
        print(f"❌ Multi-robot demo failed: {e}")
        return False


def mesh_performance_test():
    """Test mesh loading performance and optimization"""
    print("⚡ Mesh Performance Test")
    print("=" * 50)
    
    from core.enhanced_urdf_loader import EnhancedURDFLoader
    
    # Test different robots
    robots_to_test = [
        ('turtlebot3', 'waffle_pi'),
        ('ur5', 'ur5')
    ]
    
    for repo_name, variant in robots_to_test:
        print(f"\n🔬 Testing {repo_name} ({variant})...")
        
        start_time = time.time()
        
        # Load with enhanced loader
        loader = EnhancedURDFLoader()
        
        if loader.load_robot_from_external_repo(repo_name, variant):
            load_time = time.time() - start_time
            
            # Print detailed mesh information
            loader.print_mesh_info()
            
            # Get performance statistics
            stats = loader.get_mesh_statistics()
            
            print(f"\n📊 Performance Results:")
            print(f"  Load time: {load_time:.2f} seconds")
            print(f"  Total file size: {stats['total_file_size_mb']:.1f} MB")
            print(f"  Total faces: {stats['total_faces']:,}")
            print(f"  Mesh optimization: {stats['optimized_meshes']} meshes")
        else:
            print(f"❌ Failed to load {repo_name}")


def setup_repositories_only():
    """Setup external repositories without running demos"""
    print("📦 Setting Up External Repositories")
    print("=" * 50)
    
    manager = ExternalMeshManager()
    
    print("Available repositories:")
    for repo_name in manager.get_supported_robots():
        repo_info = manager.get_repository_info(repo_name)
        print(f"  📁 {repo_name}: {repo_info.url}")
        print(f"     Variants: {manager.list_robot_variants(repo_name)}")
    
    print("\n🔄 Cloning repositories...")
    success = setup_external_repositories()
    
    if success:
        print("\n✅ All repositories setup successfully!")
        manager.print_status()
    else:
        print("\n⚠️ Some repositories failed to setup")
    
    return success


def main():
    """Main function with argument parsing"""
    parser = argparse.ArgumentParser(
        description="Mesh Robot Examples - 3D mesh visualization demos",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python mesh_robots.py                                    # Run all demos
  python mesh_robots.py --robot turtlebot3               # TurtleBot3 only
  python mesh_robots.py --robot ur5 --variant ur5e       # UR5e variant
  python mesh_robots.py --demo multi                     # Multi-robot demo
  python mesh_robots.py --demo performance               # Performance test
  python mesh_robots.py --setup-repos                    # Setup repositories only

Supported Robots:
  turtlebot3: burger, waffle, waffle_pi
  ur5: ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e
        """
    )
    
    parser.add_argument("--robot", choices=['turtlebot3', 'ur5'], 
                       help="Specific robot to demo")
    parser.add_argument("--variant", type=str,
                       help="Robot variant (e.g., waffle_pi, ur5e)")  
    parser.add_argument("--demo", choices=['multi', 'performance'],
                       help="Special demo mode")
    parser.add_argument("--duration", type=float, default=15.0,
                       help="Demo duration in seconds")
    parser.add_argument("--setup-repos", action="store_true",
                       help="Setup repositories only, no demos")
    parser.add_argument("--list-robots", action="store_true",
                       help="List available robots and variants")
    
    args = parser.parse_args()
    
    # List available robots
    if args.list_robots:
        print("🤖 Available External Robots")
        print("=" * 40)
        manager = ExternalMeshManager()
        for repo_name in manager.list_available_repositories():
            variants = manager.list_robot_variants(repo_name)
            print(f"{repo_name}: {', '.join(variants) if variants else 'No variants configured'}")
        return 0
    
    # Setup repositories only
    if args.setup_repos:
        return 0 if setup_repositories_only() else 1
    
    print("🤖 SimPyROS Mesh Robot Examples")
    print("Demonstrating 3D mesh loading and visualization")
    print("=" * 60)
    
    try:
        success = False
        
        # Run specific robot demo
        if args.robot == 'turtlebot3':
            variant = args.variant or 'waffle_pi'
            success = turtlebot3_demo(variant, args.duration)
            
        elif args.robot == 'ur5':
            variant = args.variant or 'ur5'
            success = ur5_demo(variant, args.duration)
            
        # Run special demos
        elif args.demo == 'multi':
            success = multi_mesh_robot_demo(args.duration)
            
        elif args.demo == 'performance':
            mesh_performance_test()
            success = True
            
        # Run all demos
        else:
            demos = [
                ("TurtleBot3 Waffle Pi", lambda: turtlebot3_demo('waffle_pi', 8.0)),
                ("UR5 Robot Arm", lambda: ur5_demo('ur5', 8.0)),
                ("Multi-Robot", lambda: multi_mesh_robot_demo(10.0)),
                ("Performance Test", mesh_performance_test)
            ]
            
            for name, demo_func in demos:
                try:
                    print(f"\n▶️ Running: {name}")
                    result = demo_func()
                    if result is not False:  # None is OK for performance test
                        success = True
                    time.sleep(1)  # Brief pause
                    
                except KeyboardInterrupt:
                    print(f"\n⏹️ {name} interrupted by user")
                    break
                except Exception as e:
                    print(f"❌ {name} failed: {e}")
        
        if success or args.demo == 'performance':
            print("\n🎉 Mesh robot demos completed!")
            print("\nKey features demonstrated:")
            print("  ✅ Automatic external repository cloning")
            print("  ✅ 3D mesh loading and optimization")  
            print("  ✅ Multiple robot variants")
            print("  ✅ Performance optimization")
            print("  ✅ Coordinated multi-robot control")
            return 0
        else:
            print("\n❌ Some demos failed!")
            return 1
            
    except KeyboardInterrupt:
        print("\n⏹️ Interrupted by user")
        return 0
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())