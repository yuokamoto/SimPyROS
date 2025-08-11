#!/usr/bin/env python3
"""
Visualization Demos for SimPyROS

Demo functions separated from the core pyvista_visualizer library.
These demonstrate the visualization capabilities without being part of the core library.
"""

import os
import sys
import math
import time
import threading
from typing import Optional

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.pyvista_visualizer import create_urdf_robot_visualizer


def quick_robot_demo(robot_instance, urdf_path: str, robot_name: str = "demo_robot", 
                    demo_type: str = "simple", duration: float = 30.0):
    """
    Quick robot demonstration with various motion patterns
    
    Args:
        robot_instance: Robot object instance
        urdf_path: Path to URDF file
        robot_name: Name for the robot in visualization
        demo_type: Type of demo ('simple', 'realtime', 'mixed')
        duration: Demo duration in seconds
        
    Returns:
        Success status
    """
    print(f"🎮 Quick Robot Demo: {demo_type}")
    print(f"URDF: {urdf_path}")
    print(f"Duration: {duration} seconds")
    print("=" * 50)
    
    # Create visualizer
    visualizer = create_urdf_robot_visualizer()
    if not visualizer.available:
        print("❌ Visualization not available")
        return False
    
    # Load robot
    if not visualizer.load_robot(robot_name, robot_instance, urdf_path):
        print("❌ Failed to load robot")
        return False
    
    # Get robot info
    robot_info = visualizer.get_robot_info(robot_name)
    movable_joints = robot_info['movable_joints']
    
    if not movable_joints:
        print("❌ No movable joints found")
        return False
    
    print(f"🎮 Starting {demo_type} demo with joints: {movable_joints}")
    
    # Demo motion patterns
    def run_demo():
        start_time = time.time()
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            if demo_type == "simple":
                # Simple sinusoidal motion
                for i, joint_name in enumerate(movable_joints):
                    phase_offset = i * math.pi / 2
                    position = 0.5 * math.sin(0.5 * t + phase_offset)
                    visualizer.set_joint_command(robot_name, joint_name, position, max_velocity=1.0)
                    
            elif demo_type == "realtime":
                # Fast coordinated motion
                for i, joint_name in enumerate(movable_joints):
                    phase_offset = i * 2 * math.pi / len(movable_joints)
                    amplitude = 0.6
                    frequency = 1.2
                    position = amplitude * math.sin(frequency * t + phase_offset)
                    visualizer.set_joint_command(robot_name, joint_name, position, max_velocity=3.0)
                    
            else:  # mixed
                # Phase-based motion
                phase_duration = duration / 3.0
                phase = int(t / phase_duration)
                
                if phase == 0:
                    # Slow individual joints
                    active_joint = int(t * 2 / phase_duration) % len(movable_joints)
                    for i, joint_name in enumerate(movable_joints):
                        if i == active_joint:
                            position = 0.8 * math.sin(4 * t)
                            visualizer.set_joint_command(robot_name, joint_name, position, max_velocity=2.0)
                        else:
                            # Return others to home gradually
                            current_pos = robot_instance.get_joint_state(joint_name).position
                            target_pos = current_pos * 0.95
                            visualizer.set_joint_command(robot_name, joint_name, target_pos, max_velocity=1.0)
                elif phase == 1:
                    # Fast coordinated
                    for i, joint_name in enumerate(movable_joints):
                        phase_offset = i * math.pi / 3
                        position = 0.6 * math.sin(1.5 * t + phase_offset)
                        visualizer.set_joint_command(robot_name, joint_name, position, max_velocity=2.5)
                else:
                    # Return home
                    for joint_name in movable_joints:
                        current_pos = robot_instance.get_joint_state(joint_name).position
                        target_pos = current_pos * 0.9
                        visualizer.set_joint_command(robot_name, joint_name, target_pos, max_velocity=1.0)
            
            # Update title
            positions = robot_instance.get_joint_positions()
            joint_info = ", ".join([f"{name[:8]}={pos:.2f}" for name, pos in list(positions.items())[:3]])
            visualizer.plotter.title = f"Robot Demo - t={t:.1f}s - {joint_info}"
            
            time.sleep(1/30)  # 30 Hz update
    
    try:
        if visualizer.display_available:
            # Start demo in background thread
            demo_thread = threading.Thread(target=run_demo)
            demo_thread.daemon = True
            demo_thread.start()
            
            # Show interactive window
            visualizer.plotter.show()
        else:
            # Headless mode
            run_demo()
            
            # Take screenshot
            os.makedirs("output", exist_ok=True)
            visualizer.plotter.screenshot("output/quick_robot_demo.png")
            print("📸 Screenshot saved: output/quick_robot_demo.png")
        
        return True
        
    except Exception as e:
        print(f"❌ Demo error: {e}")
        return False


def interactive_robot_demo(urdf_path: str, robot_name: str = "interactive_robot"):
    """
    Interactive robot demo with user controls
    
    Args:
        urdf_path: Path to URDF file
        robot_name: Name for the robot
    """
    print("🕹️ Interactive Robot Demo")
    print("Use keyboard controls to move robot joints")
    print("=" * 50)
    
    # Create visualizer
    visualizer = create_urdf_robot_visualizer(interactive=True)
    if not visualizer.available:
        print("❌ Visualization not available")
        return False
    
    # Create robot for interactive demo
    import simpy
    from robot import create_robot_from_urdf
    
    env = simpy.Environment()
    robot = create_robot_from_urdf(env, urdf_path, robot_name)
    
    # Load robot
    if not visualizer.load_robot(robot_name, robot, urdf_path):
        print("❌ Failed to load robot")
        return False
    
    # Get movable joints
    robot_info = visualizer.get_robot_info(robot_name)
    movable_joints = robot_info['movable_joints']
    
    if not movable_joints:
        print("❌ No movable joints found")
        return False
    
    print("🎮 Controls:")
    for i, joint in enumerate(movable_joints[:9]):  # Max 9 joints for number keys
        print(f"  {i+1}: Move {joint}")
    print("  Q/W: Increase/Decrease joint speed")
    print("  R: Reset all joints to home position")
    print("  ESC: Exit")
    
    # Interactive control state
    joint_speeds = {joint: 0.0 for joint in movable_joints}
    speed_multiplier = 0.5
    
    def update_robot():
        """Update robot based on current joint speeds"""
        while True:
            for joint_name, speed in joint_speeds.items():
                if abs(speed) > 0.01:  # Only move if significant speed
                    current_pos = robot.get_joint_state(joint_name).position
                    new_pos = current_pos + speed * 0.1  # Small increment
                    visualizer.set_joint_command(robot_name, joint_name, new_pos, max_velocity=2.0)
            
            time.sleep(1/30)  # 30 Hz update
    
    # Start update thread
    update_thread = threading.Thread(target=update_robot)
    update_thread.daemon = True
    update_thread.start()
    
    print("\n🚀 Starting interactive demo...")
    
    try:
        # Show window (this will handle keyboard input)
        visualizer.plotter.title = f"Interactive Robot: {robot_name} - Use number keys to control joints"
        visualizer.plotter.show()
        return True
        
    except Exception as e:
        print(f"❌ Interactive demo error: {e}")
        return False


def create_robot_mesh_demo(visualizer, robot_type: str = 'basic', urdf_path: str = None, package_path: str = None):
    """
    Create a robot mesh for demonstration purposes
    
    Args:
        visualizer: PyVistaVisualizer instance
        robot_type: Type of robot mesh ('basic', 'urdf', etc.)
        urdf_path: Path to URDF file (if robot_type='urdf')
        package_path: ROS package path (optional)
        
    Returns:
        Robot mesh object or None
    """
    if not visualizer.available:
        return None
        
    if robot_type == 'urdf':
        if not urdf_path:
            print("❌ URDF path required for robot_type='urdf'")
            return None
        
        # Use URDF loader to create mesh
        try:
            from core.urdf_loader import URDFLoader
            loader = URDFLoader(package_path)
            
            if loader.load_urdf(urdf_path):
                return loader.create_pyvista_meshes(visualizer.pv)
            else:
                print("❌ Failed to load URDF")
                return None
                
        except ImportError:
            print("❌ URDF loader not available")
            return None
    else:
        # Use SampleRobotFactory for built-in geometric robots
        try:
            from examples.pyvista.sample_robots import SampleRobotFactory
            return SampleRobotFactory.create_robot_mesh(visualizer.pv, robot_type)
        except ImportError:
            print("❌ SampleRobotFactory not available for geometric robot types")
            return None


def main():
    """Main demo launcher"""
    import argparse
    
    parser = argparse.ArgumentParser(description="SimPyROS Visualization Demos")
    parser.add_argument("--demo", choices=["quick", "interactive"], default="quick",
                       help="Demo type to run")
    parser.add_argument("--urdf", required=True, help="Path to URDF file")
    parser.add_argument("--duration", type=float, default=30.0,
                       help="Demo duration in seconds (for quick demo)")
    parser.add_argument("--pattern", choices=["simple", "realtime", "mixed"], default="simple",
                       help="Motion pattern (for quick demo)")
    
    args = parser.parse_args()
    
    print("🎬 SimPyROS Visualization Demos")
    print("=" * 50)
    
    if args.demo == "quick":
        # Create robot for quick demo
        import simpy
        from robot import create_robot_from_urdf
        
        env = simpy.Environment()
        robot = create_robot_from_urdf(env, args.urdf, "demo_robot")
        
        success = quick_robot_demo(
            robot, args.urdf, "demo_robot", 
            args.pattern, args.duration
        )
    else:  # interactive
        success = interactive_robot_demo(args.urdf, "interactive_robot")
    
    if success:
        print("✅ Demo completed successfully!")
        return 0
    else:
        print("❌ Demo failed!")
        return 1


if __name__ == "__main__":
    sys.exit(main())