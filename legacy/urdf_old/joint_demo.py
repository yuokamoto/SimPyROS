#!/usr/bin/env python3
"""
Unified Joint Motion Demo - URDF based robot with interactive joint control
Combines the functionality of simple_joint_demo.py and realtime_joint_demo.py
"""

import sys
import os
import time
import math
import numpy as np
import argparse

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import simpy
from robot import create_robot_from_urdf
from simulation_object import Pose
from pyvista_visualizer import create_urdf_robot_visualizer, quick_robot_demo


def interactive_joint_demo(urdf_path: str, duration: float = 20.0, demo_mode: str = "mixed"):
    """
    Interactive joint motion demo using the new URDFRobotVisualizer
    
    Args:
        urdf_path: Path to URDF file
        duration: Demo duration in seconds  
        demo_mode: "simple", "realtime", or "mixed"
    """
    print("🤖🔧 Unified URDF Joint Motion Demo")
    print("=" * 60)
    print(f"URDF: {urdf_path}")
    print(f"Duration: {duration} seconds")
    print(f"Mode: {demo_mode}")
    print("=" * 60)
    
    # Create simulation environment
    env = simpy.Environment()
    
    # Create robot from URDF
    try:
        robot = create_robot_from_urdf(
            env,
            urdf_path,
            "joint_demo_robot"
        )
        
        print(f"✅ Robot created from URDF: {urdf_path}")
        robot.print_robot_info()
        
        movable_joints = [name for name in robot.get_joint_names() 
                         if robot.joints[name].joint_type.value != 'fixed']
        
        if not movable_joints:
            print("❌ No movable joints found!")
            return False
            
        print(f"🎯 Movable joints: {movable_joints}")
        
    except Exception as e:
        print(f"❌ Failed to create robot: {e}")
        return False
    
    # Create URDF robot visualizer
    visualizer = create_urdf_robot_visualizer()
    if not visualizer.available:
        print("❌ PyVista visualization not available")
        return False
    
    # Load robot into visualizer
    if not visualizer.load_robot("demo_robot", robot, urdf_path):
        print("❌ Failed to load robot into visualizer")
        return False
    
    print("✅ Robot loaded into visualizer successfully")
    
    # Configure visualization
    visualizer.plotter.show_scalar_bar = False
    visualizer.plotter.show_axes = True
    
    # Demo motion control function
    def run_joint_motion():
        """Run joint motion based on selected mode"""
        print(f"🎮 Starting {demo_mode} joint motion demo...")
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            if demo_mode == "simple":
                # Simple sinusoidal motion (from simple_joint_demo)
                phase_duration = duration / 3.0
                phase = int(t / phase_duration)
                
                if phase == 0:
                    # Slow motion
                    amplitude = 0.3
                    frequency = 0.5
                    for i, joint_name in enumerate(movable_joints):
                        phase_offset = i * math.pi / 2
                        position = amplitude * math.sin(frequency * 2 * math.pi * t + phase_offset)
                        visualizer.set_joint_command("demo_robot", joint_name, position, max_velocity=1.0)
                        
                elif phase == 1:
                    # Individual joint showcase
                    active_joint = int((t - phase_duration) * 2) % len(movable_joints)
                    for i, joint_name in enumerate(movable_joints):
                        if i == active_joint:
                            position = 0.8 * math.sin(2 * math.pi * t)
                            visualizer.set_joint_command("demo_robot", joint_name, position, max_velocity=2.0)
                        else:
                            visualizer.set_joint_command("demo_robot", joint_name, 0.0, max_velocity=1.0)
                            
                else:
                    # Return to home
                    for joint_name in movable_joints:
                        current_pos = robot.get_joint_state(joint_name).position
                        target_pos = current_pos * 0.9
                        visualizer.set_joint_command("demo_robot", joint_name, target_pos, max_velocity=1.0)
                        
            elif demo_mode == "realtime":
                # Fast coordinated motion (from realtime_joint_demo)
                phase_duration = duration / 4.0
                phase = int(t / phase_duration)
                phase_t = (t % phase_duration) / phase_duration
                
                if phase == 0:
                    # Slow large motions
                    for i, joint_name in enumerate(movable_joints):
                        amplitude = 0.8
                        frequency = 0.2
                        offset = i * math.pi / 2
                        position = amplitude * math.sin(frequency * t * 2 * math.pi + offset)
                        visualizer.set_joint_command("demo_robot", joint_name, position, max_velocity=0.5)
                        
                elif phase == 1:
                    # Individual joint showcase
                    active_joint = int(phase_t * len(movable_joints) * 2) % len(movable_joints)
                    for i, joint_name in enumerate(movable_joints):
                        if i == active_joint:
                            position = 0.9 * math.sin(6 * math.pi * phase_t)
                            visualizer.set_joint_command("demo_robot", joint_name, position, max_velocity=2.0)
                        else:
                            current_pos = robot.get_joint_state(joint_name).position
                            visualizer.set_joint_command("demo_robot", joint_name, current_pos * 0.9, max_velocity=0.5)
                            
                elif phase == 2:
                    # Fast coordinated motion
                    for i, joint_name in enumerate(movable_joints):
                        phase_offset = i * 2 * math.pi / len(movable_joints)
                        amplitude = 0.6
                        frequency = 1.5
                        position = amplitude * math.sin(frequency * t * 2 * math.pi + phase_offset)
                        visualizer.set_joint_command("demo_robot", joint_name, position, max_velocity=3.0)
                        
                else:
                    # Return to home
                    for joint_name in movable_joints:
                        current_pos = robot.get_joint_state(joint_name).position
                        target_pos = current_pos * 0.95
                        visualizer.set_joint_command("demo_robot", joint_name, target_pos, max_velocity=1.0)
                        
            else:  # mixed mode
                # Combined motion patterns
                phase_duration = duration / 4.0
                phase = int(t / phase_duration)
                
                if phase == 0:
                    # Simple slow motion
                    for i, joint_name in enumerate(movable_joints):
                        position = 0.4 * math.sin(0.8 * t + i * math.pi / 3)
                        visualizer.set_joint_command("demo_robot", joint_name, position, max_velocity=1.0)
                        
                elif phase == 1:
                    # Individual joints (simple style)
                    active_joint = int(((t - phase_duration) / phase_duration) * len(movable_joints))
                    active_joint = min(active_joint, len(movable_joints) - 1)
                    
                    for i, joint_name in enumerate(movable_joints):
                        if i == active_joint:
                            position = 0.7 * math.sin(4 * t)
                            visualizer.set_joint_command("demo_robot", joint_name, position, max_velocity=2.0)
                        else:
                            current_pos = robot.get_joint_state(joint_name).position
                            visualizer.set_joint_command("demo_robot", joint_name, current_pos * 0.8, max_velocity=1.0)
                            
                elif phase == 2:
                    # Fast coordinated (realtime style)
                    for i, joint_name in enumerate(movable_joints):
                        phase_offset = i * 2 * math.pi / len(movable_joints)
                        amplitude = 0.6
                        frequency = 1.3
                        position = amplitude * math.sin(frequency * t + phase_offset)
                        visualizer.set_joint_command("demo_robot", joint_name, position, max_velocity=2.5)
                        
                else:
                    # Gradual return to home
                    for joint_name in movable_joints:
                        current_pos = robot.get_joint_state(joint_name).position
                        target_pos = current_pos * 0.92
                        visualizer.set_joint_command("demo_robot", joint_name, target_pos, max_velocity=1.0)
            
            # Update title with joint info
            positions = robot.get_joint_positions()
            joint_info = ", ".join([f"{name[:10]}={pos:.2f}" for name, pos in list(positions.items())[:3]])
            phase_info = f"Phase {int(t / (duration/4)) + 1}/4" if demo_mode != "simple" else f"Phase {int(t / (duration/3)) + 1}/3"
            visualizer.plotter.title = f"Joint Demo ({demo_mode}) - {phase_info} - t={t:.1f}s - {joint_info}"
            
            time.sleep(1/60)  # 60 Hz update rate
        
        # Final return to home position
        print("🏠 Returning all joints to home position...")
        for joint_name in movable_joints:
            visualizer.set_joint_command("demo_robot", joint_name, 0.0, max_velocity=1.0)
        
        time.sleep(2.0)  # Wait for return to home
        visualizer.plotter.title = f"Joint Demo ({demo_mode}) - Completed - All joints at home"
        
        print("✅ Joint motion demo completed")
    
    try:
        if visualizer.display_available:
            import threading
            
            # Start motion in background thread
            motion_thread = threading.Thread(target=run_joint_motion)
            motion_thread.daemon = True
            motion_thread.start()
            
            print("🎮 Interactive joint motion demo started!")
            print("Controls:")
            print("  - Left mouse: Rotate view")
            print("  - Right mouse: Zoom")
            print("  - Middle mouse: Pan")
            print("  - Close window to exit")
            
            # Show interactive window
            visualizer.plotter.show()
            
        else:
            # Headless mode
            print("🖥️ Running in headless mode...")
            run_joint_motion()
            
            # Save screenshot
            os.makedirs("../../output", exist_ok=True)
            screenshot_path = f"../../output/joint_demo_{demo_mode}.png"
            visualizer.plotter.screenshot(screenshot_path)
            print(f"📸 Screenshot saved: {screenshot_path}")
        
        return True
        
    except Exception as e:
        print(f"❌ Demo error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main function with argument parsing"""
    parser = argparse.ArgumentParser(
        description="Unified Joint Motion Demo - URDF based robot visualization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python joint_demo.py 10                                    # 10s mixed mode with movable_robot.urdf  
  python joint_demo.py 15 simple_robot.urdf                  # 15s mixed mode with custom URDF
  python joint_demo.py 20 simple_robot.urdf --mode simple    # 20s simple mode
  python joint_demo.py 30 mobile_robot.urdf --mode realtime  # 30s realtime mode
  python joint_demo.py 15 --headless                         # 15s headless with screenshot

Demo Modes:
  simple   - Gentle individual joint movements (from simple_joint_demo)
  realtime - Fast coordinated motion (from realtime_joint_demo)  
  mixed    - Combination of both styles (default)
        """
    )
    
    parser.add_argument("duration", type=float, default=20.0, nargs='?',
                       help="Demo duration in seconds (default: 20)")
    parser.add_argument("urdf_path", type=str, default="examples/robots/movable_robot.urdf", nargs='?',
                       help="Path to URDF file (default: movable_robot.urdf)")
    parser.add_argument("--mode", choices=["simple", "realtime", "mixed"], default="mixed",
                       help="Demo mode (default: mixed)")
    parser.add_argument("--headless", action="store_true",
                       help="Run in headless mode (no interactive window)")
    
    args = parser.parse_args()
    
    # Check if URDF file exists
    if not os.path.exists(args.urdf_path):
        print(f"❌ URDF file not found: {args.urdf_path}")
        print("Available robots:")
        robots_dir = "examples/robots"
        if os.path.exists(robots_dir):
            for file in os.listdir(robots_dir):
                if file.endswith('.urdf'):
                    print(f"  - {os.path.join(robots_dir, file)}")
        return 1
    
    print("🤖🔧 SimPyROS Unified Joint Motion Demo")
    print("=" * 70)
    
    try:
        # Force headless mode if requested
        if args.headless:
            import os
            os.environ['PYVISTA_OFF_SCREEN'] = 'true'
        
        success = interactive_joint_demo(args.urdf_path, args.duration, args.mode)
        
        if success:
            print("✅ Demo completed successfully!")
            return 0
        else:
            print("❌ Demo failed!")
            return 1
            
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted by user")
        return 0
    except Exception as e:
        print(f"\n❌ Demo failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())