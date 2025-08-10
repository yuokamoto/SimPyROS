#!/usr/bin/env python3
"""
Joint Motion Visualization Demo
関節の動きを明確に視覚化するデモ
"""

import sys
import os
import time
import math
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import simpy
from robot import create_robot_from_urdf
from simulation_object import Pose
from pyvista_visualizer import PyVistaVisualizer, AnimationController
import threading


def joint_motion_demo(duration: float = 15.0):
    """
    関節動作を明確に可視化するデモ
    """
    print("🤖🔄 Joint Motion Visualization Demo")
    print("=" * 50)
    print(f"Duration: {duration} seconds")
    
    # Create simulation environment
    env = simpy.Environment()
    
    # Create robot with movable joints
    try:
        robot = create_robot_from_urdf(
            env, 
            "examples/robots/movable_robot.urdf",
            "joint_demo_robot"
        )
        
        print("✅ Robot with movable joints created")
        robot.print_robot_info()
        
        # Get movable joints
        movable_joints = [name for name in robot.get_joint_names() 
                         if robot.joints[name].joint_type.value != 'fixed']
        
        if not movable_joints:
            print("❌ No movable joints found!")
            return
            
        print(f"🎯 Movable joints: {movable_joints}")
        
    except Exception as e:
        print(f"❌ Failed to create robot: {e}")
        return
    
    # Setup PyVista visualization
    visualizer = PyVistaVisualizer()
    if not visualizer.available:
        print("❌ PyVista not available")
        return
    
    # Configure visualization
    visualizer.plotter.show_scalar_bar = False
    visualizer.plotter.show_axes = True
    
    animation_controller = AnimationController(visualizer.plotter, visualizer.pv)
    
    # Load robot visualization
    print("🔄 Loading robot visualization...")
    from pyvista_visualizer import RobotMeshFactory
    
    mesh_factory = RobotMeshFactory()
    robot_mesh = mesh_factory.create_from_urdf(visualizer.pv, "examples/robots/movable_robot.urdf")
    
    if robot_mesh is None:
        print("❌ Failed to create robot visualization")
        return
    
    # Add robot with enhanced visibility
    animation_controller.add_robot("demo_robot", robot_mesh, color='orange', opacity=0.8)
    print("✅ Robot visualization loaded")
    
    # Add ground and reference objects
    ground = visualizer.pv.Plane(center=(0, 0, -0.1), direction=(0, 0, 1), i_size=3, j_size=3)
    visualizer.plotter.add_mesh(ground, color='lightgray', opacity=0.2, name="ground")
    
    # Add coordinate axes
    axes = visualizer.pv.Axes()
    visualizer.plotter.add_mesh(axes, name="world_axes")
    
    # Joint control process with clear motion patterns
    def joint_control_process():
        """Enhanced joint control with clear motion patterns"""
        print("🎯 Starting enhanced joint motion...")
        
        start_time = env.now
        phase_duration = duration / 3.0  # 3 phases
        
        while env.now < start_time + duration:
            t = env.now - start_time
            phase = int(t / phase_duration)
            phase_t = (t % phase_duration) / phase_duration
            
            if phase == 0:
                # Phase 1: Large amplitude slow motion
                print(f"📍 Phase 1: Large slow motions (t={t:.1f}s)")
                for i, joint_name in enumerate(movable_joints):
                    amplitude = 0.8  # Large motion
                    frequency = 0.3  # Slow
                    offset = i * math.pi / 2  # Phase offset between joints
                    position = amplitude * math.sin(frequency * t * 2 * math.pi + offset)
                    robot.set_joint_position(joint_name, position, max_velocity=0.5)
                    
            elif phase == 1:
                # Phase 2: Individual joint showcase
                active_joint_idx = int(phase_t * len(movable_joints) * 2) % len(movable_joints)
                print(f"📍 Phase 2: Individual joint motion - {movable_joints[active_joint_idx]} (t={t:.1f}s)")
                
                for i, joint_name in enumerate(movable_joints):
                    if i == active_joint_idx:
                        # Active joint: large motion
                        position = 0.9 * math.sin(8 * math.pi * phase_t)
                        robot.set_joint_position(joint_name, position, max_velocity=1.5)
                    else:
                        # Other joints: return to center slowly
                        current_pos = robot.get_joint_state(joint_name).position
                        target_pos = current_pos * 0.95  # Slow decay to zero
                        robot.set_joint_position(joint_name, target_pos, max_velocity=0.3)
                        
            else:
                # Phase 3: Coordinated dance
                print(f"📍 Phase 3: Coordinated motion (t={t:.1f}s)")
                for i, joint_name in enumerate(movable_joints):
                    # Create wave-like coordinated motion
                    phase_offset = i * 2 * math.pi / len(movable_joints)
                    amplitude = 0.6
                    frequency = 1.0
                    position = amplitude * math.sin(frequency * t * 2 * math.pi + phase_offset)
                    robot.set_joint_position(joint_name, position, max_velocity=1.0)
            
            yield env.timeout(0.05)  # 20 Hz control
            
            # Print joint positions every 2 seconds
            if int(t * 5) % 10 == 0:  # Every 2 seconds
                positions = robot.get_joint_positions()
                pos_str = ", ".join([f"{name}={pos:.2f}" for name, pos in positions.items()])
                print(f"  Joint positions: {pos_str}")
        
        # Return to home position
        print("🏠 Returning to home position...")
        home_positions = {name: 0.0 for name in movable_joints}
        robot.set_joint_positions(home_positions, max_velocity=0.8)
        
        yield env.timeout(3.0)
        print("✅ Joint motion demo completed")
    
    # Visualization update process
    def visualization_update_process():
        """Update robot visualization in real-time"""
        while env.now < duration + 4.0:
            # Update robot base pose (could be moving)
            animation_controller.update_robot_pose("demo_robot", robot.pose)
            
            # Update window title with joint info
            positions = robot.get_joint_positions()
            joint_info = ", ".join([f"{name}={pos:.2f}" for name, pos in list(positions.items())[:2]])
            visualizer.plotter.title = f"Joint Motion Demo - t={env.now:.1f}s - {joint_info}"
            
            yield env.timeout(1/30)  # 30 Hz visualization update
    
    # Start simulation processes
    env.process(joint_control_process())
    env.process(visualization_update_process())
    
    print("🎮 Starting joint motion visualization...")
    print("Controls: Left-click+drag=rotate, Right-click+drag=zoom, Middle-click+drag=pan")
    print("Watch the robot joints move in 3 different motion patterns!")
    
    try:
        if visualizer.display_available:
            # Interactive mode with background simulation
            print("Interactive mode: Close the window to end the demo")
            
            def simulation_thread():
                """Run simulation in background thread"""
                env.run(until=duration + 4.0)
            
            # Start simulation thread
            sim_thread = threading.Thread(target=simulation_thread)
            sim_thread.daemon = True
            sim_thread.start()
            
            # Show interactive window
            visualizer.plotter.show()
            
        else:
            # Headless mode
            print("Running in headless mode...")
            env.run(until=duration + 4.0)
            
            # Save screenshot
            os.makedirs("output", exist_ok=True)
            visualizer.plotter.screenshot("output/joint_motion_demo.png")
            print("Screenshot saved: output/joint_motion_demo.png")
        
    except Exception as e:
        print(f"❌ Visualization error: {e}")
        # Fallback: run simulation without visualization
        env.run(until=duration + 4.0)
    
    print("✅ Joint motion demo completed")


def main():
    """Main function with command line argument support"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Joint Motion Visualization Demo")
    parser.add_argument("duration", type=float, default=15.0, nargs='?',
                       help="Demo duration in seconds (default: 15)")
    
    args = parser.parse_args()
    
    print("🤖🔄 SimPyROS Joint Motion Demo")
    print("=" * 60)
    
    try:
        joint_motion_demo(args.duration)
            
    except KeyboardInterrupt:
        print("\n⏹️  Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()