#!/usr/bin/env python3
"""
Simple Joint Motion Demo - 確実に関節の動きが見えるシンプルなデモ
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
from pyvista_visualizer import PyVistaVisualizer


def simple_joint_demo():
    """シンプルな関節動作デモ - 段階的に関節を動かす"""
    print("🤖🔧 Simple Joint Motion Demo")
    print("=" * 50)
    
    # Create environment and robot
    env = simpy.Environment()
    
    try:
        robot = create_robot_from_urdf(
            env,
            "examples/robots/movable_robot.urdf",
            "simple_demo_robot"
        )
        
        movable_joints = [name for name in robot.get_joint_names() 
                         if robot.joints[name].joint_type.value != 'fixed']
        
        print(f"✅ Robot created with joints: {movable_joints}")
        
        if not movable_joints:
            print("❌ No movable joints found!")
            return
        
    except Exception as e:
        print(f"❌ Failed to create robot: {e}")
        return
    
    # Setup PyVista visualization
    visualizer = PyVistaVisualizer()
    if not visualizer.available:
        print("❌ PyVista not available")
        return
    
    # Configure
    visualizer.plotter.show_scalar_bar = False
    visualizer.plotter.show_axes = True
    
    # Create simple geometric representation of robot
    print("🎨 Creating simple robot visualization...")
    
    # Base (orange box)
    base = visualizer.pv.Cube(x_length=0.3, y_length=0.3, z_length=0.1)
    base_actor = visualizer.plotter.add_mesh(base, color='orange', name='base')
    
    # Arm (blue cylinder) - will rotate with base_to_arm joint
    arm = visualizer.pv.Cylinder(radius=0.05, height=0.35, direction=(0, 0, 1))
    arm_transform = np.eye(4)
    arm_transform[:3, 3] = [0.2, 0, 0.175]  # Position at base joint
    arm.transform(arm_transform)
    arm_actor = visualizer.plotter.add_mesh(arm, color='blue', name='arm')
    
    # End effector (red sphere) - will move with both joints
    end_effector = visualizer.pv.Sphere(radius=0.08)
    end_transform = np.eye(4)
    end_transform[:3, 3] = [0.2, 0, 0.45]  # Initial position
    end_effector.transform(end_transform)
    end_actor = visualizer.plotter.add_mesh(end_effector, color='red', name='end')
    
    # Ground
    ground = visualizer.pv.Plane(center=(0, 0, -0.1), direction=(0, 0, 1), i_size=2, j_size=2)
    visualizer.plotter.add_mesh(ground, color='lightgray', opacity=0.3)
    
    print("✅ Simple robot visualization created")
    
    def update_robot_visual():
        """ロボットの視覚的表現を更新"""
        try:
            # Get current joint positions
            joint_positions = robot.get_joint_positions()
            
            if 'base_to_arm' in joint_positions and 'arm_to_end' in joint_positions:
                base_joint_angle = joint_positions['base_to_arm']
                arm_joint_angle = joint_positions['arm_to_end']
                
                # Update arm position (rotate around Z-axis at base)
                arm_new = visualizer.pv.Cylinder(radius=0.05, height=0.35, direction=(0, 0, 1))
                
                # Apply base joint rotation
                arm_rotation = np.eye(4)
                arm_rotation[:3, :3] = [
                    [math.cos(base_joint_angle), -math.sin(base_joint_angle), 0],
                    [math.sin(base_joint_angle), math.cos(base_joint_angle), 0],
                    [0, 0, 1]
                ]
                arm_rotation[:3, 3] = [0.2, 0, 0.175]
                arm_new.transform(arm_rotation)
                
                # Update end effector position
                end_new = visualizer.pv.Sphere(radius=0.08)
                
                # Position after both joint rotations
                # Base joint effect
                arm_tip_x = 0.2 + 0.35 * math.cos(base_joint_angle)
                arm_tip_y = 0.35 * math.sin(base_joint_angle) 
                arm_tip_z = 0.175
                
                # Arm joint effect (simplified)
                end_offset_x = 0.1 * math.cos(base_joint_angle + arm_joint_angle)
                end_offset_y = 0.1 * math.sin(base_joint_angle + arm_joint_angle)
                end_offset_z = 0.1 * math.sin(arm_joint_angle)
                
                end_transform = np.eye(4)
                end_transform[:3, 3] = [
                    arm_tip_x + end_offset_x,
                    arm_tip_y + end_offset_y, 
                    arm_tip_z + end_offset_z
                ]
                end_new.transform(end_transform)
                
                # Update actors
                if hasattr(arm_actor, 'GetMapper'):
                    arm_actor.GetMapper().SetInputData(arm_new)
                    arm_actor.GetMapper().Modified()
                
                if hasattr(end_actor, 'GetMapper'):
                    end_actor.GetMapper().SetInputData(end_new)
                    end_actor.GetMapper().Modified()
                
                return True
                
        except Exception as e:
            print(f"Error updating visual: {e}")
            return False
        
        return False
    
    # Joint motion process with visual updates
    def joint_motion_with_visual():
        """関節動作 + 視覚更新"""
        print("🎯 Starting joint motion with visual updates...")
        
        # Demo phases
        phases = [
            ("Slow motion", 0.3, 5.0),      # amplitude, duration
            ("Fast motion", 0.6, 4.0),
            ("Individual joints", 0.8, 6.0),
        ]
        
        for phase_name, amplitude, phase_duration in phases:
            print(f"📍 {phase_name} phase (amplitude: {amplitude}, duration: {phase_duration}s)")
            
            phase_start = env.now
            step = 0
            
            while env.now < phase_start + phase_duration:
                t = env.now - phase_start
                
                if phase_name == "Individual joints":
                    # Move joints one by one
                    if t < phase_duration / 2:
                        # Move first joint only
                        pos1 = amplitude * math.sin(2 * math.pi * t)
                        robot.set_joint_position(movable_joints[0], pos1, max_velocity=1.0)
                        robot.set_joint_position(movable_joints[1], 0.0, max_velocity=1.0)
                    else:
                        # Move second joint only
                        pos2 = amplitude * math.sin(2 * math.pi * (t - phase_duration/2))
                        robot.set_joint_position(movable_joints[0], 0.0, max_velocity=1.0)
                        robot.set_joint_position(movable_joints[1], pos2, max_velocity=1.0)
                else:
                    # Move both joints
                    frequency = 0.5 if "Slow" in phase_name else 1.0
                    for i, joint_name in enumerate(movable_joints):
                        phase_offset = i * math.pi / 2
                        position = amplitude * math.sin(frequency * 2 * math.pi * t + phase_offset)
                        robot.set_joint_position(joint_name, position, max_velocity=2.0)
                
                # Update visual representation
                update_robot_visual()
                
                # Update title
                positions = robot.get_joint_positions()
                joint_info = ", ".join([f"{name}={pos:.2f}" for name, pos in positions.items()])
                visualizer.plotter.title = f"{phase_name} - t={t:.1f}s - {joint_info}"
                
                yield env.timeout(0.1)  # 10 Hz
                step += 1
                
                # Print positions every second
                if step % 10 == 0:
                    pos_str = ", ".join([f"{name}={pos:.3f}" for name, pos in positions.items()])
                    print(f"  t={t:.1f}s: {pos_str}")
        
        # Return to home
        print("🏠 Returning to home position...")
        for joint_name in movable_joints:
            robot.set_joint_position(joint_name, 0.0, max_velocity=1.0)
        
        yield env.timeout(2.0)
        update_robot_visual()
        visualizer.plotter.title = "Demo completed - All joints at home position"
        
        print("✅ Joint motion demo completed")
    
    # Start motion
    env.process(joint_motion_with_visual())
    
    print("🎮 Starting simple joint motion demo...")
    print("Watch the blue cylinder (arm) and red sphere (end effector) move!")
    print("Close the window to end the demo")
    
    try:
        if visualizer.display_available:
            # Run simulation in background while showing visualization
            import threading
            
            def sim_thread():
                env.run(until=20.0)  # Total demo time
            
            thread = threading.Thread(target=sim_thread)
            thread.daemon = True
            thread.start()
            
            # Show visualization
            visualizer.plotter.show()
            
        else:
            # Headless mode
            env.run(until=20.0)
            os.makedirs("output", exist_ok=True)
            visualizer.plotter.screenshot("output/simple_joint_demo.png")
            print("Screenshot saved: output/simple_joint_demo.png")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        env.run(until=20.0)  # Fallback
    
    print("✅ Simple joint demo completed")


if __name__ == "__main__":
    simple_joint_demo()