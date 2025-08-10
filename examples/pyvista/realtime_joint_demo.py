#!/usr/bin/env python3
"""
Real-time Joint Motion Visualization Demo
リアルタイムで関節の動きが見える特別なデモ
"""

import sys
import os
import time
import math
import numpy as np
import threading

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import simpy
from robot import create_robot_from_urdf
from simulation_object import Pose
from pyvista_visualizer import PyVistaVisualizer
from urdf_loader import URDFLoader


class RealTimeJointVisualizer:
    """リアルタイム関節動作可視化システム"""
    
    def __init__(self, pv_module, plotter):
        self.pv = pv_module
        self.plotter = plotter
        self.link_actors = {}
        self.urdf_loader = None
        self.robot = None
        
    def setup_robot_visualization(self, robot, urdf_path):
        """ロボット視覚化のセットアップ"""
        self.robot = robot
        
        # URDF loader setup
        self.urdf_loader = URDFLoader()
        if not self.urdf_loader.load_urdf(urdf_path):
            print("❌ Failed to load URDF for visualization")
            return False
        
        # Create individual link meshes
        self._create_individual_link_meshes()
        return True
    
    def _create_individual_link_meshes(self):
        """個別リンクメッシュの作成"""
        print("🎨 Creating individual link meshes...")
        
        for link_name, link_info in self.urdf_loader.links.items():
            # Create mesh based on geometry type
            mesh = None
            
            if link_info.geometry_type == "box":
                size = link_info.geometry_params.get('size', [0.3, 0.3, 0.1])
                mesh = self.pv.Cube(x_length=size[0], y_length=size[1], z_length=size[2])
                
            elif link_info.geometry_type == "cylinder":
                radius = link_info.geometry_params.get('radius', 0.05)
                length = link_info.geometry_params.get('length', 0.35)
                mesh = self.pv.Cylinder(radius=radius, height=length, direction=(0, 0, 1))
                
            elif link_info.geometry_type == "sphere":
                radius = link_info.geometry_params.get('radius', 0.08)
                mesh = self.pv.Sphere(radius=radius)
            
            if mesh is not None:
                # Add to scene with link color
                color = link_info.color[:3]  # RGB only
                actor = self.plotter.add_mesh(
                    mesh, 
                    color=color, 
                    opacity=0.8, 
                    name=f"link_{link_name}"
                )
                self.link_actors[link_name] = {
                    'actor': actor,
                    'mesh': mesh,
                    'color': color
                }
                print(f"  Added {link_name}: {link_info.geometry_type} with color {color}")
    
    def update_link_poses(self):
        """関節位置に基づいてリンクの位置を更新"""
        if not self.robot or not self.link_actors:
            return
        
        try:
            # Get current link poses from robot
            link_poses = self.robot.get_link_poses()
            
            for link_name, pose in link_poses.items():
                if link_name in self.link_actors and pose is not None:
                    # Create transformation matrix
                    transform_matrix = np.eye(4)
                    transform_matrix[:3, :3] = pose.rotation.as_matrix()
                    transform_matrix[:3, 3] = pose.position
                    
                    # Update mesh position
                    original_mesh = self.link_actors[link_name]['mesh'].copy()
                    original_mesh.transform(transform_matrix)
                    
                    # Update the actor
                    actor = self.link_actors[link_name]['actor']
                    if hasattr(actor, 'GetMapper'):
                        mapper = actor.GetMapper()
                        mapper.SetInputData(original_mesh)
                        mapper.Modified()
                    
        except Exception as e:
            print(f"⚠️ Error updating link poses: {e}")


def realtime_joint_demo(duration: float = 20.0):
    """リアルタイム関節動作デモ"""
    print("🤖⚡ Real-time Joint Motion Demo")
    print("=" * 50)
    print(f"Duration: {duration} seconds")
    
    # Create simulation environment
    env = simpy.Environment()
    
    # Create robot
    try:
        robot = create_robot_from_urdf(
            env,
            "examples/robots/movable_robot.urdf",
            "realtime_robot"
        )
        
        movable_joints = [name for name in robot.get_joint_names() 
                         if robot.joints[name].joint_type.value != 'fixed']
        
        if not movable_joints:
            print("❌ No movable joints found!")
            return
        
        print(f"✅ Robot created with movable joints: {movable_joints}")
        
    except Exception as e:
        print(f"❌ Failed to create robot: {e}")
        return
    
    # Setup PyVista
    visualizer = PyVistaVisualizer()
    if not visualizer.available:
        print("❌ PyVista not available")
        return
    
    # Configure visualization
    visualizer.plotter.show_scalar_bar = False
    visualizer.plotter.show_axes = True
    visualizer.plotter.title = "Real-time Joint Motion Demo"
    
    # Setup real-time visualizer
    rt_visualizer = RealTimeJointVisualizer(visualizer.pv, visualizer.plotter)
    if not rt_visualizer.setup_robot_visualization(robot, "examples/robots/movable_robot.urdf"):
        print("❌ Failed to setup robot visualization")
        return
    
    # Add ground
    ground = visualizer.pv.Plane(center=(0, 0, -0.1), direction=(0, 0, 1), i_size=2, j_size=2)
    visualizer.plotter.add_mesh(ground, color='lightgray', opacity=0.2)
    
    print("✅ Real-time visualization setup complete")
    
    # Enhanced joint control with visual feedback
    def joint_control_process():
        """視覚フィードバック付き関節制御"""
        print("🎯 Starting real-time joint motion...")
        
        phase_duration = duration / 4.0  # 4 phases
        start_time = env.now
        
        while env.now < start_time + duration:
            t = env.now - start_time
            phase = int(t / phase_duration)
            phase_t = (t % phase_duration) / phase_duration
            
            if phase == 0:
                # Phase 1: Slow large motions
                for i, joint_name in enumerate(movable_joints):
                    amplitude = 0.8
                    frequency = 0.2
                    offset = i * math.pi / 2
                    position = amplitude * math.sin(frequency * t * 2 * math.pi + offset)
                    robot.set_joint_position(joint_name, position, max_velocity=0.5)
                    
            elif phase == 1:
                # Phase 2: Individual joint showcase
                active_joint = int(phase_t * len(movable_joints) * 2) % len(movable_joints)
                for i, joint_name in enumerate(movable_joints):
                    if i == active_joint:
                        position = 0.9 * math.sin(6 * math.pi * phase_t)
                        robot.set_joint_position(joint_name, position, max_velocity=2.0)
                    else:
                        current_pos = robot.get_joint_state(joint_name).position
                        robot.set_joint_position(joint_name, current_pos * 0.9, max_velocity=0.5)
                        
            elif phase == 2:
                # Phase 3: Fast coordinated motion
                for i, joint_name in enumerate(movable_joints):
                    phase_offset = i * 2 * math.pi / len(movable_joints)
                    amplitude = 0.6
                    frequency = 1.5
                    position = amplitude * math.sin(frequency * t * 2 * math.pi + phase_offset)
                    robot.set_joint_position(joint_name, position, max_velocity=3.0)
                    
            else:
                # Phase 4: Return to home
                for joint_name in movable_joints:
                    current_pos = robot.get_joint_state(joint_name).position
                    target_pos = current_pos * 0.95  # Gradually return to zero
                    robot.set_joint_position(joint_name, target_pos, max_velocity=1.0)
            
            yield env.timeout(0.02)  # 50 Hz control
        
        print("✅ Joint motion completed")
    
    # Visual update process
    def visual_update_process():
        """視覚更新プロセス"""
        while env.now < duration + 2.0:
            # Update link poses based on current joint positions
            rt_visualizer.update_link_poses()
            
            # Update title with joint info
            positions = robot.get_joint_positions()
            joint_info = ", ".join([f"{name}={pos:.2f}" for name, pos in list(positions.items())[:2]])
            visualizer.plotter.title = f"Real-time Joint Motion - t={env.now:.1f}s - {joint_info}"
            
            yield env.timeout(1/60)  # 60 Hz visual update
    
    # Start processes
    env.process(joint_control_process())
    env.process(visual_update_process())
    
    print("🎮 Starting real-time joint visualization...")
    print("Controls: Left-click+drag=rotate, Right-click+drag=zoom, Middle-click+drag=pan")
    print("Watch individual robot links move in real-time!")
    
    try:
        if visualizer.display_available:
            def simulation_thread():
                """Background simulation thread"""
                env.run(until=duration + 2.0)
            
            # Start simulation
            sim_thread = threading.Thread(target=simulation_thread)
            sim_thread.daemon = True  
            sim_thread.start()
            
            # Show interactive window
            visualizer.plotter.show()
            
        else:
            # Headless mode
            print("Running in headless mode...")
            env.run(until=duration + 2.0)
            
            os.makedirs("output", exist_ok=True)
            visualizer.plotter.screenshot("output/realtime_joint_demo.png")
            print("Screenshot saved: output/realtime_joint_demo.png")
        
    except Exception as e:
        print(f"❌ Visualization error: {e}")
        # Fallback
        env.run(until=duration + 2.0)
    
    print("✅ Real-time joint demo completed")


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Real-time Joint Motion Demo")
    parser.add_argument("duration", type=float, default=20.0, nargs='?',
                       help="Demo duration in seconds (default: 20)")
    
    args = parser.parse_args()
    
    print("🤖⚡ SimPyROS Real-time Joint Motion Demo")
    print("=" * 60)
    
    try:
        realtime_joint_demo(args.duration)
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()