#!/usr/bin/env python3
"""
Robot Class with PyVista Visualization Demo
Shows real-time joint control with 3D visualization
"""

import sys
import os
import time
import math
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import simpy
from robot import Robot, create_simple_arm_robot, create_mobile_robot
from simulation_object import Pose, Velocity
from pyvista_visualizer import PyVistaVisualizer, AnimationController


def interactive_robot_joint_demo(duration: float = 15.0, urdf_path: str = "examples/robots/simple_robot.urdf"):
    """
    Interactive robot joint control with PyVista visualization
    """
    print("🤖🎮 Interactive Robot Joint Control Demo")
    print("=" * 50)
    print(f"Duration: {duration} seconds")
    print(f"URDF: {urdf_path}")
    
    # Create simulation environment
    env = simpy.Environment()
    
    # Create robot
    try:
        if "simple_robot" in urdf_path:
            robot = create_simple_arm_robot(env, "visualized_arm")
        elif "mobile_robot" in urdf_path:  
            robot = create_mobile_robot(env, "visualized_mobile")
        else:
            from robot import create_robot_from_urdf
            robot = create_robot_from_urdf(env, urdf_path, "custom_robot")
        
        robot.print_robot_info()
        
    except Exception as e:
        print(f"❌ Failed to create robot: {e}")
        return
    
    # Setup visualization
    visualizer = PyVistaVisualizer()
    if not visualizer.available:
        print("❌ PyVista not available")
        return
    
    # PyVista表示設定
    visualizer.plotter.show_scalar_bar = False  # 右下のカラーバーを非表示
    visualizer.plotter.show_axes = True         # 軸表示は維持
    
    animation_controller = AnimationController(visualizer.plotter, visualizer.pv)
    
    # Load robot mesh from URDF
    print("🔄 Loading robot visualization...")
    from pyvista_visualizer import RobotMeshFactory
    
    mesh_factory = RobotMeshFactory()
    robot_mesh = mesh_factory.create_from_urdf(visualizer.pv, urdf_path)
    
    if robot_mesh is None:
        print("❌ Failed to create robot visualization")
        return
    
    # Add robot to visualization
    animation_controller.add_robot("robot", robot_mesh, color='orange', opacity=0.9)
    print("✅ Robot visualization loaded")
    
    # Add coordinate frame
    axes = visualizer.pv.Axes()
    visualizer.plotter.add_mesh(axes, name="world_frame")
    
    # Add ground plane
    ground = visualizer.pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), i_size=4, j_size=4)
    visualizer.plotter.add_mesh(ground, color='lightgray', opacity=0.3, name="ground")
    
    # Robot controller process
    def robot_joint_controller():
        """Control robot joints with interesting patterns"""
        print("🎯 Starting robot joint control...")
        
        # Get controllable joints
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        if not joint_names:
            print("❌ No controllable joints found")
            return
            
        print(f"Controlling joints: {joint_names}")
        
        # Control phases
        start_time = env.now
        
        while env.now < start_time + duration:
            t = env.now - start_time
            
            # Phase-based control
            if t < duration * 0.3:
                # Phase 1: Sinusoidal motion
                for i, joint_name in enumerate(joint_names):
                    amplitude = 0.3 + i * 0.1
                    frequency = 0.5 + i * 0.2
                    phase_offset = i * math.pi / 4
                    target_pos = amplitude * math.sin(frequency * t + phase_offset)
                    robot.set_joint_position(joint_name, target_pos, max_velocity=1.0)
                    
            elif t < duration * 0.6:
                # Phase 2: Sequential joint movement
                phase_t = (t - duration * 0.3) / (duration * 0.3)  # 0 to 1
                active_joint_idx = int(phase_t * len(joint_names)) % len(joint_names)
                
                for i, joint_name in enumerate(joint_names):
                    if i == active_joint_idx:
                        # Active joint moves in sine wave
                        target_pos = 0.5 * math.sin(4 * math.pi * phase_t)
                    else:
                        # Other joints slowly return to zero
                        current_pos = robot.get_joint_state(joint_name).position
                        target_pos = current_pos * 0.99  # Decay to zero
                    
                    robot.set_joint_position(joint_name, target_pos, max_velocity=0.8)
                    
            else:
                # Phase 3: Coordinated circular motion
                phase_t = (t - duration * 0.6) / (duration * 0.4)
                
                for i, joint_name in enumerate(joint_names):
                    # Each joint follows a circular pattern with phase offset
                    radius = 0.4
                    phase_offset = i * 2 * math.pi / len(joint_names)
                    circular_angle = 2 * math.pi * phase_t + phase_offset
                    target_pos = radius * math.sin(circular_angle)
                    robot.set_joint_position(joint_name, target_pos, max_velocity=1.5)
            
            yield env.timeout(0.05)  # 20 Hz control rate
        
        # Return to home position
        print("🏠 Returning to home position...")
        home_positions = {name: 0.0 for name in joint_names}
        robot.set_joint_positions(home_positions, max_velocity=1.0)
        
        yield env.timeout(2.0)
        print("✅ Joint control completed")
    
    # Visualization update process
    def visualization_updater():
        """Update visualization based on current robot state"""
        update_rate = 30  # Hz
        dt = 1.0 / update_rate
        
        while env.now < duration + 3.0:  # Extra time for returning home
            # Get current robot state
            current_pose = robot.pose
            joint_positions = robot.get_joint_positions()
            
            # Update robot visualization
            # Note: Individual joint visualization would require enhanced integration
            animation_controller.update_robot_pose("robot", current_pose)
            
            # Update window title with current state
            joint_info = ", ".join([f"{name}={pos:.2f}" for name, pos in list(joint_positions.items())[:3]])
            visualizer.plotter.title = f"Robot Demo - t={env.now:.1f}s - Joints: {joint_info}"
            
            yield env.timeout(dt)
    
    # Start all processes
    env.process(robot_joint_controller())
    env.process(visualization_updater())
    
    # Run simulation
    print("🎮 Starting interactive visualization...")
    print("Controls: Left-click+drag=rotate, Right-click+drag=zoom, Middle-click+drag=pan")
    print("Close the window to end simulation")
    
    try:
        if visualizer.display_available:
            # Interactive mode - simplified without timer events
            print("Interactive mode: Close the window to end the demo")
            
            # Run simulation in background
            import threading
            
            def simulation_thread():
                """Run simulation in background thread"""
                env.run(until=duration + 3.0)
            
            # Start simulation thread
            sim_thread = threading.Thread(target=simulation_thread)
            sim_thread.daemon = True
            sim_thread.start()
            
            # Show interactive window (blocks until closed)
            visualizer.plotter.show()
            
        else:
            # Headless mode
            print("Running in headless mode...")
            env.run(until=duration + 3.0)
            
            # Save final screenshot
            os.makedirs("output", exist_ok=True)
            visualizer.plotter.screenshot("output/robot_joint_demo_final.png")
            print("Screenshot saved: output/robot_joint_demo_final.png")
        
    except Exception as e:
        print(f"❌ Visualization error: {e}")
        # Fallback: run simulation without interactive visualization
        env.run(until=duration + 3.0)
    
    print("✅ Robot visualization demo completed")


def multi_robot_coordination_demo(duration: float = 20.0):
    """
    Multi-robot coordination demo with visualization
    """
    print("🤖🤖 Multi-Robot Coordination Demo")
    print("=" * 50)
    
    env = simpy.Environment()
    
    # Create multiple robots
    robots = []
    try:
        # Robot 1: Simple arm at origin
        robot1 = create_simple_arm_robot(env, "arm1", Pose(x=0, y=0, z=0))
        robots.append(("arm1", robot1, "examples/robots/simple_robot.urdf"))
        
        # Robot 2: Mobile robot
        robot2 = create_mobile_robot(env, "mobile1", Pose(x=2, y=0, z=0))
        robots.append(("mobile1", robot2, "examples/robots/mobile_robot.urdf"))
        
        print(f"✅ Created {len(robots)} robots")
        
    except Exception as e:
        print(f"❌ Failed to create robots: {e}")
        return
    
    # Setup visualization
    visualizer = PyVistaVisualizer()
    if not visualizer.available:
        print("❌ PyVista not available")
        return
    
    # PyVista表示設定
    visualizer.plotter.show_scalar_bar = False  # 右下のカラーバーを非表示
    visualizer.plotter.show_axes = True         # 軸表示は維持
    
    animation_controller = AnimationController(visualizer.plotter, visualizer.pv)
    
    # Load robot meshes
    from pyvista_visualizer import RobotMeshFactory
    mesh_factory = RobotMeshFactory()
    
    colors = ['orange', 'lightblue', 'lightgreen', 'yellow']
    
    for i, (name, robot, urdf_path) in enumerate(robots):
        robot_mesh = mesh_factory.create_from_urdf(visualizer.pv, urdf_path)
        if robot_mesh:
            animation_controller.add_robot(name, robot_mesh, color=colors[i % len(colors)])
            print(f"✅ Loaded visualization for {name}")
    
    # Add environment
    ground = visualizer.pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), i_size=8, j_size=8)
    visualizer.plotter.add_mesh(ground, color='lightgray', opacity=0.2)
    
    # Multi-robot controller
    def multi_robot_controller():
        """Coordinate multiple robots"""
        print("🎭 Starting multi-robot coordination...")
        
        start_time = env.now
        
        while env.now < start_time + duration:
            t = env.now - start_time
            
            # Robot 1: Joint control
            robot1 = robots[0][1]
            joint_names = [name for name in robot1.get_joint_names() 
                          if robot1.joints[name].joint_type.value != 'fixed']
            
            if joint_names:
                for i, joint_name in enumerate(joint_names):
                    target_pos = 0.3 * math.sin(t + i * math.pi / 3)
                    robot1.set_joint_position(joint_name, target_pos)
            
            # Robot 2: Base movement in circle
            if len(robots) > 1:
                robot2 = robots[1][1]
                radius = 1.5
                angular_freq = 0.3
                
                center_x, center_y = 2.0, 0.0
                target_x = center_x + radius * math.cos(angular_freq * t)
                target_y = center_y + radius * math.sin(angular_freq * t)
                target_yaw = angular_freq * t + math.pi/2  # Tangent to circle
                
                target_pose = Pose(x=target_x, y=target_y, z=0.0, yaw=target_yaw)
                robot2.teleport(target_pose)
            
            yield env.timeout(0.05)
        
        print("✅ Multi-robot coordination completed")
    
    # Visualization updater
    def visualization_updater():
        """Update all robot visualizations"""
        while env.now < duration + 1.0:
            for name, robot, _ in robots:
                animation_controller.update_robot_pose(name, robot.pose)
            
            # Update title
            visualizer.plotter.title = f"Multi-Robot Demo - t={env.now:.1f}s"
            
            yield env.timeout(1/30)  # 30 Hz
    
    # Start processes
    env.process(multi_robot_controller())
    env.process(visualization_updater())
    
    # Run simulation
    print("🎮 Starting multi-robot visualization...")
    
    if visualizer.display_available:
        # Interactive mode - simplified without timer events
        print("Interactive multi-robot mode: Close the window to end the demo")
        
        import threading
        
        def simulation_thread():
            """Run simulation in background thread"""
            env.run(until=duration + 1.0)
        
        # Start simulation thread
        sim_thread = threading.Thread(target=simulation_thread)
        sim_thread.daemon = True
        sim_thread.start()
        
        # Show interactive window
        visualizer.plotter.show()
    else:
        env.run(until=duration + 1.0)
        visualizer.plotter.screenshot("output/multi_robot_demo.png")
    
    print("✅ Multi-robot demo completed")


def main():
    """Main function with command line argument support"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Robot Visualization Demo")
    parser.add_argument("duration", type=float, default=15.0, nargs='?',
                       help="Demo duration in seconds (default: 15)")
    parser.add_argument("--urdf", type=str, default="examples/robots/simple_robot.urdf",
                       help="URDF file path (default: examples/robots/simple_robot.urdf)")
    parser.add_argument("--multi", action="store_true",
                       help="Run multi-robot demo instead of single robot")
    
    args = parser.parse_args()
    
    print("🤖 SimPyROS Robot + PyVista Visualization Demo")
    print("=" * 60)
    
    try:
        if args.multi:
            multi_robot_coordination_demo(args.duration)
        else:
            interactive_robot_joint_demo(args.duration, args.urdf)
            
    except KeyboardInterrupt:
        print("\n⏹️  Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()