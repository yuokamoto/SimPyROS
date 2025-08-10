#!/usr/bin/env python3
"""
URDF Robot Demo for SimPyROS
Demonstrates loading and visualizing robots from URDF files

Usage:
    python examples/pyvista/urdf_robot_demo.py [duration] [urdf_path] [options]
    
Options:
    --headless           Force headless mode (no GUI window)  
    --screenshots        Save screenshots to output/ directory
    
Examples:
    python examples/pyvista/urdf_robot_demo.py 5
    python examples/pyvista/urdf_robot_demo.py 10 my_robot.urdf
    python examples/pyvista/urdf_robot_demo.py 10 --headless
    python examples/pyvista/urdf_robot_demo.py 5 --headless --screenshots
    python examples/pyvista/urdf_robot_demo.py 15 my_robot.urdf --screenshots
"""

import sys
import os
import math
import time
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

import simpy
from simulation_object import SimulationObject, ObjectType, Pose, Velocity, ObjectParameters
from pyvista_visualizer import (
    PyVistaVisualizer, create_interactive_visualizer, setup_basic_scene,
    create_robot_mesh, AnimationController
)
try:
    from legacy.loaders.urdf_loader import create_simple_robot_urdf
except ImportError:
    # Fallback: create a simple test function
    def create_simple_robot_urdf(filepath):
        pass  # Function not needed for main functionality

def print_urdf_info(urdf_path):
    """Print detailed URDF information for debugging"""
    if not urdf_path or not os.path.exists(urdf_path):
        print(f"❌ URDF file not found: {urdf_path}")
        return False
    
    try:
        from urdf_loader import URDFLoader
        loader = URDFLoader()
        
        print(f"\n📄 URDF File Analysis: {urdf_path}")
        print("=" * 50)
        
        if not loader.is_available():
            print("❌ URDF loader dependencies not available")
            print("   Install with: pip install urdfpy trimesh pycollada")
            return False
        
        if loader.load_urdf(urdf_path):
            print("✅ URDF successfully loaded!")
            loader.print_info()
            
            # Test mesh creation
            try:
                import pyvista as pv
                mesh = loader.get_combined_mesh(pv)
                if mesh:
                    print(f"✅ 3D mesh successfully created (vertices: {mesh.n_points}, faces: {mesh.n_faces})")
                else:
                    print("⚠️  No mesh data available (using geometric primitives)")
                return True
            except Exception as e:
                print(f"⚠️  Mesh creation failed: {e}")
                return False
        else:
            print("❌ URDF loading failed")
            return False
            
    except ImportError as e:
        print(f"❌ Import error: {e}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False


def create_simulation_environment():
    """Create the SimPy simulation environment and objects"""
    env = simpy.Environment()
    
    # Create robot object
    robot_params = ObjectParameters(
        name="urdf_robot",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0, y=0, z=0.1),  # Slightly above ground
        update_interval=0.1
    )
    robot = SimulationObject(env, robot_params)
    
    # Create a target for robot to move towards
    target_params = ObjectParameters(
        name="target",
        object_type=ObjectType.STATIC,
        initial_pose=Pose(x=3, y=2, z=0.5)
    )
    target = SimulationObject(env, target_params)
    
    return env, robot, target


def robot_controller(env, robot, target, visualizer, animation_controller):
    """Control the robot movement in a figure-8 pattern"""
    print("Starting robot controller...")
    
    dt = 0.1  # Time step
    speed = 1.0
    angular_speed = 0.5
    
    iteration = 0
    while True:
        iteration += 1
        if iteration % 50 == 0:  # Every 5 seconds
            print(f"Robot controller iteration {iteration}, time: {env.now:.1f}")
        # Calculate figure-8 trajectory
        t = env.now
        
        # Figure-8 parametric equations
        target_x = 2 * math.sin(t * 0.3)
        target_y = 1.5 * math.sin(t * 0.6)  # Different frequency for figure-8
        target_z = 0.1 + 0.2 * math.sin(t * 0.2)  # Slight vertical movement
        
        # Calculate direction to target
        current_pos = robot.pose.position
        target_pos = np.array([target_x, target_y, target_z])
        
        direction = target_pos - current_pos
        distance = np.linalg.norm(direction)
        
        if distance > 0.01:  # Avoid division by zero
            direction = direction / distance
            
            # Convert world direction to robot local coordinates
            local_direction = robot.pose.rotation.inv().apply(direction)
            
            # Set velocity
            velocity = Velocity(
                linear_x=local_direction[0] * speed,
                linear_y=local_direction[1] * speed,
                linear_z=local_direction[2] * speed * 0.5,  # Slower vertical movement
                angular_z=math.sin(t * 0.4) * angular_speed  # Rotation
            )
            robot.set_velocity(velocity)
            
            # Debug: Print velocity occasionally
            if int(t * 10) % 100 == 0:  # Every 10 seconds
                print(f"Time: {t:.1f}, Robot pos: {current_pos}, Velocity: ({velocity.linear_x:.2f}, {velocity.linear_y:.2f}, {velocity.linear_z:.2f})")
        
        # Update visualization
        if visualizer.available:
            animation_controller.update_robot_pose("robot", robot.pose)
            
            # Add trajectory trail periodically
            if int(t * 10) % 50 == 0:  # Every 5 seconds
                animation_controller.add_trajectory_trail("robot", color='yellow', line_width=2)
        
        yield env.timeout(dt)


def setup_visualization(robot, target, urdf_path=None):
    """Setup PyVista visualization"""
    print("Setting up PyVista visualization...")
    
    # Create visualizer
    visualizer = create_interactive_visualizer()
    
    if not visualizer.available:
        print("PyVista not available, running simulation without visualization")
        return visualizer, None
    
    # Setup basic scene
    setup_basic_scene(visualizer)
    
    # Create animation controller
    animation_controller = AnimationController(visualizer.plotter, visualizer.pv)
    
    # Create robot mesh
    robot_mesh = None
    if urdf_path and os.path.exists(urdf_path):
        print(f"🔄 Loading robot from URDF: {urdf_path}")
        robot_mesh = create_robot_mesh(visualizer, robot_type='urdf', urdf_path=urdf_path)
        if robot_mesh:
            print(f"✅ Successfully created robot mesh from URDF")
        else:
            print(f"❌ Failed to create robot mesh from URDF")
        
    # Fallback to built-in mesh if URDF loading fails
    if robot_mesh is None:
        print("🔄 URDF loading failed or not available, using built-in wheeled robot mesh")
        robot_mesh = create_robot_mesh(visualizer, robot_type='wheeled')
        if robot_mesh:
            print("✅ Built-in wheeled robot mesh created successfully")
    
    if robot_mesh:
        # Add robot to animation controller
        animation_controller.add_robot("robot", robot_mesh, color='orange', opacity=0.8)
        print("Robot mesh successfully added to visualization")
    else:
        print("Warning: Could not create any robot mesh")
    
    # Add target as a simple sphere
    target_mesh = visualizer.pv.Sphere(center=target.pose.position, radius=0.15)
    visualizer.plotter.add_mesh(target_mesh, color='red', opacity=0.7)
    
    # Add detailed status information
    urdf_status = "❌ Not loaded"
    robot_info = "Built-in wheeled robot (fallback)"
    
    if urdf_path and os.path.exists(urdf_path):
        # Try simple URDF loader first
        try:
            from simple_urdf_loader import SimpleURDFLoader
            simple_loader = SimpleURDFLoader()
            if simple_loader.load_urdf(urdf_path):
                urdf_status = "✅ Successfully loaded (Simple Parser)"
                robot_info = f"URDF: {os.path.basename(urdf_path)}"
                robot_info += f" ({len(simple_loader.links)} links, {len(simple_loader.joints)} joints)"
            else:
                urdf_status = "⚠️ Simple loader failed"
                robot_info = "Trying advanced loader..."
        except Exception as e:
            urdf_status = "❌ Simple loader error"
            robot_info = f"Simple loader error: {e}"
        
        # Fallback to advanced URDF loader if simple failed
        if "failed" in urdf_status or "error" in urdf_status:
            try:
                from urdf_loader import URDFLoader
                loader = URDFLoader()
                if loader.is_available():
                    if loader.load_urdf(urdf_path):
                        urdf_status = "✅ Successfully loaded (Advanced Parser)"
                        robot_info = f"URDF: {os.path.basename(urdf_path)}"
                        robot_info += f" ({len(loader.links)} links, {len(loader.joints)} joints)"
                    else:
                        urdf_status = "⚠️ Advanced load failed"
                        robot_info = "Built-in robot (URDF parsing error)"
                else:
                    urdf_status = "⚠️ Dependencies missing"
                    robot_info = "Built-in robot (urdfpy/trimesh not available)"
            except Exception as e:
                urdf_status = "❌ All loaders failed"
                robot_info = f"Built-in robot (loader error: {e})"
    
    info_text = (
        "SimPyROS URDF Robot Demo\n"
        "Controls: Left-drag=rotate, Right-drag=zoom, Middle-drag=pan\n"
        f"URDF Status: {urdf_status}\n"
        f"Robot: {robot_info}"
    )
    visualizer.plotter.add_text(info_text, position='upper_left', font_size=10, name='main_info')
    
    return visualizer, animation_controller


def run_demo(duration=10.0, urdf_path=None, force_headless=False, save_screenshots=False):
    """Run the complete demo"""
    print(f"Starting URDF Robot Demo (duration: {duration}s)")
    
    # Debug: Show what URDF path we received
    if urdf_path:
        print(f"URDF path provided: {urdf_path}")
        print(f"URDF file exists: {os.path.exists(urdf_path)}")
        if os.path.exists(urdf_path):
            print(f"URDF absolute path: {os.path.abspath(urdf_path)}")
    
    # Create test URDF if none provided
    if not urdf_path:
        print("No URDF path provided - creating test URDF file...")
        urdf_path = "test_robot.urdf"
        create_simple_robot_urdf(urdf_path)
        print(f"Test URDF created: {urdf_path}")
    
    # Print detailed URDF information
    print_urdf_info(urdf_path)
    
    # Create simulation
    env, robot, target = create_simulation_environment()
    
    # Setup visualization
    visualizer, animation_controller = setup_visualization(robot, target, urdf_path)
    
    # Start robot controller process
    env.process(robot_controller(env, robot, target, visualizer, animation_controller))
    
    # Run simulation
    if visualizer.available:
        print("Starting interactive simulation...")
        print("Close the visualization window to end the simulation")
        
        # Determine mode: force_headless takes priority
        use_interactive = (not force_headless) and visualizer.display_available
        
        if use_interactive:
            try:
                print("Opening interactive 3D window...")
                print("Controls: Left-click+drag=rotate, Right-click+drag=zoom, Middle-click+drag=pan")
                print("Close the window to end simulation")
                
                # Set up manual animation loop for interactive window
                start_time = time.time()
                frame_count = 0
                
                def animation_loop():
                    """Manual animation loop"""
                    nonlocal frame_count, start_time
                    current_time = time.time() - start_time
                    
                    if current_time >= duration:
                        visualizer.plotter.close()
                        return
                    
                    # Run simulation step
                    env.run(until=env.now + 0.1)
                    
                    # Update robot visualization
                    animation_controller.update_robot_pose("robot", robot.pose)
                    
                    # Add trajectory trail periodically
                    if frame_count % 20 == 0:  # Every 2 seconds
                        animation_controller.add_trajectory_trail("robot", color='yellow', line_width=2)
                    
                    # Update info display
                    info_text = (
                        f"URDF Demo - Frame: {frame_count}\n"
                        f"Time: {current_time:.1f}/{duration:.1f}s\n"
                        f"Robot: {'URDF loaded' if urdf_path and os.path.exists(urdf_path) else 'Built-in fallback'}\n"
                        f"Pos: ({robot.pose.x:.1f}, {robot.pose.y:.1f}, {robot.pose.z:.1f})"
                    )
                    visualizer.plotter.add_text(info_text, position='upper_left', font_size=10, name='info')
                    
                    frame_count += 1
                
                # Show window and run animation
                visualizer.plotter.show(auto_close=False, interactive_update=True)
                
                # Run animation loop
                while time.time() - start_time < duration:
                    animation_loop()
                    visualizer.plotter.update()
                    time.sleep(0.1)  # 10 FPS
                    
                    # Check if window was closed
                    if not visualizer.plotter.render_window:
                        break
                
                print(f"Interactive simulation completed after {duration} seconds")
                
            except Exception as e:
                print(f"Interactive mode failed: {e}")
                print("Falling back to headless mode...")
                # Fall through to headless mode
        
        # Headless mode (forced, fallback, or when no display available)
        if not use_interactive:
            try:
                # Run simulation for specified duration
                if save_screenshots:
                    print("Running simulation in headless mode and capturing screenshots...")
                else:
                    print("Running simulation in headless mode (no screenshots)...")
                
                # Setup screenshots if requested
                if save_screenshots:
                    # Ensure output directory exists
                    os.makedirs("output", exist_ok=True)
                    
                    # Capture initial frame
                    try:
                        visualizer.plotter.screenshot(filename="output/urdf_demo_start.png")
                        print("Initial screenshot saved to output/urdf_demo_start.png")
                    except Exception as e:
                        print(f"Screenshot failed: {e}")
                        # Fallback: setup off_screen mode properly
                        visualizer.plotter.show(auto_close=False, interactive=False, window_size=(800, 600))
                        visualizer.plotter.screenshot(filename="output/urdf_demo_start.png")
                        print("Initial screenshot saved to output/urdf_demo_start.png")
                
                # Run simulation step by step
                dt = 0.1
                steps = int(duration / dt)
                
                for step in range(steps):
                    # Run simulation step
                    env.run(until=env.now + dt)
                    
                    # Update visualization every few steps
                    if step % 5 == 0:  # Every 0.5 seconds
                        animation_controller.update_robot_pose("robot", robot.pose)
                        
                        # Save screenshots if requested
                        if save_screenshots and step % 10 == 0:  # Every 1 second
                            screenshot_name = f"output/urdf_demo_step_{step:03d}.png"
                            try:
                                visualizer.plotter.screenshot(filename=screenshot_name)
                                print(f"Screenshot saved: {screenshot_name}")
                            except Exception:
                                pass  # Silently skip failed screenshots during animation
                
                # Capture final frame if requested
                if save_screenshots:
                    try:
                        visualizer.plotter.screenshot(filename="output/urdf_demo_end.png")
                        print("Final screenshot saved to output/urdf_demo_end.png")
                    except Exception:
                        print("Final screenshot failed (headless mode issue)")
                
                print(f"Headless simulation completed after {duration} seconds")
                
            except Exception as e:
                print(f"Headless simulation failed: {e}")
                return 1
    else:
        print("PyVista not available - running text-only simulation...")
        env.run(until=duration)
    
    print("Demo completed")


def main():
    """Main function with argument parsing"""
    duration = 10.0  # Default duration
    urdf_path = None
    force_headless = False
    save_screenshots = False
    
    # Parse command line arguments
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        arg = args[i]
        
        if arg == '--headless':
            force_headless = True
        elif arg == '--screenshots' or arg == '--save-screenshots':
            save_screenshots = True
        elif arg.startswith('--'):
            print(f"Unknown option: {arg}")
        else:
            # Positional arguments: duration, urdf_path
            if duration == 10.0:  # First positional arg is duration
                try:
                    duration = float(arg)
                except ValueError:
                    print(f"Invalid duration '{arg}', using default (10 seconds)")
                    duration = 10.0
            elif urdf_path is None:  # Second positional arg is URDF path
                urdf_path = arg
                # Try to resolve the path relative to current directory and script directory
                if not os.path.exists(urdf_path):
                    # Try relative to script directory
                    script_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                    alternative_path = os.path.join(script_dir, urdf_path)
                    if os.path.exists(alternative_path):
                        urdf_path = alternative_path
                        print(f"Found URDF at: {urdf_path}")
                    else:
                        print(f"URDF file not found at:")
                        print(f"  - {arg}")
                        print(f"  - {alternative_path}")
                        print(f"Will use fallback test robot.")
                        urdf_path = None
        i += 1
    
    print("=" * 60)
    print("SimPyROS URDF Robot Demonstration")
    print("=" * 60)
    
    # Display usage information
    print(f"Duration: {duration} seconds")
    if urdf_path:
        print(f"URDF file: {urdf_path}")
    else:
        print("URDF file: Will create test URDF")
    
    # Display mode information
    if force_headless:
        print("Mode: Forced headless")
    else:
        print("Mode: Auto-detect (interactive if DISPLAY available)")
    
    if save_screenshots:
        print("Screenshots: Enabled (saved to output/)")
    else:
        print("Screenshots: Disabled")
    
    print("\nUsage:")
    print("  python urdf_robot_demo.py [duration] [urdf_path] [options]")
    print("  Options:")
    print("    --headless           Force headless mode (no GUI window)")
    print("    --screenshots        Save screenshots to output/ directory")
    print("  Examples:")
    print("    python urdf_robot_demo.py 10")
    print("    python urdf_robot_demo.py 15 my_robot.urdf")
    print("    python urdf_robot_demo.py 10 --headless")
    print("    python urdf_robot_demo.py 10 --headless --screenshots")
    print("    python urdf_robot_demo.py 5 my_robot.urdf --screenshots")
    print()
    
    try:
        run_demo(duration, urdf_path, force_headless, save_screenshots)
    except Exception as e:
        print(f"Demo failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())