import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import simpy
import math
import threading
import time
import signal
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from visualizer import Object3DVisualizer


def run_simulation_with_visualization(real_time_factor=1.0):
    """Run simulation with 3D visualization
    
    Args:
        real_time_factor: Speed multiplier for simulation time
                         1.0 = real-time, 0.5 = half speed, 2.0 = double speed
    """
    
    # Global flag for clean shutdown
    shutdown_requested = False
    
    def signal_handler(signum, frame):
        nonlocal shutdown_requested
        print("\nShutdown requested (Ctrl+C). Stopping simulation...")
        shutdown_requested = True
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create simulation environment
    env = simpy.Environment()
    
    # Create visualizer
    viz = Object3DVisualizer(figure_size=(14, 10), grid_size=8.0)
    
    # Create simulation objects
    # Robot (moving)
    robot = SimulationObject(env, ObjectParameters(
        name="robot",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=0.0, z=0.0),
        update_interval=0.05
    ))
    
    # Sensor (attached to robot)
    sensor = SimulationObject(env, ObjectParameters(
        name="sensor",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=1.5, y=0.0, z=0.8),
        update_interval=0.05
    ))
    
    # Target (moving independently)
    target = SimulationObject(env, ObjectParameters(
        name="target",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=3.0, y=3.0, z=1.0),
        update_interval=0.05
    ))
    
    # Static obstacles
    obstacle1 = SimulationObject(env, ObjectParameters(
        name="obstacle1",
        object_type=ObjectType.STATIC,
        initial_pose=Pose(x=2.0, y=1.0, z=0.0)
    ))
    
    obstacle2 = SimulationObject(env, ObjectParameters(
        name="obstacle2", 
        object_type=ObjectType.STATIC,
        initial_pose=Pose(x=-1.0, y=2.0, z=0.5)
    ))
    
    # Connect sensor to robot
    robot.connect_to(sensor, Pose(x=1.5, y=0.0, z=0.8))
    
    # Add objects to visualizer
    viz.add_object(robot, color='blue', marker='o', size=150, show_trajectory=True)
    viz.add_object(sensor, color='green', marker='^', size=100, show_trajectory=True)  
    viz.add_object(target, color='red', marker='s', size=120, show_trajectory=True)
    viz.add_object(obstacle1, color='gray', marker='x', size=200, show_trajectory=False)
    viz.add_object(obstacle2, color='brown', marker='x', size=200, show_trajectory=False)
    
    # Start visualization
    viz.start_animation()
    
    # Timing variables for real-time synchronization
    sim_start_time = None
    
    def run_simulation_phase(end_time, phase_setup=None, dynamic_check=None):
        """Run simulation phase with real-time synchronization
        
        Args:
            end_time: Simulation time to run until
            phase_setup: Optional function to call before starting the phase
            dynamic_check: Optional function that returns True to continue, False to stop early
        """
        nonlocal shutdown_requested, sim_start_time
        
        if phase_setup:
            phase_setup()
        
        dt = 0.05  # Simulation time step (20 FPS)
        
        while env.now < end_time and not shutdown_requested:
            env.run(until=env.now + dt)
            
            # Check dynamic condition (for early termination)
            if dynamic_check and not dynamic_check():
                break
            
            # Real-time synchronization
            elapsed_real_time = time.time() - sim_start_time
            expected_sim_time = elapsed_real_time * real_time_factor
            
            if env.now > expected_sim_time:
                sleep_time = (env.now - expected_sim_time) / real_time_factor
                if sleep_time > 0:
                    time.sleep(sleep_time)
    
    def simulation_thread():
        """Run simulation in separate thread with real-time control"""
        nonlocal shutdown_requested, sim_start_time
        
        # Define simulation phases
        phases = [
            {
                'name': 'Phase 1: Robot circular motion',
                'end_time': 8.0,
                'setup': lambda: robot.set_velocity(Velocity(linear_x=1.0, angular_z=0.8)),
                'cleanup': lambda: robot.stop()
            },
            {
                'name': 'Phase 2: Target figure-8 motion', 
                'end_time': 15.0,
                'setup': lambda: target.set_velocity(Velocity(linear_x=0.8, linear_y=0.5, angular_z=0.6)),
                'cleanup': None
            },
            {
                'name': 'Phase 3: Complex motion',
                'end_time': 25.0, 
                'setup': lambda: (
                    robot.set_velocity(Velocity(linear_x=0.5, linear_y=0.3, angular_z=-0.4)),
                    target.set_velocity(Velocity(linear_x=-0.6, linear_y=0.4, angular_z=0.9))
                ),
                'cleanup': None
            },
            {
                'name': 'Phase 4: Robot approaches target',
                'end_time': 30.0,
                'setup': lambda: setup_approach_phase(),
                'cleanup': lambda: robot.stop(),
                'dynamic_check': lambda: check_target_distance()
            }
        ]
        
        def setup_approach_phase():
            """Setup for target approach phase"""
            robot.stop()
            target.stop()
            
            # Calculate direction to target
            robot_pos = robot.pose.position
            target_pos = target.pose.position
            direction = target_pos - robot_pos
            distance = np.linalg.norm(direction)
            
            # Calculate approach parameters
            
            if distance > 0.5:  # Only move if reasonably far
                # Normalize world direction
                world_direction = direction / distance
                
                # Convert world direction to robot local coordinates
                local_direction = robot.pose.rotation.inv().apply(world_direction)
                
                # Calculate speed based on distance
                speed = min(1.0, distance * 0.5)  # Slower approach
                
                # Set velocity in robot's local coordinate frame
                robot.set_velocity(Velocity(linear_x=local_direction[0] * speed, 
                                          linear_y=local_direction[1] * speed,
                                          linear_z=local_direction[2] * speed * 0.1))
            # If already close, no movement needed
        
        def check_target_distance():
            """Check if robot should continue approaching target"""
            current_distance = np.linalg.norm(target.pose.position - robot.pose.position)
            if current_distance < 1.0:  # Stop when close enough
                robot.stop()
                return False  # Stop the phase
            return True  # Continue
        
        try:
            print(f"Starting simulation... (real-time factor: {real_time_factor:.1f}x)")
            sim_start_time = time.time()
            
            # Run all phases
            for phase in phases:
                if shutdown_requested:
                    break
                
                print(phase['name'])
                run_simulation_phase(phase['end_time'], phase['setup'], phase.get('dynamic_check'))
                
                if phase['cleanup']:
                    phase['cleanup']()
            
            if not shutdown_requested:
                print("Simulation completed!")
            else:
                print("Simulation stopped by user request")
                
        except Exception as e:
            print(f"Simulation error: {e}")
        finally:
            # Ensure all objects are stopped
            robot.stop()
            target.stop()
        
    # Start simulation in background thread
    sim_thread = threading.Thread(target=simulation_thread)
    sim_thread.daemon = True
    sim_thread.start()
    
    # Show visualization (blocking)
    try:
        viz.show(block=True)
    except KeyboardInterrupt:
        print("\nVisualization stopped by user")
        shutdown_requested = True
    finally:
        # Clean up
        viz.cleanup()
        if 'sim_thread' in locals() and sim_thread.is_alive():
            sim_thread.join(timeout=1.0)


def simple_visualization_demo():
    """Simple demo without real-time simulation"""
    
    env = simpy.Environment()
    viz = Object3DVisualizer()
    
    # Create a few objects
    obj1 = SimulationObject(env, ObjectParameters(
        name="obj1", object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0)))
        
    obj2 = SimulationObject(env, ObjectParameters(
        name="obj2", object_type=ObjectType.STATIC,
        initial_pose=Pose(x=2.0, y=1.0, z=0.5, yaw=math.pi/4)))
    
    obj1.connect_to(obj2, Pose(x=2.0, y=1.0, z=0.5))
    
    viz.add_object(obj1, color='blue', marker='o', size=150)
    viz.add_object(obj2, color='red', marker='s', size=100)
    
    print("Demo: Static visualization of connected objects")
    print("- Blue circle: Dynamic object")
    print("- Red square: Static object") 
    print("- Dashed line: Connection")
    print("- RGB axes: Object coordinate frames")
    
    # Update visualization once
    viz._update_plot(0)
    viz.show(block=True)


if __name__ == "__main__":
    import sys
    import numpy as np
    
    print("SimPyROS 3D Visualization Demo")
    print("Usage: python visualization_demo.py [static|<real_time_factor>]")
    print("  static - Static visualization")
    print("  <number> - Real-time factor (1.0=real-time, 0.5=half speed, 2.0=double speed)")
    print()
    
    # Parse command line arguments
    real_time_factor = 1.0
    static_mode = False
    
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg == "static":
            static_mode = True
        else:
            try:
                real_time_factor = float(arg)
                if real_time_factor <= 0:
                    print("Warning: Real-time factor must be positive, using 1.0")
                    real_time_factor = 1.0
            except ValueError:
                print(f"Warning: Invalid real-time factor '{arg}', using 1.0")
                real_time_factor = 1.0
    
    if static_mode:
        print("Running static demo...")
        simple_visualization_demo()
    else:
        print(f"Running animated simulation demo (speed: {real_time_factor:.1f}x)...")
        print("- Objects: Robot (blue), Sensor (green), Target (red), Obstacles (gray/brown)")
        print("- Press Ctrl+C to stop")
        if real_time_factor != 1.0:
            print(f"- Animation speed: {real_time_factor:.1f}x {'faster' if real_time_factor > 1.0 else 'slower'} than real-time")
        print()
        
        try:
            run_simulation_with_visualization(real_time_factor)
        except KeyboardInterrupt:
            print("\nDemo stopped by user")
        except Exception as e:
            print(f"Error: {e}")
            print("Make sure matplotlib is installed: pip install matplotlib")