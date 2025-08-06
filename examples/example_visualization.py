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


def run_simulation_with_visualization():
    """Run simulation with 3D visualization"""
    
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
        object_type=ObjectType.STATIC,
        initial_pose=Pose(x=1.5, y=0.0, z=0.8)
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
    
    def simulation_thread():
        """Run simulation in separate thread"""
        nonlocal shutdown_requested
        
        try:
            print("Starting simulation...")
            
            # Phase 1: Robot moves in circle
            if not shutdown_requested:
                print("Phase 1: Robot circular motion")
                robot.set_velocity(Velocity(linear_x=1.0, angular_z=0.8))
                env.run(until=8)
                robot.stop()
            
            # Phase 2: Target moves in figure-8
            if not shutdown_requested:
                print("Phase 2: Target figure-8 motion")
                target.set_velocity(Velocity(linear_x=0.8, linear_y=0.5, angular_z=0.6))
                env.run(until=15)
            
            # Phase 3: Both objects move
            if not shutdown_requested:
                print("Phase 3: Complex motion")
                robot.set_velocity(Velocity(linear_x=0.5, linear_y=0.3, angular_z=-0.4))
                target.set_velocity(Velocity(linear_x=-0.6, linear_y=0.4, angular_z=0.9))
                env.run(until=25)
            
            # Phase 4: Robot approaches target
            if not shutdown_requested:
                print("Phase 4: Robot approaches target")
                robot.stop()
                target.stop()
                
                # Calculate direction to target
                robot_pos = robot.pose.position
                target_pos = target.pose.position
                direction = target_pos - robot_pos
                distance = np.linalg.norm(direction)
                
                if distance > 0.1:
                    direction = direction / distance  # Normalize
                    speed = min(1.0, distance)  # Slow down as we approach
                    robot.set_velocity(Velocity(linear_x=direction[0] * speed, 
                                              linear_y=direction[1] * speed,
                                              linear_z=direction[2] * speed * 0.1))
                    env.run(until=30)
                    robot.stop()
            
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
    
    # Check command line argument
    if len(sys.argv) > 1 and sys.argv[1] == "static":
        print("Running static demo...")
        simple_visualization_demo()
    else:
        print("Running animated simulation demo...")
        print("- Objects: Robot (blue), Sensor (green), Target (red), Obstacles (gray/brown)")
        print("- Use 'python example_visualization.py static' for static demo")
        
        try:
            run_simulation_with_visualization()
        except KeyboardInterrupt:
            print("\nDemo stopped by user")
        except Exception as e:
            print(f"Error: {e}")
            print("Make sure matplotlib is installed: pip install matplotlib")