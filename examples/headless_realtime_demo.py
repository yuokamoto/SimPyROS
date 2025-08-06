#!/usr/bin/env python3
"""
Headless real-time demonstration - saves frames instead of showing window
Perfect for testing in environments without GUI
"""

import threading
import time
import math
import os
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from visualizer import Object3DVisualizer
import simpy


def headless_realtime_demo(duration=10, frame_interval=1.0):
    """Run real-time demo and save frames periodically"""
    
    print("Headless Real-time Motion Demo")
    print("==============================")
    print(f"Duration: {duration} seconds")
    print(f"Saving frames every {frame_interval} seconds")
    print("Files will be saved as frame_000.png, frame_001.png, etc.")
    print()
    
    # Create simulation environment
    env = simpy.Environment()
    
    # Create visualization  
    viz = Object3DVisualizer(figure_size=(10, 8), grid_size=4.0)
    
    # Create objects
    robot = SimulationObject(env, ObjectParameters(
        name="robot",
        object_type=ObjectType.DYNAMIC, 
        initial_pose=Pose(x=0.0, y=0.0, z=1.0),
        update_interval=0.05
    ))
    
    sensor = SimulationObject(env, ObjectParameters(
        name="sensor",
        object_type=ObjectType.STATIC,
        initial_pose=Pose(x=1.5, y=0.0, z=1.3)
    ))
    
    robot.connect_to(sensor, Pose(x=1.5, y=0.0, z=0.3))
    
    # Add to visualization
    viz.add_object(robot, color='blue', marker='o', size=150, show_trajectory=True)
    viz.add_object(sensor, color='green', marker='^', size=100, show_trajectory=True)
    
    # Motion patterns
    def update_motion(t):
        """Update robot motion based on time"""
        if t < 3:
            # Circular motion
            radius = 1.5
            speed = 1.0
            x = radius * math.cos(speed * t)
            y = radius * math.sin(speed * t) 
            z = 1.0 + 0.3 * math.sin(2 * speed * t)
            yaw = speed * t
        elif t < 6:
            # Figure-8
            a, b = 2.0, 1.0
            omega = 0.8
            x = a * math.sin(omega * t)
            y = b * math.sin(2 * omega * t)
            z = 1.0 + 0.2 * math.cos(omega * t)
            yaw = omega * t * 0.5
        else:
            # Spiral
            spiral_r = 0.2 * (t - 6)
            omega = 1.5
            x = spiral_r * math.cos(omega * t)
            y = spiral_r * math.sin(omega * t)
            z = 1.0 + 0.1 * (t - 6)
            yaw = omega * t
        
        robot.pose = Pose(x=x, y=y, z=z, yaw=yaw)
    
    # Simulation loop
    start_time = time.time()
    frame_count = 0
    last_frame_time = 0
    
    print("Starting simulation...")
    
    while True:
        current_time = time.time()
        sim_time = current_time - start_time
        
        if sim_time >= duration:
            print(f"Completed {duration} seconds of simulation")
            break
        
        # Update motion
        update_motion(sim_time)
        
        # Save frame periodically
        if sim_time - last_frame_time >= frame_interval:
            viz._update_plot(0)  # Update visualization
            os.makedirs("output", exist_ok=True)
            filename = f"output/frame_{frame_count:03d}.png"
            viz.save_frame(filename, dpi=150)
            print(f"t={sim_time:.1f}s: Saved {filename}")
            
            last_frame_time = sim_time
            frame_count += 1
        
        # Progress updates
        progress = int(sim_time)
        if hasattr(update_motion, '_last_progress'):
            if progress != update_motion._last_progress and progress % 2 == 0:
                print(f"t={sim_time:.1f}s: Running motion pattern...")
                update_motion._last_progress = progress
        else:
            update_motion._last_progress = progress
        
        # Control frame rate
        time.sleep(0.1)  # 10 FPS
    
    # Save final frame
    viz._update_plot(0)
    final_filename = f"output/final_frame.png"
    viz.save_frame(final_filename, dpi=200)
    print(f"Saved final frame: {final_filename}")
    
    # Cleanup
    viz.cleanup()
    
    print(f"\nDemo completed!")
    print(f"Total frames saved: {frame_count + 1}")
    print(f"Files saved in current directory: {os.getcwd()}")
    
    return frame_count + 1


def quick_demo():
    """Quick 5-second demo"""
    print("Quick 5-second demo...")
    return headless_realtime_demo(duration=5, frame_interval=1.0)


def detailed_demo():
    """Detailed demo with more frames"""
    print("Detailed demo with more frames...")
    return headless_realtime_demo(duration=12, frame_interval=0.5)


def motion_analysis_demo():
    """Generate frames for motion analysis"""
    print("Motion analysis demo - high frame rate...")
    return headless_realtime_demo(duration=8, frame_interval=0.2)


if __name__ == "__main__":
    import sys
    
    try:
        if len(sys.argv) > 1:
            mode = sys.argv[1].lower()
            if mode == "quick":
                frames = quick_demo()
            elif mode == "detailed":  
                frames = detailed_demo()
            elif mode == "analysis":
                frames = motion_analysis_demo()
            else:
                print(f"Unknown mode: {mode}")
                print("Available modes: quick, detailed, analysis")
                sys.exit(1)
        else:
            # Default
            frames = headless_realtime_demo()
            
        print(f"\nSuccess! Generated {frames} frame files.")
        print("You can view the motion by opening the PNG files in order.")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"Demo error: {e}")
        import traceback
        traceback.print_exc()