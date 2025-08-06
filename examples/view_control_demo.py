#!/usr/bin/env python3
"""
View control demonstration - shows how mouse view changes persist during animation
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import simpy
import math
import threading
import time
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from visualizer import Object3DVisualizer


def view_control_demo():
    """Demonstrate persistent view control during animation"""
    
    env = simpy.Environment()
    viz = Object3DVisualizer(figure_size=(12, 9), grid_size=6.0)
    
    # Create a simple moving object
    robot = SimulationObject(env, ObjectParameters(
        name="robot",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=0.0, z=1.0),
        update_interval=0.05
    ))
    
    viz.add_object(robot, color='blue', marker='o', size=150, show_trajectory=True)
    
    # Setup keyboard controls
    def on_key_press(event):
        if event.key == 'h':
            print("\\n=== View Control Help ===")
            print("Mouse controls:")
            print("  - Left drag: Rotate view (will persist during animation!)")
            print("  - Right drag: Pan view")
            print("  - Mouse wheel: Zoom in/out")
            print("\\nKeyboard shortcuts:")
            print("  - 'r': Reset to default view")
            print("  - 'v': Print current view angles")
            print("  - 'h': Show this help")
            print("  - 'q': Quit")
            print("========================\\n")
        elif event.key == 'r':
            viz.set_view(elev=20, azim=45)
            print("View reset to default (elev=20, azim=45)")
        elif event.key == 'v':
            print(f"Current view: elev={viz.ax.elev:.1f}°, azim={viz.ax.azim:.1f}°")
        elif event.key == 'q':
            viz.cleanup()
    
    viz.fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    # Motion controller
    def motion_controller():
        start_time = time.time()
        print("Starting circular motion...")
        print("Try dragging with left mouse button to change view!")
        print("Press 'h' for help, 'q' to quit")
        
        while True:
            try:
                current_time = time.time() - start_time
                
                # Simple circular motion
                radius = 2.0
                speed = 0.5
                x = radius * math.cos(speed * current_time)
                y = radius * math.sin(speed * current_time)
                z = 1.0 + 0.3 * math.sin(2 * speed * current_time)
                yaw = speed * current_time
                
                robot.pose = Pose(x=x, y=y, z=z, yaw=yaw)
                time.sleep(0.05)  # 20 FPS
                
            except Exception as e:
                print(f"Motion controller error: {e}")
                break
    
    # Start animation and motion
    viz.start_animation()
    
    motion_thread = threading.Thread(target=motion_controller)
    motion_thread.daemon = True
    motion_thread.start()
    
    print("\\n=== View Control Demo ===")
    print("The robot will move in a circle.")
    print("Try changing the view with your mouse - it should persist!")
    print("Press 'h' for help, Ctrl+C to quit")
    print("========================\\n")
    
    try:
        viz.show(block=True)
    except KeyboardInterrupt:
        print("\\nDemo stopped by user")
    finally:
        viz.cleanup()


if __name__ == "__main__":
    view_control_demo()