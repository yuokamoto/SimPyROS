#!/usr/bin/env python3
"""
Interactive visualization demo with enhanced mouse controls
Shows how to use mouse interactions during simulation
"""

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
import matplotlib.pyplot as plt


class InteractiveVisualizationDemo:
    """Enhanced interactive visualization with mouse controls"""
    
    def __init__(self):
        self.env = simpy.Environment()
        self.viz = Object3DVisualizer(figure_size=(12, 9), grid_size=8.0)
        self.objects = {}
        self.paused = False
        
    def setup_objects(self):
        """Create simulation objects"""
        # Robot
        self.objects['robot'] = SimulationObject(self.env, ObjectParameters(
            name="robot",
            object_type=ObjectType.DYNAMIC,
            initial_pose=Pose(x=0.0, y=0.0, z=1.0),
            update_interval=0.05
        ))
        
        # Sensor attached to robot
        self.objects['sensor'] = SimulationObject(self.env, ObjectParameters(
            name="sensor", 
            object_type=ObjectType.STATIC,
            initial_pose=Pose(x=2.0, y=0.0, z=1.5)
        ))
        
        # Target
        self.objects['target'] = SimulationObject(self.env, ObjectParameters(
            name="target",
            object_type=ObjectType.DYNAMIC,
            initial_pose=Pose(x=4.0, y=4.0, z=2.0),
            update_interval=0.05
        ))
        
        # Connect robot and sensor
        self.objects['robot'].connect_to(self.objects['sensor'], Pose(x=2.0, y=0.0, z=0.5))
        
        # Add to visualizer
        self.viz.add_object(self.objects['robot'], color='blue', marker='o', size=150, show_trajectory=True)
        self.viz.add_object(self.objects['sensor'], color='green', marker='^', size=100, show_trajectory=True)
        self.viz.add_object(self.objects['target'], color='red', marker='*', size=200, show_trajectory=True)
        
    def setup_interactive_controls(self):
        """Setup interactive controls and event handlers"""
        
        def on_key_press(event):
            """Handle keyboard events"""
            if event.key == 'p':
                self.paused = not self.paused
                print(f"Simulation {'paused' if self.paused else 'resumed'}")
            elif event.key == 'r':
                # Reset view
                self.viz.set_view(elev=20, azim=45)
                print("View reset to default")
            elif event.key == 'c':
                # Clear trajectories
                self.viz.clear_trajectories()
                print("Trajectories cleared")
            elif event.key == 't':
                # Toggle connections
                self.viz.toggle_connections()
                print(f"Connections {'visible' if self.viz.show_connections else 'hidden'}")
            elif event.key == 'q':
                # Quit
                self.viz.cleanup()
                plt.close('all')
                
        def on_mouse_press(event):
            """Handle mouse events"""
            if event.inaxes == self.viz.ax:
                if event.dblclick:
                    print(f"Double-clicked at: ({event.xdata:.2f}, {event.ydata:.2f})")
                    # Could add functionality like teleporting objects to clicked location
                    
        # Connect event handlers
        self.viz.fig.canvas.mpl_connect('key_press_event', on_key_press)
        self.viz.fig.canvas.mpl_connect('button_press_event', on_mouse_press)
        
        # Print instructions
        print("\\n=== Interactive Controls ===")
        print("Mouse:")
        print("  - Left drag: Rotate view")
        print("  - Right drag: Pan view") 
        print("  - Mouse wheel: Zoom in/out")
        print("  - Double-click: Print coordinates")
        print("\\nKeyboard:")
        print("  - 'p': Pause/Resume simulation")
        print("  - 'r': Reset view to default")
        print("  - 'c': Clear trajectories")
        print("  - 't': Toggle connection lines")
        print("  - 'q': Quit")
        print("  - Ctrl+C: Emergency stop")
        print("========================\\n")
        
    def motion_controller(self):
        """Control object motion with different patterns"""
        start_time = time.time()
        
        while True:
            if self.paused:
                time.sleep(0.1)
                continue
                
            current_time = time.time() - start_time
            
            # Robot circular motion
            radius = 2.0
            speed = 0.5
            robot_x = radius * math.cos(speed * current_time)
            robot_y = radius * math.sin(speed * current_time)
            robot_z = 1.0 + 0.3 * math.sin(2 * speed * current_time)
            robot_yaw = speed * current_time
            
            self.objects['robot'].pose = Pose(x=robot_x, y=robot_y, z=robot_z, yaw=robot_yaw)
            
            # Target figure-8 motion
            a, b = 3.0, 2.0
            omega = 0.3
            target_x = a * math.sin(omega * current_time)
            target_y = b * math.sin(2 * omega * current_time)
            target_z = 2.0 + 0.5 * math.cos(omega * current_time)
            
            self.objects['target'].pose = Pose(x=target_x, y=target_y, z=target_z)
            
            time.sleep(0.05)  # 20 FPS
    
    def run(self):
        """Run interactive demo"""
        try:
            print("Interactive Visualization Demo")
            print("Setting up objects...")
            self.setup_objects()
            
            print("Setting up interactive controls...")
            self.setup_interactive_controls()
            
            print("Starting animation...")
            self.viz.start_animation()
            
            # Start motion controller in background thread
            motion_thread = threading.Thread(target=self.motion_controller)
            motion_thread.daemon = True
            motion_thread.start()
            
            print("Showing visualization...")
            print("Try moving the mouse and using keyboard shortcuts!")
            
            # Show visualization (blocking)
            self.viz.show(block=True)
            
        except KeyboardInterrupt:
            print("\\nDemo interrupted by user")
        except Exception as e:
            print(f"Demo error: {e}")
        finally:
            self.viz.cleanup()
            print("Demo finished!")


if __name__ == "__main__":
    demo = InteractiveVisualizationDemo()
    demo.run()