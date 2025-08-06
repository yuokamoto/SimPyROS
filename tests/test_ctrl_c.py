import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#!/usr/bin/env python3
"""
Test script to verify Ctrl+C handling in visualization
"""

import simpy
import math
import numpy as np
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from visualizer import Object3DVisualizer
import threading
import time


def test_ctrl_c_handling():
    """Test Ctrl+C handling in visualization"""
    
    print("Testing Ctrl+C handling for visualization")
    print("Instructions:")
    print("1. A simple animation will start")
    print("2. Press Ctrl+C to test graceful shutdown")
    print("3. The window should close cleanly")
    print()
    
    # Create simulation
    env = simpy.Environment()
    viz = Object3DVisualizer(figure_size=(10, 8), grid_size=5.0)
    
    # Create a simple moving object
    obj = SimulationObject(env, ObjectParameters(
        name="test_object",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=0.0, z=1.0),
        update_interval=0.05
    ))
    
    viz.add_object(obj, color='red', marker='o', size=150, show_trajectory=True)
    
    def animation_thread():
        """Simple circular motion"""
        try:
            print("Starting circular motion animation...")
            radius = 2.0
            angular_speed = 0.5  # rad/s
            
            # Infinite loop - will be interrupted by Ctrl+C
            t = 0
            while True:
                # Update object position manually for smooth circular motion
                x = radius * math.cos(angular_speed * t)
                y = radius * math.sin(angular_speed * t) 
                z = 1.0 + 0.3 * math.sin(angular_speed * t * 2)
                yaw = angular_speed * t
                
                obj.pose = Pose(x=x, y=y, z=z, yaw=yaw)
                
                time.sleep(0.05)  # 20 FPS
                t += 0.05
                
        except KeyboardInterrupt:
            print("Animation thread stopped by Ctrl+C")
        except Exception as e:
            print(f"Animation thread error: {e}")
    
    # Start animation in background
    anim_thread = threading.Thread(target=animation_thread)
    anim_thread.daemon = True
    anim_thread.start()
    
    # Start visualization
    viz.start_animation()
    
    # Show window (this should handle Ctrl+C gracefully)
    try:
        print("Visualization started. Press Ctrl+C to close.")
        viz.show(block=True)
    except KeyboardInterrupt:
        print("\nCtrl+C caught in main thread")
    finally:
        print("Cleaning up...")
        viz.cleanup()
        print("Test completed!")


def quick_static_test():
    """Quick test without animation for faster testing"""
    
    print("Quick static test - should close immediately with Ctrl+C")
    
    env = simpy.Environment()
    viz = Object3DVisualizer()
    
    # Simple static object
    obj = SimulationObject(env, ObjectParameters(
        name="static_test", object_type=ObjectType.STATIC,
        initial_pose=Pose(x=1.0, y=1.0, z=1.0, yaw=math.pi/4)
    ))
    
    viz.add_object(obj, color='blue', marker='s', size=200)
    viz._update_plot(0)
    
    try:
        print("Static visualization - Press Ctrl+C to close")
        viz.show(block=True)
    except KeyboardInterrupt:
        print("\nStatic test interrupted successfully")
    finally:
        viz.cleanup()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "quick":
        quick_static_test()
    else:
        test_ctrl_c_handling()