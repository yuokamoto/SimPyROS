#!/usr/bin/env python3
"""
Simple real-time demonstration using threading and time.sleep
This avoids dependency on simpy.rt and works in any environment
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import threading
import time
import math
import signal
import simpy
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from visualizer import Object3DVisualizer


class SimpleRealtimeDemo:
    """Simple real-time demo using threading"""
    
    def __init__(self, real_time_factor=1.0):
        self.real_time_factor = real_time_factor
        self.running = True
        self.start_time = None
        
        # Setup signal handling
        signal.signal(signal.SIGINT, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        print("\nStopping demo...")
        self.running = False
        sys.exit(0)
        
    def run_demo(self, duration_seconds=20):
        """Run the demo for specified duration"""
        
        print("Simple Real-time Motion Demo")
        print("============================")
        print(f"Duration: {duration_seconds} seconds")
        print("Watch the blue robot move in circular patterns")
        print("The green sensor is attached and follows the robot")
        print("Press Ctrl+C to stop early")
        print()
        
        # Create simulation environment (regular simpy, not real-time)
        env = simpy.Environment()
        
        # Create visualization
        viz = Object3DVisualizer(figure_size=(10, 8), grid_size=4.0)
        viz.set_trajectory_length(500)  # Keep more trajectory points
        
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
        
        # Connect sensor to robot
        robot.connect_to(sensor, Pose(x=1.5, y=0.0, z=0.3))
        
        # Add to visualization
        viz.add_object(robot, color='blue', marker='o', size=150, show_trajectory=True)
        viz.add_object(sensor, color='green', marker='^', size=100, show_trajectory=True)
        
        # Motion control function
        def update_motion(t):
            """Update robot motion based on time"""
            if not self.running:
                robot.stop()
                return
                
            # Different motion patterns based on time
            if t < 5:
                # Phase 1: Circular motion
                radius = 1.5
                speed = 0.8
                robot.pose = Pose(
                    x=radius * math.cos(speed * t),
                    y=radius * math.sin(speed * t),
                    z=1.0 + 0.2 * math.sin(2 * speed * t),
                    yaw=speed * t
                )
            elif t < 10:
                # Phase 2: Figure-8 motion
                a, b = 2.0, 1.0
                omega = 0.6
                robot.pose = Pose(
                    x=a * math.sin(omega * t),
                    y=b * math.sin(2 * omega * t),
                    z=1.0 + 0.3 * math.cos(omega * t),
                    yaw=omega * t * 0.5
                )
            elif t < 15:
                # Phase 3: Spiral motion
                spiral_r = 0.1 * t
                omega = 1.2
                robot.pose = Pose(
                    x=spiral_r * math.cos(omega * t),
                    y=spiral_r * math.sin(omega * t),
                    z=1.0 + 0.1 * t,
                    yaw=omega * t
                )
            else:
                # Phase 4: Oscillating motion
                amp = 2.0
                freq = 2.0
                robot.pose = Pose(
                    x=amp * math.cos(freq * t) * math.exp(-0.1 * (t - 15)),
                    y=amp * math.sin(freq * t) * math.exp(-0.1 * (t - 15)),
                    z=1.0 + 0.5 * math.sin(freq * t),
                    yaw=freq * t
                )
        
        # Real-time update loop
        def realtime_updater():
            """Update simulation in real-time"""
            self.start_time = time.time()
            
            while self.running:
                current_time = time.time()
                sim_time = (current_time - self.start_time) * self.real_time_factor
                
                if sim_time > duration_seconds:
                    print(f"Demo completed after {duration_seconds} seconds")
                    break
                
                # Update motion
                update_motion(sim_time)
                
                # Show progress every 5 seconds
                if int(sim_time) % 5 == 0 and int(sim_time) != getattr(self, '_last_progress', -1):
                    self._last_progress = int(sim_time)
                    print(f"t={sim_time:.1f}s - Running motion phase {min(4, int(sim_time/5)+1)}")
                
                # Sleep to control update rate
                time.sleep(0.05)  # 20 FPS
            
            self.running = False
        
        # Start updater thread
        updater_thread = threading.Thread(target=realtime_updater)
        updater_thread.daemon = True
        updater_thread.start()
        
        # Start visualization
        viz.start_animation()
        
        try:
            # Show visualization (this will block until window is closed)
            viz.show(block=True)
        except KeyboardInterrupt:
            print("\nDemo interrupted")
        finally:
            self.running = False
            viz.cleanup()
            print("Demo finished!")


def quick_test():
    """Quick 10-second test"""
    print("Quick 10-second test...")
    demo = SimpleRealtimeDemo(real_time_factor=1.0)
    demo.run_demo(duration_seconds=10)


def slow_motion_test():
    """Slow motion test"""
    print("Slow motion test (0.5x speed)...")
    demo = SimpleRealtimeDemo(real_time_factor=0.5)
    demo.run_demo(duration_seconds=15)


def fast_motion_test():
    """Fast motion test"""
    print("Fast motion test (2x speed)...")
    demo = SimpleRealtimeDemo(real_time_factor=2.0)
    demo.run_demo(duration_seconds=10)


if __name__ == "__main__":
    import os
    
    # Check if we have display
    if 'DISPLAY' not in os.environ:
        print("No display detected. This demo requires a GUI environment.")
        print("In headless mode, consider running test_visualization_static.py instead.")
        sys.exit(1)
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
        if mode == "quick":
            quick_test()
        elif mode == "slow":
            slow_motion_test()
        elif mode == "fast":
            fast_motion_test()
        else:
            print(f"Unknown mode: {mode}")
            print("Available modes: quick, slow, fast")
    else:
        # Default demo
        demo = SimpleRealtimeDemo(real_time_factor=1.0)
        demo.run_demo(duration_seconds=20)