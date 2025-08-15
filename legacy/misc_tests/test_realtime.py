import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#!/usr/bin/env python3
"""
Simple real-time simulation test for viewing motion in real-time
"""

import simpy.rt
import time
import math
import numpy as np
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from visualizer import Object3DVisualizer


def test_realtime_motion():
    """Test real-time motion visualization"""
    
    print("Real-time Motion Test")
    print("=====================")
    print("This demo will run for 20 seconds in real-time")
    print("You should see smooth motion of objects")
    print("Press Ctrl+C to stop early")
    print()
    
    # Create real-time environment (1.0 = real-time speed)
    env = simpy.rt.RealtimeEnvironment(factor=1.0, strict=False)
    
    # Create visualization
    viz = Object3DVisualizer(figure_size=(12, 8), grid_size=4.0)
    
    # Create objects
    robot = SimulationObject(env, ObjectParameters(
        name="robot",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=0.0, z=1.0),
        update_interval=0.02  # 50 FPS updates
    ))
    
    sensor = SimulationObject(env, ObjectParameters(
        name="sensor",
        object_type=ObjectType.STATIC,
        initial_pose=Pose(x=1.0, y=0.0, z=1.2)
    ))
    
    robot.connect_to(sensor, Pose(x=1.0, y=0.0, z=0.2))
    
    # Add to visualization
    viz.add_object(robot, color='blue', marker='o', size=150, show_trajectory=True)
    viz.add_object(sensor, color='green', marker='^', size=100, show_trajectory=True)
    
    def motion_controller():
        """Control robot motion in real-time"""
        try:
            print(f"t=0s: Starting motion...")
            
            # Phase 1: Move forward
            robot.set_velocity(Velocity(linear_x=1.0))
            yield env.timeout(3.0)  # 3 seconds
            print(f"t={env.now:.1f}s: Starting circular motion")
            
            # Phase 2: Circular motion
            robot.set_velocity(Velocity(linear_x=0.8, angular_z=0.6))
            yield env.timeout(5.0)  # 5 seconds
            print(f"t={env.now:.1f}s: Figure-8 pattern")
            
            # Phase 3: Figure-8
            robot.set_velocity(Velocity(linear_x=0.6, linear_y=0.4, angular_z=0.8))
            yield env.timeout(4.0)  # 4 seconds
            print(f"t={env.now:.1f}s: Reverse direction")
            
            # Phase 4: Reverse
            robot.set_velocity(Velocity(linear_x=-0.5, angular_z=-0.7))
            yield env.timeout(3.0)  # 3 seconds
            print(f"t={env.now:.1f}s: Random walk")
            
            # Phase 5: Random-like motion
            robot.set_velocity(Velocity(linear_x=0.7, linear_y=-0.3, angular_z=1.2))
            yield env.timeout(3.0)  # 3 seconds
            print(f"t={env.now:.1f}s: Stopping")
            
            # Phase 6: Stop
            robot.stop()
            yield env.timeout(2.0)  # 2 seconds
            print(f"t={env.now:.1f}s: Motion test completed!")
            
        except Exception as e:
            print(f"Motion controller error: {e}")
    
    # Start motion controller
    env.process(motion_controller())
    
    # Start visualization
    viz.start_animation()
    
    try:
        print("Starting visualization...")
        
        # Run simulation for 20 seconds
        start_time = time.time()
        viz.show(block=False)  # Non-blocking show
        
        # Run environment
        env.run(until=20.0)
        
        # Keep visualization open for a moment
        print("Simulation completed. Keeping visualization open for 3 more seconds...")
        time.sleep(3.0)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test error: {e}")
    finally:
        viz.cleanup()
        end_time = time.time()
        real_duration = end_time - start_time if 'start_time' in locals() else 0
        print(f"Real time elapsed: {real_duration:.1f}s")
        print("Test completed!")


def test_speed_comparison():
    """Compare different simulation speeds"""
    
    speeds = [0.5, 1.0, 2.0]  # Half speed, real-time, double speed
    
    for speed in speeds:
        print(f"\nTesting at {speed}x speed...")
        env = simpy.rt.RealtimeEnvironment(factor=speed, strict=False)
        
        robot = SimulationObject(env, ObjectParameters(
            name="robot", object_type=ObjectType.DYNAMIC,
            initial_pose=Pose(x=0.0, y=0.0, z=1.0), update_interval=0.05
        ))
        
        def simple_motion():
            robot.set_velocity(Velocity(angular_z=1.0))  # 1 radian/second
            yield env.timeout(math.pi)  # Half circle in simulation time
            robot.stop()
            
        env.process(simple_motion())
        
        start_time = time.time()
        env.run()
        end_time = time.time()
        
        real_duration = end_time - start_time
        expected_duration = math.pi / speed
        
        print(f"Speed {speed}x: Real time = {real_duration:.2f}s, Expected = {expected_duration:.2f}s")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "speed":
        test_speed_comparison()
    else:
        test_realtime_motion()