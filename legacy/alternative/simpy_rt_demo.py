import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import simpy.rt
import simpy
import math
import threading
import time
import signal
import numpy as np
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from visualizer import Object3DVisualizer


class RealtimeSimulationRunner:
    """Run SimPy simulation in real-time with visualization"""
    
    def __init__(self, time_scale=1.0, initial_time_factor=0.1):
        """
        Args:
            time_scale: Simulation time scale (1.0 = real-time, 2.0 = 2x speed)
            initial_time_factor: Factor for initial time acceleration (faster startup)
        """
        self.time_scale = time_scale
        self.initial_time_factor = initial_time_factor
        self.shutdown_requested = False
        
        # Create real-time environment
        self.env = simpy.rt.RealtimeEnvironment(
            factor=initial_time_factor,  # Start faster, then slow down
            strict=False  # Don't crash if simulation can't keep up
        )
        
        # Visualization
        self.viz = None
        self.objects = {}
        
        # Setup signal handling
        self._setup_signal_handlers()
        
    def _setup_signal_handlers(self):
        """Setup graceful shutdown handlers"""
        def signal_handler(signum, frame):
            print(f"\nShutdown requested (signal {signum})")
            self.shutdown_requested = True
            if self.viz:
                self.viz.cleanup()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        if hasattr(signal, 'SIGTERM'):
            signal.signal(signal.SIGTERM, signal_handler)
    
    def create_visualization(self, **viz_kwargs):
        """Create and setup visualization"""
        self.viz = Object3DVisualizer(**viz_kwargs)
        return self.viz
    
    def add_object(self, name: str, obj_params: ObjectParameters, viz_params=None):
        """Add simulation object with visualization"""
        obj = SimulationObject(self.env, obj_params)
        self.objects[name] = obj
        
        if self.viz and viz_params:
            self.viz.add_object(obj, **viz_params)
        
        return obj
    
    def time_scale_controller(self):
        """Control simulation time scale during runtime"""
        try:
            # Start with fast time for initial positioning
            yield self.env.timeout(2.0)  # 2 simulation seconds
            print("Switching to real-time...")
            self.env.factor = self.time_scale
            
        except Exception as e:
            print(f"Time scale controller error: {e}")
    
    def run_simulation_phase(self, phase_name: str, phase_function, duration: float):
        """Run a simulation phase with progress indication"""
        if self.shutdown_requested:
            return
            
        print(f"Phase: {phase_name}")
        start_time = self.env.now
        target_time = start_time + duration
        
        # Execute phase function
        phase_function()
        
        # Wait for phase duration
        while self.env.now < target_time and not self.shutdown_requested:
            yield self.env.timeout(0.1)
            
        # Progress indicator
        progress = min(100, (self.env.now - start_time) / duration * 100)
        print(f"Phase '{phase_name}' progress: {progress:.1f}%")
    
    def run(self, simulation_function, total_duration=None):
        """Run simulation with real-time synchronization"""
        try:
            print("Starting real-time simulation...")
            print("- Press Ctrl+C to stop")
            print("- Simulation will run in real-time after initial setup")
            
            # Start time scale controller
            self.env.process(self.time_scale_controller())
            
            # Start simulation
            if hasattr(simulation_function, '__call__'):
                self.env.process(simulation_function())
            
            # Start visualization if available
            if self.viz:
                self.viz.start_animation()
                
                # Run visualization in separate thread
                viz_thread = threading.Thread(target=lambda: self.viz.show(block=False))
                viz_thread.daemon = True
                viz_thread.start()
            
            # Run simulation
            if total_duration:
                print(f"Running for {total_duration} seconds...")
                self.env.run(until=total_duration)
            else:
                print("Running indefinitely...")
                self.env.run()
                
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        except Exception as e:
            print(f"Simulation error: {e}")
        finally:
            print("Simulation finished")
            if self.viz:
                self.viz.cleanup()


def demo_circular_motion():
    """Demo with circular motion"""
    
    # Create simulation runner
    runner = RealtimeSimulationRunner(time_scale=1.0, initial_time_factor=0.1)
    
    # Create visualization
    viz = runner.create_visualization(figure_size=(12, 9), grid_size=6.0)
    
    # Add objects
    robot = runner.add_object("robot", ObjectParameters(
        name="robot",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=0.0, z=1.0),
        update_interval=0.05
    ), viz_params={'color': 'blue', 'marker': 'o', 'size': 150, 'show_trajectory': True})
    
    sensor = runner.add_object("sensor", ObjectParameters(
        name="sensor",
        object_type=ObjectType.STATIC,
        initial_pose=Pose(x=2.0, y=0.0, z=1.5)
    ), viz_params={'color': 'green', 'marker': '^', 'size': 100, 'show_trajectory': True})
    
    # Connect objects
    robot.connect_to(sensor, Pose(x=2.0, y=0.0, z=0.5))
    
    def simulation_process():
        """Simulation process generator"""
        try:
            # Phase 1: Initialization
            yield runner.env.timeout(1.0)
            print("Phase 1: Starting circular motion")
            robot.set_velocity(Velocity(linear_x=1.5, angular_z=0.8))
            
            # Phase 2: Circular motion
            yield runner.env.timeout(10.0)
            print("Phase 2: Changing direction")
            robot.set_velocity(Velocity(linear_x=1.0, linear_y=0.5, angular_z=-0.6))
            
            # Phase 3: Complex motion
            yield runner.env.timeout(8.0)
            print("Phase 3: Spiral motion")
            robot.set_velocity(Velocity(linear_x=0.8, angular_z=1.2))
            
            # Phase 4: Stop
            yield runner.env.timeout(5.0)
            print("Phase 4: Stopping")
            robot.stop()
            
            yield runner.env.timeout(2.0)
            print("Simulation phases completed!")
            
        except Exception as e:
            print(f"Simulation process error: {e}")
    
    # Run simulation
    runner.run(simulation_process, total_duration=30.0)


def demo_multi_object_interaction():
    """Demo with multiple interacting objects"""
    
    runner = RealtimeSimulationRunner(time_scale=1.0)
    viz = runner.create_visualization(figure_size=(14, 10), grid_size=8.0)
    
    # Create multiple objects
    robot1 = runner.add_object("robot1", ObjectParameters(
        name="robot1", object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=-3.0, y=0.0, z=0.5), update_interval=0.05
    ), viz_params={'color': 'blue', 'marker': 'o', 'size': 150})
    
    robot2 = runner.add_object("robot2", ObjectParameters(
        name="robot2", object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=3.0, y=0.0, z=0.5), update_interval=0.05
    ), viz_params={'color': 'red', 'marker': 's', 'size': 150})
    
    target = runner.add_object("target", ObjectParameters(
        name="target", object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=4.0, z=2.0), update_interval=0.05
    ), viz_params={'color': 'gold', 'marker': '*', 'size': 200})
    
    obstacle = runner.add_object("obstacle", ObjectParameters(
        name="obstacle", object_type=ObjectType.STATIC,
        initial_pose=Pose(x=0.0, y=0.0, z=1.0)
    ), viz_params={'color': 'gray', 'marker': 'x', 'size': 300})
    
    def multi_robot_simulation():
        """Multi-robot simulation"""
        try:
            print("Multi-robot demo starting...")
            
            # Phase 1: Robots approach center
            yield runner.env.timeout(0.5)
            print("Phase 1: Robots approach center")
            robot1.set_velocity(Velocity(linear_x=1.0))
            robot2.set_velocity(Velocity(linear_x=-1.0))
            
            yield runner.env.timeout(4.0)
            
            # Phase 2: Circular dance around obstacle
            print("Phase 2: Circular dance around obstacle")
            robot1.set_velocity(Velocity(linear_x=0.5, linear_y=1.0, angular_z=0.5))
            robot2.set_velocity(Velocity(linear_x=-0.5, linear_y=-1.0, angular_z=-0.5))
            target.set_velocity(Velocity(linear_y=-0.3, angular_z=0.2))
            
            yield runner.env.timeout(8.0)
            
            # Phase 3: Pursuit behavior
            print("Phase 3: Robots chase target")
            robot1.set_velocity(Velocity(linear_x=0.8, linear_y=0.6))
            robot2.set_velocity(Velocity(linear_x=-0.6, linear_y=0.8))
            target.set_velocity(Velocity(linear_x=0.5, linear_y=-1.0, angular_z=1.0))
            
            yield runner.env.timeout(10.0)
            
            # Phase 4: Formation
            print("Phase 4: Formation flying")
            robot1.set_velocity(Velocity(linear_x=-0.3, linear_y=0.3))
            robot2.set_velocity(Velocity(linear_x=0.3, linear_y=0.3))
            target.set_velocity(Velocity(linear_y=0.5))
            
            yield runner.env.timeout(6.0)
            
            # Stop all
            print("Stopping all robots")
            robot1.stop()
            robot2.stop()
            target.stop()
            
            yield runner.env.timeout(2.0)
            print("Multi-robot demo completed!")
            
        except Exception as e:
            print(f"Multi-robot simulation error: {e}")
    
    runner.run(multi_robot_simulation, total_duration=35.0)


def interactive_demo():
    """Interactive demo with user input"""
    
    runner = RealtimeSimulationRunner(time_scale=1.0)
    viz = runner.create_visualization(figure_size=(10, 8), grid_size=5.0)
    
    robot = runner.add_object("robot", ObjectParameters(
        name="robot", object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=0.0, z=1.0), update_interval=0.05
    ), viz_params={'color': 'purple', 'marker': 'o', 'size': 200, 'show_trajectory': True})
    
    def interactive_simulation():
        """Interactive control simulation"""
        print("Interactive demo - robot will follow preset patterns")
        print("Watch the visualization and use Ctrl+C to stop anytime")
        
        patterns = [
            ("Circle", Velocity(linear_x=1.0, angular_z=1.0)),
            ("Figure-8", Velocity(linear_x=0.8, linear_y=0.4, angular_z=0.6)),
            ("Spiral", Velocity(linear_x=0.5, angular_z=1.5)),
            ("Random walk", Velocity(linear_x=0.7, linear_y=0.3, angular_z=-0.8))
        ]
        
        for i, (pattern_name, velocity) in enumerate(patterns):
            if runner.shutdown_requested:
                break
                
            print(f"\nPattern {i+1}: {pattern_name}")
            robot.set_velocity(velocity)
            yield runner.env.timeout(8.0)
        
        robot.stop()
        print("\nAll patterns completed!")
    
    runner.run(interactive_simulation, total_duration=35.0)


if __name__ == "__main__":
    print("SimPyROS Real-time Simulation Demos")
    print("Choose a demo:")
    print("1. Circular motion (default)")
    print("2. Multi-robot interaction")
    print("3. Interactive patterns")
    
    try:
        import sys
        if len(sys.argv) > 1:
            choice = sys.argv[1]
        else:
            choice = "1"
            
        if choice == "2":
            demo_multi_object_interaction()
        elif choice == "3":
            interactive_demo()
        else:
            demo_circular_motion()
            
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"Demo error: {e}")
        import traceback
        traceback.print_exc()