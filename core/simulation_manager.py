#!/usr/bin/env python3
"""
SimulationManager - Simplified Simulation Execution

Provides a unified interface for simulation management, reducing user code complexity
from ~100 lines to ~20 lines while maintaining full functionality.

Features:
- Automatic lifecycle management
- Dynamic object updates  
- Headless/visualization toggle
- ROS 2 compatibility layer
- Simplified user interface
"""

import time
import threading
import signal
import sys
from typing import Dict, List, Optional, Callable, Any, Union
from dataclasses import dataclass
import warnings

import simpy
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from core.robot import Robot, create_robot_from_urdf, RobotParameters
from core.simulation_object import SimulationObject, ObjectParameters, Pose, Velocity
from core.pyvista_visualizer import URDFRobotVisualizer, create_urdf_robot_visualizer


@dataclass
class SimulationConfig:
    """Configuration for simulation manager"""
    update_rate: float = 100.0  # Hz
    visualization: bool = True
    visualization_update_rate: float = 30.0  # Hz
    window_size: tuple = (1200, 800)
    auto_setup_scene: bool = True
    real_time_factor: float = 1.0  # Real time multiplier (1.0 = real time, 0.5 = half speed, 2.0 = double speed)
    
    
class ControlCallback:
    """Wrapper for user control callbacks"""
    
    def __init__(self, callback: Callable, frequency: float = 10.0):
        self.callback = callback
        self.frequency = frequency
        self.last_call = 0.0
        
    def should_call(self, current_time: float) -> bool:
        """Check if callback should be called based on frequency"""
        interval = 1.0 / self.frequency
        return (current_time - self.last_call) >= interval
        
    def call(self, dt: float, current_time: float):
        """Call the callback and update last call time"""
        self.callback(dt)
        self.last_call = current_time


class SimulationManager:
    """
    Central simulation manager for simplified robot simulation
    
    This class provides a high-level interface that reduces the complexity
    of setting up and running robot simulations. It handles:
    - SimPy environment management
    - Robot and object lifecycle
    - Visualization setup and updates
    - Control callback management
    - Graceful shutdown handling
    
    Example usage:
        sim = SimulationManager()
        robot = sim.add_robot_from_urdf("my_robot", "path/to/robot.urdf")
        sim.set_robot_control_callback("my_robot", my_control_function)
        sim.run()
    """
    
    def __init__(self, config: Optional[SimulationConfig] = None):
        """
        Initialize simulation manager
        
        Args:
            config: Simulation configuration, uses defaults if None
        """
        self.config = config or SimulationConfig()
        
        # Core components
        self.env: Optional[simpy.Environment] = None
        self.visualizer: Optional[URDFRobotVisualizer] = None
        
        # Simulation state
        self.robots: Dict[str, Robot] = {}
        self.objects: Dict[str, SimulationObject] = {}
        self.control_callbacks: Dict[str, ControlCallback] = {}
        
        # Runtime state
        self._running = False
        self._shutdown_requested = False
        self._simulation_thread: Optional[threading.Thread] = None
        self._visualization_thread: Optional[threading.Thread] = None
        
        # Performance tracking
        self._start_time = 0.0
        self._frame_count = 0
        
        # Setup signal handlers for graceful shutdown
        self._setup_signal_handlers()
        
    def _setup_signal_handlers(self):
        """Setup signal handlers for graceful shutdown"""
        def signal_handler(signum, frame):
            print("\n🛑 Shutdown requested...")
            self._shutdown_requested = True
            self.shutdown()
            # Force exit if shutdown doesn't work
            import sys
            sys.exit(0)
            
        signal.signal(signal.SIGINT, signal_handler)
        if hasattr(signal, 'SIGTERM'):
            signal.signal(signal.SIGTERM, signal_handler)
    
    def _initialize_environment(self):
        """Initialize SimPy environment"""
        if self.env is None:
            self.env = simpy.Environment()
            print("✅ SimPy environment initialized")
    
    def _initialize_visualization(self):
        """Initialize visualization system if enabled"""
        if not self.config.visualization:
            return
            
        if self.visualizer is None:
            self.visualizer = create_urdf_robot_visualizer(
                interactive=True,
                window_size=self.config.window_size
            )
            
            if self.visualizer.available:
                print("✅ Visualization system initialized")
            else:
                warnings.warn("Visualization system not available, running headless")
                self.config.visualization = False
    
    def add_robot_from_urdf(self, 
                           name: str, 
                           urdf_path: str,
                           initial_pose: Optional[Pose] = None,
                           joint_update_rate: float = 100.0) -> Robot:
        """
        Add robot from URDF file
        
        Args:
            name: Unique name for the robot
            urdf_path: Path to URDF file
            initial_pose: Initial pose (defaults to origin)
            joint_update_rate: Joint control frequency in Hz
            
        Returns:
            Robot instance
            
        Raises:
            RuntimeError: If robot creation fails
        """
        if name in self.robots:
            raise ValueError(f"Robot '{name}' already exists")
        
        # Initialize environment if needed
        self._initialize_environment()
        
        try:
            # Create robot
            robot = create_robot_from_urdf(
                self.env,
                urdf_path,
                name,
                initial_pose,
                joint_update_rate
            )
            
            self.robots[name] = robot
            
            # Add to visualization if available
            if self.config.visualization:
                self._initialize_visualization()
                if self.visualizer and self.visualizer.available:
                    self.visualizer.load_robot(name, robot, urdf_path)
            
            print(f"✅ Robot '{name}' added from {urdf_path}")
            return robot
            
        except Exception as e:
            raise RuntimeError(f"Failed to create robot '{name}': {e}")
    
    def add_robot(self, name: str, robot: Robot) -> bool:
        """
        Add existing robot instance
        
        Args:
            name: Unique name for the robot
            robot: Robot instance
            
        Returns:
            Success status
        """
        if name in self.robots:
            raise ValueError(f"Robot '{name}' already exists")
        
        self.robots[name] = robot
        print(f"✅ Robot '{name}' added")
        return True
    
    def add_object(self, name: str, obj: SimulationObject) -> bool:
        """
        Add generic simulation object
        
        Args:
            name: Unique name for the object
            obj: SimulationObject instance
            
        Returns:
            Success status
        """
        if name in self.objects:
            raise ValueError(f"Object '{name}' already exists")
        
        self.objects[name] = obj
        print(f"✅ Object '{name}' added")
        return True
    
    def set_robot_control_callback(self, 
                                  name: str, 
                                  callback: Callable[[float], None],
                                  frequency: float = 10.0) -> bool:
        """
        Set periodic control callback for robot
        
        Args:
            name: Robot name
            callback: Control function that takes dt (time delta) as argument
            frequency: Callback frequency in Hz
            
        Returns:
            Success status
        """
        if name not in self.robots:
            raise ValueError(f"Robot '{name}' not found")
        
        self.control_callbacks[name] = ControlCallback(callback, frequency)
        print(f"✅ Control callback set for robot '{name}' at {frequency} Hz")
        return True
    
    def get_robot(self, name: str) -> Optional[Robot]:
        """Get robot by name"""
        return self.robots.get(name)
    
    def get_object(self, name: str) -> Optional[SimulationObject]:
        """Get object by name"""
        return self.objects.get(name)
    
    def set_robot_velocity(self, name: str, velocity: Velocity) -> bool:
        """Set robot velocity"""
        robot = self.get_robot(name)
        if robot:
            robot.set_velocity(velocity)
            return True
        return False
    
    def set_robot_joint_position(self, 
                                name: str, 
                                joint_name: str, 
                                position: float,
                                max_velocity: Optional[float] = None) -> bool:
        """Set robot joint position"""
        robot = self.get_robot(name)
        if robot:
            robot.set_joint_position(joint_name, position, max_velocity)
            return True
        return False
    
    def set_robot_joint_positions(self, 
                                 name: str, 
                                 positions: Dict[str, float],
                                 max_velocity: Optional[float] = None) -> bool:
        """Set multiple robot joint positions"""
        robot = self.get_robot(name)
        if robot:
            robot.set_joint_positions(positions, max_velocity)
            return True
        return False
    
    def _simulation_loop(self):
        """Main simulation loop running in separate thread"""
        if not self.env:
            print("❌ Environment not initialized")
            return
            
        dt = 1.0 / self.config.update_rate
        # Apply real time factor to sleep calculation
        real_time_dt = dt / self.config.real_time_factor
        
        while self._running and not self._shutdown_requested:
            loop_start = time.time()
            
            # Process control callbacks
            current_time = time.time()
            for robot_name, callback in self.control_callbacks.items():
                if callback.should_call(current_time):
                    try:
                        callback.call(dt, current_time)
                    except Exception as e:
                        print(f"⚠️ Control callback error for {robot_name}: {e}")
            
            # Update SimPy environment
            try:
                self.env.run(until=self.env.now + dt)
            except Exception as e:
                print(f"⚠️ Simulation step error: {e}")
            
            # Sleep to maintain real time factor
            # real_time_factor < 1.0: slower than real time (more sleep)
            # real_time_factor > 1.0: faster than real time (less sleep)
            elapsed = time.time() - loop_start
            sleep_time = max(0, real_time_dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            self._frame_count += 1
    
    def _visualization_loop(self):
        """Visualization update loop running in separate thread"""
        if not self.visualizer or not self.visualizer.available:
            return
            
        dt = 1.0 / self.config.visualization_update_rate
        
        while self._running and not self._shutdown_requested:
            loop_start = time.time()
            
            # Update robot visualizations
            for robot_name, robot in self.robots.items():
                try:
                    self.visualizer.update_robot_visualization(robot_name)
                except Exception as e:
                    print(f"⚠️ Visualization update error for {robot_name}: {e}")
            
            # Sleep to maintain visualization rate
            elapsed = time.time() - loop_start
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def run(self, duration: Optional[float] = None) -> bool:
        """
        Run simulation until Ctrl+C, window close, or duration expires
        
        Args:
            duration: Optional simulation duration in seconds
            
        Returns:
            Success status
        """
        if not self.robots and not self.objects:
            print("⚠️ No robots or objects to simulate")
            return False
        
        print("🚀 Starting simulation...")
        print(f"   Robots: {len(self.robots)}")
        print(f"   Objects: {len(self.objects)}")
        print(f"   Update rate: {self.config.update_rate} Hz")
        print(f"   Real time factor: {self.config.real_time_factor}x")
        print(f"   Visualization: {'On' if self.config.visualization else 'Off'}")
        print("   Press Ctrl+C to stop")
        print("=" * 50)
        
        self._running = True
        self._start_time = time.time()
        
        # Start simulation thread
        self._simulation_thread = threading.Thread(
            target=self._simulation_loop,
            name="SimulationLoop"
        )
        self._simulation_thread.daemon = True
        self._simulation_thread.start()
        
        # Start visualization thread
        if self.config.visualization and self.visualizer and self.visualizer.available:
            self._visualization_thread = threading.Thread(
                target=self._visualization_loop,
                name="VisualizationLoop"  
            )
            self._visualization_thread.daemon = True
            self._visualization_thread.start()
        
        try:
            # Show interactive window first if visualization is enabled
            if self.config.visualization and self.visualizer and self.visualizer.available:
                # Show interactive window (non-blocking for duration-based runs)
                try:
                    if duration:
                        # Non-blocking show for timed runs
                        self.visualizer.plotter.show(auto_close=False, interactive_update=True)
                        # Run for specified duration
                        end_time = time.time() + duration
                        while time.time() < end_time and self._running and not self._shutdown_requested:
                            self.visualizer.plotter.update()
                            time.sleep(0.1)
                        # Close window after duration
                        self.visualizer.plotter.close()
                    else:
                        # Blocking show for indefinite runs
                        self.visualizer.plotter.show()
                except KeyboardInterrupt:
                    print("\n🛑 Visualization interrupted")
                    self._shutdown_requested = True
            else:
                # Headless mode - wait based on duration
                if duration:
                    end_time = time.time() + duration
                    while time.time() < end_time and self._running and not self._shutdown_requested:
                        time.sleep(0.1)
                else:
                    # Wait for shutdown signal
                    while self._running and not self._shutdown_requested:
                        time.sleep(0.1)
                        
        except KeyboardInterrupt:
            print("\n🛑 Interrupted by user")
            self._shutdown_requested = True
        finally:
            self.shutdown()
        
        return True
    
    def shutdown(self):
        """Graceful shutdown of simulation"""
        if not self._running and not self._shutdown_requested:
            return
            
        print("🛑 Shutting down simulation...")
        self._shutdown_requested = True
        self._running = False
        
        # Clean up visualization first (this can hang)
        if self.visualizer:
            try:
                if hasattr(self.visualizer, 'plotter') and self.visualizer.plotter:
                    self.visualizer.plotter.close()
            except Exception as e:
                print(f"⚠️ Visualization cleanup warning: {e}")
        
        # Wait for threads to finish with shorter timeout
        if self._simulation_thread and self._simulation_thread.is_alive():
            self._simulation_thread.join(timeout=1.0)
            if self._simulation_thread.is_alive():
                print("⚠️ Simulation thread didn't shut down gracefully")
            
        if self._visualization_thread and self._visualization_thread.is_alive():
            self._visualization_thread.join(timeout=1.0)
            if self._visualization_thread.is_alive():
                print("⚠️ Visualization thread didn't shut down gracefully")
        
        # Print performance statistics
        if self._start_time > 0:
            elapsed = time.time() - self._start_time
            avg_fps = self._frame_count / elapsed if elapsed > 0 else 0
            print(f"📊 Simulation ran for {elapsed:.1f}s")
            print(f"📊 Average update rate: {avg_fps:.1f} Hz")
        
        print("✅ Simulation shutdown complete")
    
    def get_simulation_info(self) -> Dict[str, Any]:
        """Get simulation information and statistics"""
        elapsed = time.time() - self._start_time if self._start_time > 0 else 0
        avg_fps = self._frame_count / elapsed if elapsed > 0 else 0
        
        return {
            'running': self._running,
            'robots': list(self.robots.keys()),
            'objects': list(self.objects.keys()),
            'elapsed_time': elapsed,
            'frame_count': self._frame_count,
            'average_fps': avg_fps,
            'config': self.config
        }


# Convenience functions for common use cases
def create_simple_simulation(visualization: bool = True, 
                           update_rate: float = 60.0,
                           real_time_factor: float = 1.0) -> SimulationManager:
    """Create a simple simulation manager with default settings"""
    config = SimulationConfig(
        update_rate=update_rate,
        visualization=visualization,
        real_time_factor=real_time_factor
    )
    return SimulationManager(config)