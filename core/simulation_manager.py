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
import signal
import sys
from typing import Dict, List, Optional, Callable, Any, Union
from dataclasses import dataclass
import warnings

import simpy
import simpy.rt
import simpy.core
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from core.robot import Robot, create_robot_from_urdf, RobotParameters
from core.simulation_object import SimulationObject, ObjectParameters, Pose, Velocity
from core.pyvista_visualizer import URDFRobotVisualizer, create_urdf_robot_visualizer
from core.optimized_pyvista_visualizer import OptimizedURDFRobotVisualizer, create_optimized_urdf_visualizer
from core.time_manager import TimeManager, set_global_time_manager


@dataclass
class SimulationConfig:
    """Configuration for simulation manager"""
    update_rate: float = 100.0  # Hz
    visualization: bool = True
    visualization_update_rate: float = 30.0  # Hz
    window_size: tuple = (1200, 800)
    auto_setup_scene: bool = True
    real_time_factor: float = 1.0  # Real time multiplier (1.0 = real time, 0.5 = half speed, 2.0 = double speed, 0.0 = max speed)
    time_step: float = 0.01  # Simulation time step in seconds
    enable_frequency_grouping: bool = True  # Auto-group robots by joint_update_rate
    
    # PyVista optimization settings
    use_optimized_visualizer: bool = True  # Use optimized PyVista visualizer
    visualization_optimization_level: str = 'balanced'  # 'performance', 'balanced', 'quality'
    enable_batch_rendering: bool = True  # Enable batch rendering for multiple robots
    
    
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
    Orchestrator for independent SimPy process-based robot simulation
    
    This class coordinates multiple independent SimPy processes:
    - Robot subsystem processes (joint control, sensors, navigation)
    - Simulation object motion processes
    - Visualization process
    - User control callback processes
    
    Features:
    - RealtimeEnvironment for accurate timing
    - Independent process architecture leveraging SimPy's strengths
    - Centralized time management
    - Event-driven robot behaviors
    
    Example usage:
        sim = SimulationManager()
        robot = sim.add_robot_from_urdf("my_robot", "path/to/robot.urdf")
        sim.set_robot_control_callback("my_robot", my_control_function)
        sim.run()
    """
    
    def __init__(self, config: Optional[SimulationConfig] = None):
        """
        Initialize simulation manager with RealtimeEnvironment
        
        Args:
            config: Simulation configuration, uses defaults if None
        """
        self.config = config or SimulationConfig()
        
        # Core components - use RealtimeEnvironment
        self.time_manager: Optional[TimeManager] = None
        self.visualizer: Optional[Union[URDFRobotVisualizer, OptimizedURDFRobotVisualizer]] = None
        
        # Simulation state
        self.robots: Dict[str, Robot] = {}
        self.objects: Dict[str, SimulationObject] = {}
        self.control_callbacks: Dict[str, ControlCallback] = {}
        
        # Frequency grouping for automatic optimization
        self.frequency_groups: Dict[float, Dict] = {}  # frequency -> {robots: [], callbacks: [], process: None}
        self.robot_frequencies: Dict[str, float] = {}  # robot_name -> frequency
        self._frequency_grouping_enabled = self.config.enable_frequency_grouping
        
        # Runtime state
        self._running = False
        self._shutdown_requested = False
        self._simulation_paused = False
        self._reset_requested = False
        
        # SimPy processes - simplified architecture
        self._visualization_process = None
        self._control_callback_process = None
        
        # Performance tracking
        self._start_time = 0.0
        self._frame_count = 0
        
        # Initialize time management system
        self._initialize_time_management()
        
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
    
    def _initialize_time_management(self):
        """Initialize RealtimeEnvironment-based time management"""
        if self.time_manager is None:
            self.time_manager = TimeManager(
                real_time_factor=self.config.real_time_factor,
                strict=False
            )
            # Set as global time manager for easy access
            set_global_time_manager(self.time_manager)
            print(f"✅ TimeManager initialized with RealtimeEnvironment (factor={self.config.real_time_factor}x)")
    
    @property
    def env(self) -> simpy.rt.RealtimeEnvironment:
        """Get the SimPy RealtimeEnvironment"""
        return self.time_manager.env if self.time_manager else None
    
    def _initialize_visualization(self):
        """Initialize visualization system if enabled"""
        if not self.config.visualization:
            return
            
        if self.visualizer is None:
            # Choose visualizer type based on configuration
            if self.config.use_optimized_visualizer:
                print(f"🚀 Creating optimized PyVista visualizer (level: {self.config.visualization_optimization_level})")
                self.visualizer = create_optimized_urdf_visualizer(
                    interactive=True,
                    window_size=self.config.window_size,
                    optimization_level=self.config.visualization_optimization_level
                )
            else:
                print("📺 Creating standard PyVista visualizer")
                self.visualizer = create_urdf_robot_visualizer(
                    interactive=True,
                    window_size=self.config.window_size
                )
            
            if self.visualizer.available:
                visualizer_type = "Optimized" if self.config.use_optimized_visualizer else "Standard"
                print(f"✅ {visualizer_type} visualization system initialized")
                
                # Print optimization info for optimized visualizer
                if self.config.use_optimized_visualizer and hasattr(self.visualizer, 'get_performance_stats'):
                    print(f"   Optimization level: {self.config.visualization_optimization_level}")
                    print(f"   Batch rendering: {'Enabled' if self.config.enable_batch_rendering else 'Disabled'}")
            else:
                warnings.warn("Visualization system not available, running headless")
                self.config.visualization = False
    
    def add_robot_from_urdf(self, 
                           name: str, 
                           urdf_path: str,
                           initial_pose: Optional[Pose] = None,
                           joint_update_rate: float = 10.0,
                           unified_process: bool = True) -> Robot:
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
        
        # Ensure time management is initialized
        self._initialize_time_management()
        
        try:
            # If frequency grouping is enabled, disable robot's individual processes
            effective_unified_process = unified_process
            if self._frequency_grouping_enabled:
                effective_unified_process = False  # Disable individual robot processes
                
            # Create robot with time manager reference
            robot = create_robot_from_urdf(
                self.env,
                urdf_path,
                name,
                initial_pose,
                joint_update_rate,
                time_manager=self.time_manager,
                unified_process=effective_unified_process,
                frequency_grouping_managed=self._frequency_grouping_enabled
            )
            
            self.robots[name] = robot
            
            # Auto-register for frequency grouping if enabled
            if self._frequency_grouping_enabled:
                self._add_robot_to_frequency_group(name, robot, joint_update_rate)
            
            # Add to visualization if available
            if self.config.visualization:
                self._initialize_visualization()
                if self.visualizer and self.visualizer.available:
                    # Connect visualizer to simulation manager BEFORE loading robots
                    if hasattr(self.visualizer, 'connect_simulation_manager'):
                        self.visualizer.connect_simulation_manager(self)
                    self.visualizer.load_robot(name, robot)  # Updated method signature
            
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
        
        # Auto-add to frequency grouping if enabled
        if self._frequency_grouping_enabled:
            self._add_simulation_object_to_frequency_group(name, obj)
        
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
        
        if self._frequency_grouping_enabled:
            # Add callback to frequency group
            self._add_callback_to_frequency_group(name, callback, frequency)
        else:
            # Traditional individual callback
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
    
    def _control_callback_process_loop(self):
        """Independent process for user control callbacks"""
        callback_dt = 1.0 / 10.0  # 10 Hz for control callbacks, sufficient responsiveness
        
        while self._running and not self._shutdown_requested:
            # Check if simulation is paused
            if self._simulation_paused:
                yield self.env.timeout(callback_dt)
                continue
            
            # Get current simulation time from time manager
            current_sim_time = self.time_manager.get_sim_time()
            
            # Process control callbacks
            for robot_name, callback in self.control_callbacks.items():
                if callback.should_call(current_sim_time):
                    try:
                        callback.call(callback_dt, current_sim_time)
                    except Exception as e:
                        print(f"⚠️ Control callback error for {robot_name}: {e}")
            
            self._frame_count += 1
            
            # Yield control
            yield self.env.timeout(callback_dt)
    
    def _visualization_process_loop(self):
        """Visualization update process loop - SimPy pure environment with optimization support"""
        if not self.visualizer or not self.visualizer.available:
            return
            
        dt = 1.0 / self.config.visualization_update_rate
        
        while self._running and not self._shutdown_requested:
            
            # Use batch rendering for optimized visualizer
            if (self.config.use_optimized_visualizer and 
                self.config.enable_batch_rendering and 
                hasattr(self.visualizer, 'batch_mode') and 
                len(self.robots) > 1):
                
                # Batch update all robots
                try:
                    with self.visualizer.batch_mode():
                        for robot_name, robot in self.robots.items():
                            try:
                                self.visualizer.update_robot_visualization(robot_name, force_render=False)
                            except Exception as e:
                                print(f"⚠️ Robot visualization update error for {robot_name}: {e}")
                        
                        # Update simulation objects in batch
                        for object_name, obj in self.objects.items():
                            try:
                                self._update_object_visualization(object_name, obj)
                            except Exception as e:
                                print(f"⚠️ Object visualization update error for {object_name}: {e}")
                    # Rendering happens automatically when exiting batch_mode context
                    
                except Exception as e:
                    print(f"⚠️ Batch visualization update error: {e}")
                    
            else:
                # Standard individual updates
                for robot_name, robot in self.robots.items():
                    try:
                        self.visualizer.update_robot_visualization(robot_name)
                    except Exception as e:
                        print(f"⚠️ Robot visualization update error for {robot_name}: {e}")
                
                # Update simulation object visualizations
                for object_name, obj in self.objects.items():
                    try:
                        self._update_object_visualization(object_name, obj)
                    except Exception as e:
                        print(f"⚠️ Object visualization update error for {object_name}: {e}")
            
            # Yield control back to SimPy
            yield self.env.timeout(dt)
    
    def _duration_monitor(self, duration: float):
        """Monitor simulation duration and close visualization when time expires"""
        # For RealtimeEnvironment, we want to wait for simulation time duration
        # The RealtimeEnvironment will automatically adjust wall time based on real_time_factor
        yield self.env.timeout(duration)
        print(f"\n⏰ Duration {duration}s completed - closing visualization")
        self._shutdown_requested = True
        if self.visualizer and hasattr(self.visualizer, 'plotter') and self.visualizer.plotter:
            try:
                self.visualizer.plotter.close()
            except:
                pass
    
    def _update_object_visualization(self, object_name: str, obj: SimulationObject):
        """Update visualization for a simulation object (box, sphere, etc.)"""
        if not self.visualizer or not hasattr(self.visualizer, 'plotter'):
            return
        
        try:
            # Check if object has an associated mesh actor
            if hasattr(obj, '_mesh_actor') and obj._mesh_actor is not None:
                # Update transformation matrix
                transform_matrix = obj.pose.to_transformation_matrix()
                
                # Set transformation on the mesh actor
                try:
                    vtk_matrix = self.visualizer.pv.vtk.vtkMatrix4x4()
                    for i in range(4):
                        for j in range(4):
                            vtk_matrix.SetElement(i, j, transform_matrix[i, j])
                    obj._mesh_actor.SetUserMatrix(vtk_matrix)
                except Exception as e:
                    print(f"⚠️ Failed to update {object_name} transformation: {e}")
            else:
                # Object not yet visualized, create mesh if needed
                self._create_object_mesh(object_name, obj)
                
        except Exception as e:
            print(f"⚠️ Failed to update visualization for {object_name}: {e}")
    
    def _create_object_mesh(self, object_name: str, obj: SimulationObject):
        """Create mesh representation for simulation object"""
        if not self.visualizer or not hasattr(self.visualizer, 'plotter'):
            return
        
        try:
            mesh = None
            # Create mesh based on object parameters
            if hasattr(obj.parameters, 'geometry_type'):
                geometry_type = obj.parameters.geometry_type
                params = getattr(obj.parameters, 'geometry_params', {})
                
                if geometry_type == "box":
                    size = params.get('size', [0.3, 0.3, 0.3])
                    mesh = self.visualizer.pv.Cube(x_length=size[0], y_length=size[1], z_length=size[2])
                elif geometry_type == "cylinder":
                    radius = params.get('radius', 0.05)
                    height = params.get('height', 0.2)
                    mesh = self.visualizer.pv.Cylinder(radius=radius, height=height, direction=(0, 0, 1))
                elif geometry_type == "sphere":
                    radius = params.get('radius', 0.1)
                    mesh = self.visualizer.pv.Sphere(radius=radius)
            
            if mesh is not None:
                # Apply current transformation
                transform_matrix = obj.pose.to_transformation_matrix()
                mesh.transform(transform_matrix, inplace=True)
                
                # Get color
                color = getattr(obj.parameters, 'color', (0.6, 0.6, 0.6))
                if len(color) > 3:
                    color = color[:3]  # RGB only for PyVista
                
                # Add to scene and store reference
                actor = self.visualizer.plotter.add_mesh(
                    mesh, 
                    color=color, 
                    opacity=0.8, 
                    name=f"object_{object_name}"
                )
                obj._mesh_actor = actor
                print(f"✅ Created visualization for {object_name}: {geometry_type}")
                
        except Exception as e:
            print(f"⚠️ Failed to create mesh for {object_name}: {e}")
    
    def run(self, duration: Optional[float] = None, auto_close: bool = False) -> bool:
        """
        Run simulation until Ctrl+C, window close, or duration expires
        
        Args:
            duration: Optional simulation duration in seconds
            auto_close: If True, automatically close visualization when duration expires
            
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
        if self.config.real_time_factor == 0.0:
            print("   Real time factor: MAX SPEED (0.0)")
        else:
            print(f"   Real time factor: {self.config.real_time_factor}x")
        print(f"   Visualization: {'On' if self.config.visualization else 'Off'}")
        print("   Press Ctrl+C to stop")
        print("=" * 50)
        
        self._running = True
        self._start_time = time.time()
        self._real_time_start = time.time()
        self._sim_time = 0.0
        
        # Start control processes
        if self._frequency_grouping_enabled and self.frequency_groups:
            # Start frequency-grouped processes
            self._start_frequency_grouped_processes()
        elif self.control_callbacks:
            # Start traditional control callback process
            self._control_callback_process = self.env.process(self._control_callback_process_loop())
        
        # Start time manager simulation
        self.time_manager.start_simulation()
        
        # Start visualization process
        if self.config.visualization and self.visualizer and self.visualizer.available:
            self._visualization_process = self.env.process(self._visualization_process_loop())
            
            # Connect visualizer to simulation manager for real-time factor control
            if hasattr(self.visualizer, 'connect_simulation_manager'):
                self.visualizer.connect_simulation_manager(self)
        
        try:
            if self.config.visualization and self.visualizer and self.visualizer.available:
                # Visualization mode: start window first, then run simulation with real-time updates
                try:
                    # Start PyVista window in non-blocking mode
                    self.visualizer.plotter.show(
                        auto_close=False, 
                        interactive_update=True
                    )
                    
                    # Run simulation with real-time visualization updates
                    simulation_end_time = None
                    if duration:
                        # Set end time
                        simulation_end_time = self.env.now + duration
                        if auto_close:
                            print(f"🕐 Simulation will run for {duration}s, then auto-close")
                        else:
                            print(f"🕐 Simulation will run for {duration}s, but window stays open")
                    
                    while not self._shutdown_requested:
                        # Check if simulation time has ended
                        if simulation_end_time and self.env.now >= simulation_end_time:
                            # Simulation finished
                            if not hasattr(self, '_simulation_ended'):
                                print(f"\n⏰ Simulation time ({duration}s) completed")
                                self._simulation_ended = True
                                # Stop the simulation processes
                                self._running = False  # This will stop simulation callbacks
                                
                                if auto_close:
                                    # Auto-close mode: gracefully end visualization
                                    print("🔄 Auto-closing for next example...")
                                    self._shutdown_requested = True
                                    # Give a moment for the message to display
                                    time.sleep(0.1)
                                    break
                                else:
                                    # Manual mode: keep visualization running
                                    print("💡 Visualization continues - close window or press Ctrl+C to exit")
                        else:
                            # Run simulation step - let RealtimeEnvironment handle proper timing
                            try:
                                # Much smaller increment to allow callbacks to execute properly
                                step_size = 1.0 / self.config.update_rate  # Use update rate for step size
                                target_time = min(self.env.now + step_size, simulation_end_time or self.env.now + step_size)
                                self.env.run(until=target_time)
                            except simpy.core.EmptySchedule:
                                # All processes finished
                                break
                        
                        # Update visualization at reasonable rate (not every simulation step)
                        try:
                            self.visualizer.plotter.update()
                            # No sleep - let RealtimeEnvironment handle timing
                        except:
                            # Window closed by user
                            break
                                
                except KeyboardInterrupt:
                    print("\n🛑 Visualization interrupted")
                    self._shutdown_requested = True
            else:
                # Headless mode: let RealtimeEnvironment handle timing properly
                if duration:
                    # Start duration monitor process for headless mode
                    duration_process = self.env.process(self._duration_monitor(duration))
                    print(f"🕐 Headless simulation will run for {duration}s (real-time factor: {self.config.real_time_factor}x)")
                    
                    # Run SimPy environment until duration expires - let RealtimeEnvironment handle timing
                    try:
                        # Run until duration monitor finishes
                        self.env.run(until=self.env.now + duration + 0.1)  # Allow slight buffer
                    except simpy.core.EmptySchedule:
                        # All processes finished naturally
                        print(f"\n⏰ Headless simulation completed naturally")
                        
                    print(f"\n⏰ Headless simulation duration ({duration}s) completed")
                else:
                    try:
                        # Infinite headless simulation
                        self.env.run()
                    except KeyboardInterrupt:
                        print("\n⏹️ Simulation stopped by user")
                        
        except KeyboardInterrupt:
            print("\n🛑 Interrupted by user")
            self._shutdown_requested = True
        finally:
            # Print timing statistics before shutdown
            self._print_timing_stats()
            self.shutdown()
        
        return True
    
    def set_realtime_factor(self, factor: float):
        """
        Update real-time factor during simulation
        
        Args:
            factor: New real-time factor (0.1 to 10.0)
        """
        old_factor = self.config.real_time_factor
        self.config.real_time_factor = factor
        
        # Update time manager
        if self.time_manager:
            self.time_manager.set_real_time_factor(factor)
        
        print(f"⏱️ SimulationManager real-time factor: {old_factor:.2f}x → {factor:.2f}x")
        
        # Sync to visualizer if available
        if hasattr(self, 'visualizer') and self.visualizer and hasattr(self.visualizer, 'realtime_factor'):
            if abs(self.visualizer.realtime_factor - factor) > 0.01:
                self.visualizer.realtime_factor = factor
                print(f"🔄 Synchronized visualizer real-time factor: {factor:.2f}x")
    
    def get_realtime_factor(self) -> float:
        """Get current real-time factor"""
        if self.time_manager:
            return self.time_manager.get_real_time_factor()
        return self.config.real_time_factor
    
    # Centralized time management methods (memo.txt item 10)
    def get_sim_time(self) -> float:
        """Get current simulation time from time manager"""
        return self.time_manager.get_sim_time() if self.time_manager else 0.0
    
    def get_real_time(self) -> float:
        """Get real time elapsed since simulation start"""
        return self.time_manager.get_real_time_elapsed() if self.time_manager else 0.0
    
    def get_time_step(self) -> float:
        """Get current simulation time step"""
        return self.config.time_step
    
    def set_time_step(self, time_step: float):
        """Set simulation time step"""
        if time_step > 0:
            self.config.time_step = time_step
            print(f"⏱️ Time step updated: {time_step:.4f}s")
    
    def get_timing_stats(self) -> dict:
        """Get real-time synchronization statistics from time manager"""
        if self.time_manager:
            stats = self.time_manager.get_timing_stats()
            return {
                'sim_time': stats.sim_time,
                'real_time_elapsed': stats.real_time_elapsed,
                'target_speed': stats.target_speed,
                'actual_speed': stats.actual_speed,
                'real_time_factor': stats.real_time_factor
            }
        return {'accuracy': 'N/A'}
    
    def pause_simulation(self):
        """Pause the simulation"""
        self._simulation_paused = True
        
        # Also stop all robot base velocities when paused
        for robot_name, robot in self.robots.items():
            try:
                # Stop robot base motion using correct Velocity constructor arguments
                from core.simulation_object import Velocity
                robot.set_velocity(Velocity(
                    linear_x=0.0, linear_y=0.0, linear_z=0.0,
                    angular_x=0.0, angular_y=0.0, angular_z=0.0
                ))
                print(f"⏸️ Stopped base velocity for robot '{robot_name}'")
            except Exception as e:
                print(f"⚠️ Error stopping robot '{robot_name}' base velocity: {e}")
        
        print("⏸️ Simulation paused by user")
    
    def resume_simulation(self):
        """Resume the simulation"""
        self._simulation_paused = False
        print("▶️ Simulation resumed by user")
    
    def clear_all_robots(self):
        """Remove all robots from simulation"""
        try:
            # Clear control callbacks
            self.control_callbacks.clear()
            
            # Clear from visualizer
            if self.visualizer and hasattr(self.visualizer, 'robots'):
                for robot_name in list(self.robots.keys()):
                    if robot_name in self.visualizer.robots:
                        del self.visualizer.robots[robot_name]
                    if robot_name in self.visualizer.link_actors:
                        # Remove actors from plotter safely
                        link_actors = self.visualizer.link_actors[robot_name]
                        for link_name, actor_info in link_actors.items():
                            try:
                                if self.visualizer.plotter:
                                    self.visualizer.plotter.remove_actor(f"{robot_name}_{link_name}")
                            except Exception as e:
                                # Silently continue if actor removal fails
                                pass
                        del self.visualizer.link_actors[robot_name]
                    if robot_name in self.visualizer.urdf_loaders:
                        del self.visualizer.urdf_loaders[robot_name]
                    if robot_name in self.visualizer.animation_controllers:
                        del self.visualizer.animation_controllers[robot_name]
            
            # Clear robots and objects
            self.robots.clear()
            self.objects.clear()
            
            print("🗑️ All robots and objects cleared from simulation")
            
        except Exception as e:
            print(f"⚠️ Error clearing robots: {e}")
    
    def reset_simulation(self):
        """Reset simulation to initial state"""
        self._reset_requested = True
        print("🔄 Simulation reset requested")
        
        # Clear all robots and objects for fresh start
        self.clear_all_robots()
        
        # Reset simulation state flags
        self._running = False
        self._shutdown_requested = False
        if hasattr(self, '_simulation_ended'):
            delattr(self, '_simulation_ended')
        
        # Reinitialize visualizer if needed
        if self.config.visualization and (not self.visualizer or not self.visualizer.plotter):
            try:
                self._initialize_visualization()
            except Exception as e:
                print(f"⚠️ Visualization reinitialization warning: {e}")
        
        # Reset simulation time
        self._frame_count = 0
        self._start_time = time.time()
        self._real_time_start = time.time()
        self._sim_time = 0.0
        
        # Reset timing statistics
        self._timing_stats = {
            'target_real_time': 0.0,
            'actual_real_time': 0.0,
            'processing_times': [],
            'sleep_times': []
        }
        
        self._reset_requested = False
    
    def is_paused(self) -> bool:
        """Check if simulation is paused"""
        return self._simulation_paused
    
    def shutdown(self):
        """Graceful shutdown of simulation"""
        if not self._running and not self._shutdown_requested:
            return
            
        print("🛑 Shutting down simulation...")
        self._shutdown_requested = True
        self._running = False
        
        # Shutdown all robot processes
        for robot_name, robot in self.robots.items():
            try:
                robot.shutdown_processes()
            except Exception as e:
                print(f"⚠️ Error shutting down robot '{robot_name}': {e}")
        
        # Shutdown all object processes
        for object_name, obj in self.objects.items():
            try:
                obj.stop_motion_process()
            except Exception as e:
                print(f"⚠️ Error shutting down object '{object_name}': {e}")
        
        # Clean up visualization safely
        if self.visualizer:
            try:
                if hasattr(self.visualizer, 'plotter') and self.visualizer.plotter:
                    self.visualizer.plotter.close()
                    self.visualizer.plotter = None
            except Exception as e:
                print(f"⚠️ Visualization cleanup warning: {e}")
            finally:
                self.visualizer = None
        
        # SimPy processes are automatically cleaned up when environment shuts down
        if self._control_callback_process:
            try:
                self._control_callback_process.interrupt()
            except Exception:
                pass
        
        if self._visualization_process:
            try:
                self._visualization_process.interrupt()
            except Exception:
                pass
        
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
        
        info = {
            'running': self._running,
            'robots': list(self.robots.keys()),
            'objects': list(self.objects.keys()),
            'elapsed_time': elapsed,
            'frame_count': self._frame_count,
            'average_fps': avg_fps,
            'config': self.config
        }
        
        # Add frequency grouping stats if enabled
        if self._frequency_grouping_enabled and self.frequency_groups:
            info['frequency_grouping'] = {
                'enabled': True,
                'groups': len(self.frequency_groups),
                'frequencies': list(self.frequency_groups.keys()),
                'total_robots': sum(len(group['robots']) for group in self.frequency_groups.values()),
                'process_reduction_percent': self._calculate_process_reduction_percent()
            }
        else:
            info['frequency_grouping'] = {'enabled': False}
        
        return info
    
    # ===============================
    # Frequency Grouping Methods
    # ===============================
    
    def _add_robot_to_frequency_group(self, name: str, robot: Robot, frequency: float):
        """Add robot to appropriate frequency group"""
        
        if frequency not in self.frequency_groups:
            self.frequency_groups[frequency] = {
                'robots': [],
                'objects': [],
                'callbacks': [],
                'robot_updates': [],
                'object_updates': [],
                'process': None,
                'callback_count': 0,
                'update_count': 0
            }
        
        # Add robot info to group
        robot_info = {
            'name': name,
            'instance': robot,
            'frequency': frequency,
            'type': 'robot'
        }
        
        # Add robot update function to frequency group
        robot_update_info = {
            'name': name,
            'type': 'robot_update',
            'instance': robot,
            'update_function': self._create_robot_update_wrapper(robot),
            'call_count': 0
        }
        
        self.frequency_groups[frequency]['robots'].append(robot_info)
        self.frequency_groups[frequency]['robot_updates'].append(robot_update_info)
        self.robot_frequencies[name] = frequency
        
        print(f"🔄 Added robot '{name}' to {frequency} Hz frequency group "
              f"({len(self.frequency_groups[frequency]['robots'])} robots in group)")
        print(f"   ↳ Robot internal updates will be grouped at {frequency} Hz")
    
    def _add_callback_to_frequency_group(self, name: str, callback: Callable, frequency: float):
        """Add control callback to appropriate frequency group"""
        
        # Use robot's registered frequency if different from callback frequency
        robot_frequency = self.robot_frequencies.get(name, frequency)
        
        if robot_frequency != frequency:
            print(f"⚠️ Robot '{name}' callback frequency ({frequency} Hz) differs from "
                  f"robot joint_update_rate ({robot_frequency} Hz). Using robot frequency.")
            frequency = robot_frequency
        
        if frequency not in self.frequency_groups:
            # This shouldn't happen if robot was added first, but handle gracefully
            self.frequency_groups[frequency] = {
                'robots': [],
                'callbacks': [],
                'process': None,
                'callback_count': 0
            }
        
        # Add callback info to group
        callback_info = {
            'robot_name': name,
            'callback': callback,
            'call_count': 0
        }
        
        self.frequency_groups[frequency]['callbacks'].append(callback_info)
        print(f"🎮 Added callback for '{name}' to {frequency} Hz frequency group")
    
    def _add_simulation_object_to_frequency_group(self, name: str, obj: SimulationObject):
        """Add simulation object to appropriate frequency group based on its update rate"""
        
        # Get object's update frequency
        update_interval = getattr(obj, 'update_interval', 1.0)  # Default 1s interval
        frequency = 1.0 / update_interval  # Convert interval to frequency
        
        if frequency not in self.frequency_groups:
            self.frequency_groups[frequency] = {
                'robots': [],
                'objects': [],
                'callbacks': [],
                'robot_updates': [],
                'object_updates': [],
                'process': None,
                'callback_count': 0,
                'update_count': 0
            }
        
        # Add object info to group
        object_info = {
            'name': name,
            'instance': obj,
            'frequency': frequency,
            'type': 'simulation_object'
        }
        
        # Add object update function to frequency group
        object_update_info = {
            'name': name,
            'type': 'object_update',
            'instance': obj,
            'update_function': self._create_object_update_wrapper(obj),
            'call_count': 0
        }
        
        self.frequency_groups[frequency]['objects'].append(object_info)
        self.frequency_groups[frequency]['object_updates'].append(object_update_info)
        
        print(f"🌍 Added object '{name}' to {frequency} Hz frequency group "
              f"({len(self.frequency_groups[frequency]['objects'])} objects in group)")
        print(f"   ↳ Object internal updates will be grouped at {frequency} Hz")
    
    def _create_robot_update_wrapper(self, robot: Robot):
        """Create wrapper for robot internal updates"""
        def robot_update_wrapper(dt: float):
            try:
                # Call robot's internal update methods
                current_sim_time = self.get_sim_time()
                
                # Check if robot has update_joints_if_needed method (centralized architecture)
                if hasattr(robot, 'update_joints_if_needed'):
                    robot.update_joints_if_needed(current_sim_time)
                
                # Check if robot has update_if_needed method (base motion)
                if hasattr(robot, 'update_if_needed'):
                    robot.update_if_needed(current_sim_time)
                
                # Check if robot has _update_forward_kinematics method
                if hasattr(robot, '_update_forward_kinematics'):
                    robot._update_forward_kinematics()
                    
            except Exception as e:
                print(f"❌ Robot update error for {robot.name}: {e}")
        
        return robot_update_wrapper
    
    def _create_object_update_wrapper(self, obj: SimulationObject):
        """Create wrapper for simulation object internal updates"""
        def object_update_wrapper(dt: float):
            try:
                current_sim_time = self.get_sim_time()
                
                # Check if object has update_if_needed method
                if hasattr(obj, 'update_if_needed'):
                    obj.update_if_needed(current_sim_time)
                
                # Check if object has update method
                elif hasattr(obj, 'update'):
                    obj.update(dt)
                    
            except Exception as e:
                print(f"❌ Object update error for {obj.name if hasattr(obj, 'name') else 'unnamed'}: {e}")
        
        return object_update_wrapper
    
    def _start_frequency_grouped_processes(self):
        """Start all frequency-grouped processes"""
        
        print(f"🚀 Starting frequency-grouped processes...")
        
        active_frequencies = sorted(self.frequency_groups.keys())
        total_robots = sum(len(group['robots']) for group in self.frequency_groups.values())
        
        print(f"Process optimization:")
        print(f"   Traditional approach: {total_robots} processes (1 per robot)")
        print(f"   Frequency-grouped: {len(active_frequencies)} processes (1 per frequency)")
        reduction_percent = self._calculate_process_reduction_percent()
        print(f"   Process reduction: {reduction_percent:.1f}%")
        
        print(f"Frequency distribution:")
        for frequency in active_frequencies:
            group = self.frequency_groups[frequency]
            robot_count = len(group['robots'])
            object_count = len(group['objects'])
            callback_count = len(group['callbacks'])
            robot_update_count = len(group['robot_updates'])
            object_update_count = len(group['object_updates'])
            
            total_items = robot_count + object_count + callback_count
            internal_updates = robot_update_count + object_update_count
            
            print(f"   {frequency:6.1f} Hz: {robot_count:3d} robots, {object_count:2d} objects, "
                  f"{callback_count:3d} callbacks, {internal_updates:3d} internal updates")
            
            # Start process for this frequency
            process = self.env.process(self._frequency_group_process_loop(frequency))
            group['process'] = process
        
        print(f"✅ Started {len(active_frequencies)} frequency-grouped processes")
    
    def _frequency_group_process_loop(self, frequency: float):
        """Process loop for a specific frequency group - handles all updates at this frequency"""
        
        group = self.frequency_groups[frequency]
        dt = 1.0 / frequency
        
        print(f"🔄 Starting {frequency} Hz unified frequency group process")
        print(f"   Processing: {len(group['callbacks'])} callbacks, "
              f"{len(group['robot_updates'])} robot updates, "
              f"{len(group['object_updates'])} object updates")
        
        while self._running and not self._shutdown_requested:
            try:
                if not self._simulation_paused:
                    # 1. Execute all robot internal updates in this frequency group
                    for robot_update_info in group['robot_updates']:
                        try:
                            robot_update_info['update_function'](dt)
                            robot_update_info['call_count'] += 1
                            group['update_count'] += 1
                        except Exception as e:
                            print(f"❌ Robot update error for {robot_update_info['name']}: {e}")
                    
                    # 2. Execute all object internal updates in this frequency group  
                    for object_update_info in group['object_updates']:
                        try:
                            object_update_info['update_function'](dt)
                            object_update_info['call_count'] += 1
                            group['update_count'] += 1
                        except Exception as e:
                            print(f"❌ Object update error for {object_update_info['name']}: {e}")
                    
                    # 3. Execute all user control callbacks in this frequency group
                    for callback_info in group['callbacks']:
                        try:
                            callback_info['callback'](dt)
                            callback_info['call_count'] += 1
                            group['callback_count'] += 1
                        except Exception as e:
                            print(f"❌ Callback error for {callback_info['robot_name']}: {e}")
                
                # Single yield for entire frequency group - this is the revolutionary optimization!
                # All robot updates, object updates, and callbacks processed in ONE yield
                yield self.env.timeout(dt)
                
            except Exception as e:
                print(f"❌ Frequency group {frequency} Hz process error: {e}")
                break
    
    def _calculate_process_reduction_percent(self) -> float:
        """Calculate process reduction percentage from frequency grouping"""
        
        if not self.frequency_groups:
            return 0.0
        
        # Count total entities that would traditionally need separate processes
        total_robots = sum(len(group['robots']) for group in self.frequency_groups.values())
        total_objects = sum(len(group['objects']) for group in self.frequency_groups.values())
        
        # In traditional architecture:
        # - Each robot would need unified_process (1 process) OR separate processes for joint/sensor/base/nav (4 processes)
        # - Each object would need 1 process
        # For simplicity, assume 1 process per robot + 1 per object in traditional approach
        traditional_processes = total_robots + total_objects
        
        # Frequency-grouped approach uses only 1 process per unique frequency
        frequency_grouped_processes = len(self.frequency_groups)
        
        if traditional_processes == 0:
            return 0.0
        
        return (1 - frequency_grouped_processes / traditional_processes) * 100
    
    def get_frequency_grouping_stats(self) -> Dict[str, Any]:
        """Get detailed frequency grouping statistics"""
        
        if not self._frequency_grouping_enabled:
            return {'enabled': False}
        
        total_callbacks = sum(group['callback_count'] for group in self.frequency_groups.values())
        total_updates = sum(group['update_count'] for group in self.frequency_groups.values())
        total_robots = sum(len(group['robots']) for group in self.frequency_groups.values())
        total_objects = sum(len(group['objects']) for group in self.frequency_groups.values())
        
        stats = {
            'enabled': True,
            'total_groups': len(self.frequency_groups),
            'total_robots': total_robots,
            'total_objects': total_objects,
            'total_callbacks': total_callbacks,
            'total_internal_updates': total_updates,
            'process_reduction_percent': self._calculate_process_reduction_percent(),
            'groups': {}
        }
        
        for frequency, group in self.frequency_groups.items():
            stats['groups'][frequency] = {
                'frequency': frequency,
                'robot_count': len(group['robots']),
                'object_count': len(group['objects']),
                'callback_count': len(group['callbacks']),
                'robot_update_count': len(group['robot_updates']),
                'object_update_count': len(group['object_updates']),
                'total_callback_calls': group['callback_count'],
                'total_update_calls': group['update_count'],
                'robots': [robot['name'] for robot in group['robots']],
                'objects': [obj['name'] for obj in group['objects']]
            }
        
        return stats
    
    def get_sim_time(self) -> float:
        """Get current simulation time"""
        return self.time_manager.get_sim_time() if self.time_manager else 0.0
    
    def _print_timing_stats(self):
        """Print detailed timing statistics"""
        if not self.time_manager:
            return
            
        try:
            stats = self.time_manager.get_timing_stats()
            
            print(f"\n📊 Timing Performance Statistics:")
            print(f"   Simulation time: {stats.sim_time:.2f}s")
            print(f"   Real time elapsed: {stats.real_time_elapsed:.2f}s")
            print(f"   Target factor: {stats.real_time_factor:.2f}x")
            print(f"   Actual factor: {stats.actual_speed}")
            
            # Calculate error percentage
            if stats.real_time_elapsed > 0 and stats.sim_time > 0:
                actual_factor = stats.sim_time / stats.real_time_elapsed
                error_percent = abs(actual_factor - stats.real_time_factor) / stats.real_time_factor * 100
                
                if error_percent < 5.0:
                    status = "✅ EXCELLENT"
                elif error_percent < 10.0:
                    status = "✅ GOOD" 
                elif error_percent < 20.0:
                    status = "⚠️ FAIR"
                else:
                    status = "❌ POOR"
                    
                print(f"   Timing accuracy: {status} ({error_percent:.1f}% error)")
            else:
                print(f"   Timing accuracy: N/A (insufficient data)")
                
        except Exception as e:
            print(f"⚠️ Could not get timing stats: {e}")


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