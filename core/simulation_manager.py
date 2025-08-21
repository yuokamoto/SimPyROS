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

# Import logging configuration
from core.logger import get_logger, log_success, log_warning, log_error, log_debug, log_info

from core.robot import Robot, create_robot_from_urdf, RobotParameters
from core.simulation_object import SimulationObject, ObjectParameters, Pose, Velocity
from core.pyvista_visualizer import URDFRobotVisualizer, create_urdf_robot_visualizer
# Removed unused import: from core.unified_pyvista_visualizer import UnifiedPyVistaVisualizer, create_unified_visualizer
from core.time_manager import TimeManager, set_global_time_manager
from core.process_separated_urdf_visualizer import create_process_separated_urdf_visualizer
from core.simulation_monitor import create_simulation_monitor
from core.multiprocessing_cleanup import cleanup_multiprocessing_resources


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
    
    # Visualization backend settings
    visualization_backend: str = 'process_separated_pyvista'  # 'pyvista', 'process_separated_pyvista'
    # Removed deprecated use_optimized_visualizer option - use visualization_backend instead
    visualization_optimization_level: str = 'balanced'  # 'performance', 'balanced', 'quality'
    enable_batch_rendering: bool = True  # Enable batch rendering for multiple robots
    
    
    # Process-separated PyVista settings
    max_robots: int = 5  # Maximum number of robots for shared memory (デフォルト値を小さく)
    max_links_per_robot: int = 15  # Maximum links per robot for shared memory
    
    # Monitor window settings
    enable_monitor: bool = True  # Enable simulation monitor window (with improved X11 error handling)
    monitor_enable_controls: bool = False  # Enable control buttons in monitor
    
    
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
        
        # Setup logging
        self.logger = get_logger(f'simpyros.simulation')
        
        # Core components - use RealtimeEnvironment
        self.time_manager: Optional[TimeManager] = None
        self.visualizer: Optional[URDFRobotVisualizer] = None
        self.monitor = None  # Simulation monitor window
        
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
            log_warning(self.logger, "\n🛑 Shutdown requested...")
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
            log_success(self.logger, f"TimeManager initialized with RealtimeEnvironment (factor={self.config.real_time_factor}x)")
    
    @property
    def env(self) -> simpy.rt.RealtimeEnvironment:
        """Get the SimPy RealtimeEnvironment"""
        return self.time_manager.env if self.time_manager else None
    
    def _initialize_monitor(self):
        """Initialize simulation monitor window if enabled with error handling"""
        if not self.config.enable_monitor or self.monitor is not None:
            return
            
        try:
            log_info(self.logger, "Creating simulation monitor window")
            backend_name = self.config.visualization_backend if self.config.visualization else 'headless'
            self.monitor = create_simulation_monitor(
                title=f"SimPyROS Monitor - {backend_name}",
                enable_controls=self.config.monitor_enable_controls,
                control_callback=self._handle_monitor_control if self.config.monitor_enable_controls else None
            )
            
            # Give the monitor a moment to initialize
            import time
            time.sleep(0.1)
            
            if self.monitor and hasattr(self.monitor, 'running') and self.monitor.running:
                log_success(self.logger, "Simulation monitor initialized successfully")
            else:
                log_warning(self.logger, "Monitor initialization may have failed")
                
        except Exception as e:
            log_warning(self.logger, f"Failed to create monitor window: {e}")
            log_info(self.logger, "Continuing simulation without monitor...")
            self.monitor = None
    
    def _initialize_visualization(self):
        """Initialize visualization system if enabled"""
        if not self.config.visualization:
            return
            
        if self.visualizer is None:
            # Choose visualizer backend based on configuration
            backend = self.config.visualization_backend.lower()
            
            # Legacy configuration handling removed - use visualization_backend directly
            
            log_info(self.logger, f"Initializing visualization backend: {backend}")
            
            try:
                if backend == 'process_separated_pyvista':
                    log_info(self.logger, "Creating process-separated PyVista visualizer")
                    from core.process_separated_pyvista import SharedMemoryConfig
                    config = SharedMemoryConfig(
                        max_robots=self.config.max_robots,
                        max_links_per_robot=self.config.max_links_per_robot,
                        update_frequency=self.config.visualization_update_rate
                    )
                    self.visualizer = create_process_separated_urdf_visualizer(config=config)
                    
                    
                # Deprecated optimized_pyvista backend removed - use process_separated_pyvista
                    
                elif backend == 'pyvista':
                    log_info(self.logger, "Creating standard PyVista visualizer")
                    self.visualizer = create_urdf_robot_visualizer(
                        interactive=True,
                        window_size=self.config.window_size
                    )
                    
                else:
                    raise ValueError(f"Unknown visualization backend: {backend}")
                
                # Connect time manager and simulation manager to visualizer
                if hasattr(self.visualizer, 'connect_time_manager') and self.time_manager:
                    self.visualizer.connect_time_manager(self.time_manager)
                    
                if hasattr(self.visualizer, 'connect_simulation_manager'):
                    self.visualizer.connect_simulation_manager(self)
                
                # Check availability and setup
                if self.visualizer.available:
                    log_success(self.logger, f"{backend} visualization system initialized")
                    
                    # Print backend-specific info
                    if backend == 'process_separated_pyvista':
                        log_debug(self.logger, "Process separation: ENABLED")
                        log_debug(self.logger, f"Shared memory: {self.config.max_robots} robots, {self.config.max_links_per_robot} links/robot")
                        log_debug(self.logger, "Crash isolation: PyVista crashes don't affect simulation")
                        log_debug(self.logger, "Non-blocking updates: SimPy performance maintained")
                        
                        
                    elif backend == 'pyvista':
                        log_debug(self.logger, "Standard PyVista rendering")
                else:
                    warnings.warn(f"{backend} visualization system not available, running headless")
                    self.config.visualization = False
                    
            except Exception as e:
                log_error(self.logger, f"Failed to initialize {backend} visualizer: {e}")
                warnings.warn(f"Visualization backend '{backend}' failed, running headless")
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
            
            log_success(self.logger, f"Robot '{name}' added from {urdf_path}")
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
        log_success(self.logger, f"Robot '{name}' added")
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
        
        log_success(self.logger, f"Object '{name}' added")
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
        
        log_success(self.logger, f"Control callback set for robot '{name}' at {frequency} Hz")
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
            
            # Periodic speed limit enforcement
            if self.time_manager:
                self.time_manager.check_and_enforce_speed_limit()
            
            # Process control callbacks
            for robot_name, callback in self.control_callbacks.items():
                if callback.should_call(current_sim_time):
                    try:
                        callback.call(callback_dt, current_sim_time)
                    except Exception as e:
                        log_warning(self.logger, f"Control callback error for {robot_name}: {e}")
            
            self._frame_count += 1
            
            # Yield control
            yield self.env.timeout(callback_dt)
    
    def _visualization_process_loop(self):
        """Visualization update process loop - SimPy pure environment with optimization support"""
        if not self.visualizer or not self.visualizer.available:
            return
            
        dt = 1.0 / self.config.visualization_update_rate
        
        while self._running and not self._shutdown_requested:
            
            # Standard visualization updates (deprecated batch rendering removed)
            for robot_name, robot in self.robots.items():
                try:
                    self.visualizer.update_robot_visualization(robot_name)
                except Exception as e:
                    log_warning(self.logger, f"Robot visualization update error for {robot_name}: {e}")
            
            # Update simulation object visualizations
            for object_name, obj in self.objects.items():
                try:
                    self._update_object_visualization(object_name, obj)
                except Exception as e:
                    log_warning(self.logger, f"Object visualization update error for {object_name}: {e}")
            
            # Update monitor window with current data
            self._update_monitor_data()
            
            # Yield control back to SimPy
            yield self.env.timeout(dt)
    
    def _duration_monitor(self, duration: float, duration_mode: str = 'sim_time'):
        """Monitor simulation duration and close visualization when time expires"""
        if duration_mode == 'wall_time':
            # Wall time mode: wait for actual elapsed time regardless of real_time_factor
            start_time = time.time()
            while time.time() - start_time < duration and not self._shutdown_requested:
                # Use much smaller timeout to maintain simulation responsiveness
                yield self.env.timeout(0.01)  # Check every 10ms in simulation time
            log_info(self.logger, f"Wall time duration {duration}s completed - closing visualization")
        else:
            # Simulation time mode (default): wait for simulation time duration
            # The RealtimeEnvironment will automatically adjust wall time based on real_time_factor
            yield self.env.timeout(duration)
            log_info(self.logger, f"Simulation time duration {duration}s completed - closing visualization")
            
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
                    log_warning(self.logger, f"Failed to update {object_name} transformation: {e}")
            else:
                # Object not yet visualized, create mesh if needed
                self._create_object_mesh(object_name, obj)
                
        except Exception as e:
            log_warning(self.logger, f"Failed to update visualization for {object_name}: {e}")
    
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
                log_success(self.logger, f"Created visualization for {object_name}: {geometry_type}")
                
        except Exception as e:
            log_warning(self.logger, f"Failed to create mesh for {object_name}: {e}")
    
    def run(self, duration: Optional[float] = None, auto_close: bool = False, duration_mode: str = 'sim_time') -> bool:
        """
        Run simulation until Ctrl+C, window close, or duration expires
        
        Args:
            duration: Optional duration in seconds
            auto_close: If True, automatically close visualization when duration expires
            duration_mode: 'sim_time' (default) for simulation time, 'wall_time' for real elapsed time
            
        Returns:
            Success status
        """
        if not self.robots and not self.objects:
            log_warning(self.logger, "No robots or objects to simulate")
            return False
        
        log_info(self.logger, "Starting simulation...")
        log_info(self.logger, f"Robots: {len(self.robots)}")
        log_info(self.logger, f"Objects: {len(self.objects)}")
        log_info(self.logger, f"Update rate: {self.config.update_rate} Hz")
        if self.config.real_time_factor == 0.0:
            log_info(self.logger, "Real time factor: MAX SPEED (0.0)")
        else:
            log_info(self.logger, f"Real time factor: {self.config.real_time_factor}x")
        log_info(self.logger, f"Visualization: {'On' if self.config.visualization else 'Off'}")
        log_info(self.logger, "Press Ctrl+C to stop")
        log_info(self.logger, "=" * 50)
        
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
        
        # Initialize monitor window
        self._initialize_monitor()
        
        # Start visualization process
        if self.config.visualization and self.visualizer and self.visualizer.available:
            self._visualization_process = self.env.process(self._visualization_process_loop())
            
            # Connect visualizer to simulation manager for real-time factor control
            if hasattr(self.visualizer, 'connect_simulation_manager'):
                self.visualizer.connect_simulation_manager(self)
        
        try:
            if self.config.visualization and self.visualizer and self.visualizer.available:
                # Check if this is a process-separated visualizer
                is_process_separated = hasattr(self.visualizer, '_run_visualization_process')
                
                # Set up simulation duration
                simulation_end_time = None
                if duration:
                    simulation_end_time = self.env.now + duration
                    if auto_close:
                        log_info(self.logger, f"Simulation will run for {duration}s, then auto-close")
                    else:
                        log_info(self.logger, f"Simulation will run for {duration}s, but window stays open")
                
                if not is_process_separated:
                    # Standard visualizers (PyVista with plotter)
                    try:
                        # Start PyVista window in non-blocking mode
                        self.visualizer.plotter.show(
                            auto_close=False, 
                            interactive_update=True
                        )
                        
                        # Visualization loop for standard visualizers
                        while not self._shutdown_requested:
                            # Check if simulation time has ended
                            if simulation_end_time and self.env.now >= simulation_end_time:
                                self._handle_simulation_end(duration, auto_close)
                                break
                            else:
                                # Run simulation step
                                if not self._run_simulation_step(simulation_end_time):
                                    break
                            
                            # Update visualization
                            try:
                                self.visualizer.plotter.update()
                            except:
                                # Window closed by user
                                break
                    
                    except Exception as e:
                        log_warning(self.logger, f"Visualization error: {e}")
                        # Fall back to headless mode
                        is_process_separated = True
                
                if is_process_separated:
                    # Process-separated or headless mode
                    log_info(self.logger, "Running with process-separated visualization")
                    
                    try:
                        while not self._shutdown_requested:
                            # Check if simulation time has ended
                            if simulation_end_time and self.env.now >= simulation_end_time:
                                self._handle_simulation_end(duration, auto_close)
                                break
                            else:
                                # Run simulation step
                                if not self._run_simulation_step(simulation_end_time):
                                    break
                                    
                    except KeyboardInterrupt:
                        log_info(self.logger, "\nVisualization interrupted")
                        self._shutdown_requested = True
            else:
                # Headless mode: let RealtimeEnvironment handle timing properly
                if duration:
                    # Start duration monitor process for headless mode
                    duration_process = self.env.process(self._duration_monitor(duration, duration_mode))
                    
                    if duration_mode == 'wall_time':
                        log_info(self.logger, f"Headless simulation will run for {duration}s wall time (regardless of RTF {self.config.real_time_factor}x)")
                    else:
                        log_info(self.logger, f"Headless simulation will run for {duration}s simulation time (RTF {self.config.real_time_factor}x → ~{duration/self.config.real_time_factor:.1f}s wall time)")
                    
                    # Run SimPy environment until duration expires - let RealtimeEnvironment handle timing
                    try:
                        if duration_mode == 'wall_time':
                            # For wall time mode, run longer to ensure duration monitor completes
                            self.env.run(until=self.env.now + duration * self.config.real_time_factor + 1.0)
                        else:
                            # For sim time mode, run until duration monitor finishes
                            self.env.run(until=self.env.now + duration + 0.1)  # Allow slight buffer
                    except simpy.core.EmptySchedule:
                        # All processes finished naturally
                        log_info(self.logger, "Headless simulation completed naturally")
                        
                    log_info(self.logger, f"Headless simulation duration ({duration}s {duration_mode}) completed")
                else:
                    try:
                        # Infinite headless simulation
                        self.env.run()
                    except KeyboardInterrupt:
                        log_info(self.logger, "\nSimulation stopped by user")
                        
        except KeyboardInterrupt:
            log_info(self.logger, "\nInterrupted by user")
            self._shutdown_requested = True
        finally:
            # Print timing statistics before shutdown
            self._print_timing_stats()
            self.shutdown()
        
        return True
    
    def _handle_simulation_end(self, duration: float, auto_close: bool):
        """Handle simulation end for both visualization modes"""
        if not hasattr(self, '_simulation_ended'):
            log_info(self.logger, f"Simulation time ({duration}s) completed")
            self._simulation_ended = True
            self._running = False  # Stop simulation callbacks
            
            if auto_close:
                log_info(self.logger, "Auto-closing for next example...")
                self._shutdown_requested = True
                time.sleep(0.1)
            else:
                log_info(self.logger, "Visualization continues - close window or press Ctrl+C to exit")
    
    def _run_simulation_step(self, simulation_end_time: Optional[float]) -> bool:
        """Run a single simulation step, returns False if simulation should end"""
        try:
            step_size = 1.0 / self.config.update_rate
            target_time = min(
                self.env.now + step_size, 
                simulation_end_time or self.env.now + step_size
            )
            self.env.run(until=target_time)
            return True
        except simpy.core.EmptySchedule:
            # All processes finished
            return False
    
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
        
        log_info(self.logger, f"SimulationManager real-time factor: {old_factor:.2f}x → {factor:.2f}x")
        
        # Sync to visualizer if available
        if hasattr(self, 'visualizer') and self.visualizer and hasattr(self.visualizer, 'realtime_factor'):
            if abs(self.visualizer.realtime_factor - factor) > 0.01:
                self.visualizer.realtime_factor = factor
                log_debug(self.logger, f"Synchronized visualizer real-time factor: {factor:.2f}x")
    
    def get_realtime_factor(self) -> float:
        """Get current real-time factor"""
        if self.time_manager:
            return self.time_manager.get_real_time_factor()
        return self.config.real_time_factor
    
    def set_speed_limit_enabled(self, enabled: bool):
        """Enable or disable automatic speed limiting to prevent actual speed from exceeding target"""
        if self.time_manager:
            self.time_manager.set_speed_limit_enabled(enabled)
            log_info(self.logger, f"Speed limiting {'ENABLED' if enabled else 'DISABLED'}")
        else:
            log_warning(self.logger, "TimeManager not available - speed limiting not available")
    
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
            log_info(self.logger, f"Time step updated: {time_step:.4f}s")
    
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
                log_info(self.logger, f"Stopped base velocity for robot '{robot_name}'")
            except Exception as e:
                log_warning(self.logger, f"Error stopping robot '{robot_name}' base velocity: {e}")
        
        log_info(self.logger, "Simulation paused by user")
    
    def resume_simulation(self):
        """Resume the simulation"""
        self._simulation_paused = False
        log_info(self.logger, "Simulation resumed by user")
    
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
            
            log_info(self.logger, "All robots and objects cleared from simulation")
            
        except Exception as e:
            log_warning(self.logger, f"Error clearing robots: {e}")
    
    def reset_simulation(self):
        """Reset simulation to initial state"""
        self._reset_requested = True
        log_info(self.logger, "Simulation reset requested")
        
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
                log_warning(self.logger, f"Visualization reinitialization warning: {e}")
        
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
            
        log_info(self.logger, "Shutting down simulation...")
        self._shutdown_requested = True
        self._running = False
        
        # Shutdown all robot processes
        for robot_name, robot in self.robots.items():
            try:
                robot.shutdown_processes()
            except Exception as e:
                log_warning(self.logger, f"Error shutting down robot '{robot_name}': {e}")
        
        # Shutdown all object processes
        for object_name, obj in self.objects.items():
            try:
                obj.stop_motion_process()
            except Exception as e:
                log_warning(self.logger, f"Error shutting down object '{object_name}': {e}")
        
        # Clean up visualization safely with proper resource management
        if self.visualizer:
            try:
                # Check if this is a process-separated visualizer
                if hasattr(self.visualizer, 'shutdown'):
                    log_info(self.logger, "Shutting down process-separated visualizer...")
                    # Process-separated visualizer has its own shutdown method
                    self.visualizer.shutdown()
                    log_info(self.logger, "Process-separated visualizer shutdown complete")
                elif hasattr(self.visualizer, 'plotter') and self.visualizer.plotter:
                    # Standard PyVista visualizer
                    log_info(self.logger, "Closing standard PyVista visualizer...")
                    self.visualizer.plotter.close()
                    self.visualizer.plotter = None
                    log_info(self.logger, "Standard PyVista visualizer closed")
            except Exception as e:
                log_warning(self.logger, f"Visualization cleanup warning: {e}")
            finally:
                self.visualizer = None
        
        # Clean up monitor window
        if self.monitor:
            try:
                self.monitor.stop()
            except Exception as e:
                log_warning(self.logger, f"Monitor cleanup warning: {e}")
            finally:
                self.monitor = None
        
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
        
        # Log performance statistics
        if self._start_time > 0:
            elapsed = time.time() - self._start_time
            avg_fps = self._frame_count / elapsed if elapsed > 0 else 0
            log_info(self.logger, f"Simulation ran for {elapsed:.1f}s")
            log_info(self.logger, f"Average update rate: {avg_fps:.1f} Hz")
        
        # Final multiprocessing resource cleanup
        try:
            cleanup_multiprocessing_resources()
            log_info(self.logger, "Multiprocessing resources cleaned up")
        except Exception as e:
            log_warning(self.logger, f"Error during multiprocessing cleanup: {e}")
        
        log_success(self.logger, "Simulation shutdown complete")
    
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
        
        log_debug(self.logger, f"Added robot '{name}' to {frequency} Hz frequency group "
              f"({len(self.frequency_groups[frequency]['robots'])} robots in group)")
        log_debug(self.logger, f"Robot internal updates will be grouped at {frequency} Hz")
    
    def _add_callback_to_frequency_group(self, name: str, callback: Callable, frequency: float):
        """Add control callback to appropriate frequency group"""
        
        # Use robot's registered frequency if different from callback frequency
        robot_frequency = self.robot_frequencies.get(name, frequency)
        
        if robot_frequency != frequency:
            log_warning(self.logger, f"Robot '{name}' callback frequency ({frequency} Hz) differs from "
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
        log_debug(self.logger, f"Added callback for '{name}' to {frequency} Hz frequency group")
    
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
        
        log_debug(self.logger, f"Added object '{name}' to {frequency} Hz frequency group "
              f"({len(self.frequency_groups[frequency]['objects'])} objects in group)")
        log_debug(self.logger, f"Object internal updates will be grouped at {frequency} Hz")
    
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
                log_error(self.logger, f"Robot update error for {robot.name}: {e}")
        
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
                log_error(self.logger, f"Object update error for {obj.name if hasattr(obj, 'name') else 'unnamed'}: {e}")
        
        return object_update_wrapper
    
    def _start_frequency_grouped_processes(self):
        """Start all frequency-grouped processes"""
        
        log_info(self.logger, "Starting frequency-grouped processes...")
        
        active_frequencies = sorted(self.frequency_groups.keys())
        total_robots = sum(len(group['robots']) for group in self.frequency_groups.values())
        
        log_debug(self.logger, "Process optimization:")
        log_debug(self.logger, f"Traditional approach: {total_robots} processes (1 per robot)")
        log_debug(self.logger, f"Frequency-grouped: {len(active_frequencies)} processes (1 per frequency)")
        reduction_percent = self._calculate_process_reduction_percent()
        log_debug(self.logger, f"Process reduction: {reduction_percent:.1f}%")
        
        log_debug(self.logger, "Frequency distribution:")
        for frequency in active_frequencies:
            group = self.frequency_groups[frequency]
            robot_count = len(group['robots'])
            object_count = len(group['objects'])
            callback_count = len(group['callbacks'])
            robot_update_count = len(group['robot_updates'])
            object_update_count = len(group['object_updates'])
            
            total_items = robot_count + object_count + callback_count
            internal_updates = robot_update_count + object_update_count
            
            log_debug(self.logger, f"{frequency:6.1f} Hz: {robot_count:3d} robots, {object_count:2d} objects, "
                  f"{callback_count:3d} callbacks, {internal_updates:3d} internal updates")
            
            # Start process for this frequency
            process = self.env.process(self._frequency_group_process_loop(frequency))
            group['process'] = process
        
        log_success(self.logger, f"Started {len(active_frequencies)} frequency-grouped processes")
    
    def _frequency_group_process_loop(self, frequency: float):
        """Process loop for a specific frequency group - handles all updates at this frequency"""
        
        group = self.frequency_groups[frequency]
        dt = 1.0 / frequency
        
        log_debug(self.logger, f"Starting {frequency} Hz unified frequency group process")
        log_debug(self.logger, f"Processing: {len(group['callbacks'])} callbacks, "
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
                            log_error(self.logger, f"Robot update error for {robot_update_info['name']}: {e}")
                    
                    # 2. Execute all object internal updates in this frequency group  
                    for object_update_info in group['object_updates']:
                        try:
                            object_update_info['update_function'](dt)
                            object_update_info['call_count'] += 1
                            group['update_count'] += 1
                        except Exception as e:
                            log_error(self.logger, f"Object update error for {object_update_info['name']}: {e}")
                    
                    # 3. Execute all user control callbacks in this frequency group
                    for callback_info in group['callbacks']:
                        try:
                            callback_info['callback'](dt)
                            callback_info['call_count'] += 1
                            group['callback_count'] += 1
                        except Exception as e:
                            log_error(self.logger, f"Callback error for {callback_info['robot_name']}: {e}")
                
                # Single yield for entire frequency group - this is the revolutionary optimization!
                # All robot updates, object updates, and callbacks processed in ONE yield
                yield self.env.timeout(dt)
                
            except Exception as e:
                log_error(self.logger, f"Frequency group {frequency} Hz process error: {e}")
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
            
            log_info(self.logger, "Timing Performance Statistics:")
            log_info(self.logger, f"Simulation time: {stats.sim_time:.2f}s")
            log_info(self.logger, f"Real time elapsed: {stats.real_time_elapsed:.2f}s")
            log_info(self.logger, f"Target factor: {stats.real_time_factor:.2f}x")
            log_info(self.logger, f"Actual factor: {stats.actual_speed}")
            
            # Calculate error percentage
            if stats.real_time_elapsed > 0 and stats.sim_time > 0:
                actual_factor = stats.sim_time / stats.real_time_elapsed
                error_percent = abs(actual_factor - stats.real_time_factor) / stats.real_time_factor * 100
                
                if error_percent < 5.0:
                    status = "EXCELLENT"
                elif error_percent < 10.0:
                    status = "GOOD" 
                elif error_percent < 20.0:
                    status = "FAIR"
                else:
                    status = "POOR"
                    
                log_info(self.logger, f"Timing accuracy: {status} ({error_percent:.1f}% error)")
            else:
                log_info(self.logger, "Timing accuracy: N/A (insufficient data)")
                
        except Exception as e:
            log_warning(self.logger, f"Could not get timing stats: {e}")
    
    def _handle_monitor_control(self, command: str):
        """Handle control commands from monitor window"""
        if command == 'toggle_play_pause':
            self._simulation_paused = not self._simulation_paused
            log_info(self.logger, f"Monitor control: {'Paused' if self._simulation_paused else 'Resumed'}")
        elif command == 'reset':
            self._reset_requested = True
            log_info(self.logger, "Monitor control: Reset requested")
    
    def _update_monitor_data(self):
        """Update monitor window with current simulation data"""
        if not self.monitor:
            return
        
        # Check if monitor is still running
        if hasattr(self.monitor, 'running') and not self.monitor.running:
            return
            
        
        try:
            # Get timing stats
            timing_stats = self.time_manager.get_timing_stats() if self.time_manager else None
            # Calculate timing accuracy
            timing_accuracy = 0.0
            if timing_stats and timing_stats.real_time_elapsed > 0 and timing_stats.sim_time > 0:
                actual_factor = timing_stats.sim_time / timing_stats.real_time_elapsed
                if timing_stats.real_time_factor > 0:
                    error_percent = abs(actual_factor - timing_stats.real_time_factor) / timing_stats.real_time_factor * 100
                    timing_accuracy = 100.0 - error_percent
                else:
                    error_percent = 'N/A'
                    timing_accuracy = 'N/A'
                        
            # Prepare enhanced monitor data
            monitor_data = {
                'sim_time': timing_stats.sim_time if timing_stats else 0.0,
                'target_rtf': timing_stats.real_time_factor if timing_stats else self.config.real_time_factor,
                'actual_rtf': f"{actual_factor:.2f}x" if actual_factor > 0 else "N/A",
                'time_step': (self.config.time_step * 1000),  # Convert to milliseconds
                'fps': 1.0 / self.config.time_step,
                'visualization': self.config.visualization_backend if self.config.visualization else 'headless',
                'robots': len(self.robots),
                'objects': len(self.objects),
                'architecture': 'Event-Driven SimPy',
                'simulation_state': 'paused' if self._simulation_paused else 'running'
            }
            
            # Send data to monitor with error handling
            try:
                self.monitor.update_data(monitor_data)
            except Exception as monitor_error:
                # If monitor fails repeatedly, disable it
                if hasattr(self.monitor, 'x11_error_count'):
                    self.monitor.x11_error_count += 1
                    if self.monitor.x11_error_count >= self.monitor.max_x11_errors:
                        log_warning(self.logger, "Monitor experiencing too many errors, disabling...")
                        try:
                            self.monitor.stop()
                        except:
                            pass
                        self.monitor = None
            
        except Exception as e:
            # Silently ignore monitor update errors to avoid disrupting simulation
            pass


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