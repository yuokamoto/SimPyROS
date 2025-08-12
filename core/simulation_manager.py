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
        self._simulation_paused = False
        self._reset_requested = False
        # SimPy processes for pure simulation environment
        self._simulation_process = None
        self._visualization_process = None
        
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
    
    def _simulation_process_loop(self):
        """Main simulation process loop - SimPy pure environment"""
        dt = 1.0 / self.config.update_rate
        
        while self._running and not self._shutdown_requested:
            # Dynamically apply real time factor to simulation time (updated each loop)
            simpy_dt = dt * self.config.real_time_factor
            # Check if simulation is paused
            if self._simulation_paused:
                # Skip control updates when paused, but continue visualization
                yield self.env.timeout(simpy_dt)
                continue
                
            # Process control callbacks
            current_time = self.env.now
            for robot_name, callback in self.control_callbacks.items():
                if callback.should_call(current_time):
                    try:
                        callback.call(dt, current_time)
                    except Exception as e:
                        print(f"⚠️ Control callback error for {robot_name}: {e}")
            
            self._frame_count += 1
            
            # Yield control back to SimPy with time-adjusted interval
            yield self.env.timeout(simpy_dt)
    
    def _visualization_process_loop(self):
        """Visualization update process loop - SimPy pure environment"""
        if not self.visualizer or not self.visualizer.available:
            return
            
        dt = 1.0 / self.config.visualization_update_rate
        
        while self._running and not self._shutdown_requested:
            
            # Update robot visualizations
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
        print(f"   Real time factor: {self.config.real_time_factor}x")
        print(f"   Visualization: {'On' if self.config.visualization else 'Off'}")
        print("   Press Ctrl+C to stop")
        print("=" * 50)
        
        self._running = True
        self._start_time = time.time()
        
        # Start simulation process in SimPy environment
        self._simulation_process = self.env.process(self._simulation_process_loop())
        
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
                            # Run simulation step
                            self.env.run(until=min(self.env.now + 0.01, simulation_end_time or self.env.now + 0.01))
                        
                        # Always update visualization
                        try:
                            self.visualizer.plotter.update()
                        except:
                            # Window closed by user
                            break
                                
                except KeyboardInterrupt:
                    print("\n🛑 Visualization interrupted")
                    self._shutdown_requested = True
            else:
                # Headless mode: run SimPy environment directly
                if duration:
                    self.env.run(until=duration)
                else:
                    try:
                        self.env.run()
                    except KeyboardInterrupt:
                        print("\n⏹️ Simulation stopped by user")
                        
        except KeyboardInterrupt:
            print("\n🛑 Interrupted by user")
            self._shutdown_requested = True
        finally:
            self.shutdown()
        
        return True
    
    def set_realtime_factor(self, factor: float):
        """
        Update real-time factor during simulation
        
        Args:
            factor: New real-time factor (0.1 to 5.0)
        """
        old_factor = self.config.real_time_factor
        if factor < 0.1:
            factor = 0.1
        elif factor > 5.0:
            factor = 5.0
            
        self.config.real_time_factor = factor
        print(f"⏱️ SimulationManager real-time factor: {old_factor:.2f}x → {factor:.2f}x")
        
        # Also sync back to visualizer if it has different value
        if hasattr(self, 'visualizer') and self.visualizer and hasattr(self.visualizer, 'realtime_factor'):
            if abs(self.visualizer.realtime_factor - factor) > 0.01:
                self.visualizer.realtime_factor = factor
                print(f"🔄 Synchronized visualizer real-time factor: {factor:.2f}x")
    
    def get_realtime_factor(self) -> float:
        """Get current real-time factor"""
        return self.config.real_time_factor
    
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
        
        # Clean up visualization safely
        if self.visualizer:
            try:
                if hasattr(self.visualizer, 'plotter') and self.visualizer.plotter:
                    # Try gentle close first
                    try:
                        self.visualizer.plotter.close()
                    except:
                        # If gentle close fails, try more forceful cleanup
                        try:
                            if hasattr(self.visualizer.plotter, '_exit'):
                                self.visualizer.plotter._exit()
                        except:
                            pass
                    # Clear the plotter reference
                    self.visualizer.plotter = None
            except Exception as e:
                print(f"⚠️ Visualization cleanup warning: {e}")
            finally:
                # Always clear the visualizer reference
                self.visualizer = None
        
        # SimPy processes are automatically cleaned up when environment shuts down
        if self._simulation_process:
            try:
                self._simulation_process.interrupt()
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