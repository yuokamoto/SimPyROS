import simpy
import simpy.rt
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Generator
from enum import Enum
import math
import copy
import numpy as np
import threading
from scipy.spatial.transform import Rotation


class ObjectType(Enum):
    STATIC = "static"
    DYNAMIC = "dynamic"


class Pose:
    """Pose using quaternion for rotation + 3D position"""
    
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
                 roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
        """Create pose from position and Euler angles"""
        self.position = np.array([x, y, z])
        self.rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
    
    @classmethod
    def from_position_quaternion(cls, position: np.ndarray, quaternion: np.ndarray) -> 'Pose':
        """Create pose from position and quaternion [x, y, z, w]"""
        pose = cls()
        pose.position = position.copy()
        pose.rotation = Rotation.from_quat(quaternion)
        return pose
    
    @classmethod
    def from_position_rotation(cls, position: np.ndarray, rotation: Rotation) -> 'Pose':
        """Create pose from position and Rotation object"""
        pose = cls()
        pose.position = position.copy()
        pose.rotation = rotation
        return pose
    
    @property
    def x(self) -> float:
        return self.position[0]
    
    @property 
    def y(self) -> float:
        return self.position[1]
    
    @property
    def z(self) -> float:
        return self.position[2]
    
    @property
    def quaternion(self) -> np.ndarray:
        """Get quaternion as [x, y, z, w]"""
        return self.rotation.as_quat()
    
    @property
    def euler(self) -> np.ndarray:
        """Get [roll, pitch, yaw] in radians"""
        return self.rotation.as_euler('xyz')
    
    @property
    def roll(self) -> float:
        return self.euler[0]
    
    @property
    def pitch(self) -> float:
        return self.euler[1]
    
    @property
    def yaw(self) -> float:
        return self.euler[2]
    
    def copy(self) -> 'Pose':
        return Pose.from_position_rotation(self.position, self.rotation)
    
    def to_transformation_matrix(self) -> np.ndarray:
        """Convert pose to 4x4 transformation matrix for PyVista/VTK"""
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation.as_matrix()
        matrix[:3, 3] = self.position
        return matrix
    
    def __str__(self) -> str:
        """String representation of pose showing position and rotation"""
        roll_deg = math.degrees(self.roll)
        pitch_deg = math.degrees(self.pitch)
        yaw_deg = math.degrees(self.yaw)
        return (f"Pose(pos=({self.x:.3f}, {self.y:.3f}, {self.z:.3f}), "
                f"rot=({roll_deg:.1f}°, {pitch_deg:.1f}°, {yaw_deg:.1f}°))")
    
    def __repr__(self) -> str:
        """Detailed representation of pose"""
        return self.__str__()
    
    def transform_by(self, parent_pose: 'Pose') -> 'Pose':
        """Transform this pose by parent pose"""
        # Rotate position by parent's rotation
        rotated_position = parent_pose.rotation.apply(self.position)
        # Add to parent's position
        world_position = parent_pose.position + rotated_position
        # Combine rotations
        world_rotation = parent_pose.rotation * self.rotation
        
        return Pose.from_position_rotation(world_position, world_rotation)
    
    def inverse_transform_by(self, parent_pose: 'Pose') -> 'Pose':
        """Get relative pose with respect to parent pose"""
        # Get relative position
        relative_position = parent_pose.rotation.inv().apply(self.position - parent_pose.position)
        # Get relative rotation
        relative_rotation = parent_pose.rotation.inv() * self.rotation
        
        return Pose.from_position_rotation(relative_position, relative_rotation)


class Velocity:
    """Velocity represented as a 6-element numpy array [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]"""
    
    def __init__(self, linear_x: float = 0.0, linear_y: float = 0.0, linear_z: float = 0.0,
                 angular_x: float = 0.0, angular_y: float = 0.0, angular_z: float = 0.0):
        self.data = np.array([linear_x, linear_y, linear_z, angular_x, angular_y, angular_z])
    
    @classmethod
    def from_array(cls, array: np.ndarray) -> 'Velocity':
        """Create velocity from numpy array"""
        vel = cls()
        vel.data = array.copy()
        return vel
    
    @classmethod
    def zero(cls) -> 'Velocity':
        """Create zero velocity"""
        return cls()
    
    @property
    def linear_x(self) -> float:
        return self.data[0]
    
    @property
    def linear_y(self) -> float:
        return self.data[1]
    
    @property
    def linear_z(self) -> float:
        return self.data[2]
    
    @property
    def angular_x(self) -> float:
        return self.data[3]
    
    @property
    def angular_y(self) -> float:
        return self.data[4]
    
    @property
    def angular_z(self) -> float:
        return self.data[5]
    
    @property
    def linear(self) -> np.ndarray:
        """Get linear velocity as 3D vector"""
        return self.data[:3]
    
    @property
    def angular(self) -> np.ndarray:
        """Get angular velocity as 3D vector"""
        return self.data[3:]
    
    def copy(self) -> 'Velocity':
        return Velocity.from_array(self.data)
    
    def __mul__(self, scalar: float) -> 'Velocity':
        return Velocity.from_array(self.data * scalar)
    
    def __rmul__(self, scalar: float) -> 'Velocity':
        return self.__mul__(scalar)
    
    def __add__(self, other: 'Velocity') -> 'Velocity':
        return Velocity.from_array(self.data + other.data)
    
    def __sub__(self, other: 'Velocity') -> 'Velocity':
        return Velocity.from_array(self.data - other.data)


@dataclass
class ObjectParameters:
    name: str
    object_type: ObjectType
    urdf_path: Optional[str] = None
    sdf_path: Optional[str] = None
    initial_pose: Pose = Pose()
    update_rate: float = 10.0  # Hz
    frequency_grouping_managed: bool = False  # If True, disable individual motion process
    unified_process: bool = False  # If True, use unified event-driven process instead of separate motion process


class SimulationObject:
    def __init__(self, env: simpy.Environment, parameters: ObjectParameters, time_manager=None):
        self.env = env
        self.parameters = parameters
        self.pose = parameters.initial_pose
        self.velocity = Velocity.zero()
        self._running = False
        
        # Reference to time manager for centralized time access
        self.time_manager = time_manager
        
        # Connection system (bidirectional) - thread-safe
        self.connected_objects: List['SimulationObject'] = []
        self.relative_poses: Dict['SimulationObject', Pose] = {}
        self._connection_lock = threading.Lock()
        
        # Independent SimPy processes for different subsystems
        self._motion_process = None
        self._sensor_process = None
        self._navigation_process = None
        self._process_active = False
        
        # Unified process system (moved from Robot class)
        self._unified_process = None
        self.use_unified_process = getattr(parameters, 'unified_process', False)
        
        # Event-driven optimization
        self._event_lock = threading.Lock()
        
        # Navigation functionality - pluggable navigation system
        self._navigation_controller = None  # Will be set when navigation is configured
        
        # Sensor functionality - pluggable sensor system
        self._sensor_manager = None  # Will be set when sensors are configured
        
        # Start processes for dynamic objects
        if self.parameters.object_type == ObjectType.DYNAMIC:
            if self.use_unified_process:
                self._start_unified_update_process()
            else:
                self._start_separate_update_processes()
    
    def _start_separate_update_processes(self):
        """Start separate SimPy processes for motion, sensors, and navigation"""
        if not self._process_active:
            # Check if motion is managed by FrequencyGrouping
            if self.parameters.frequency_grouping_managed:
                print(f"🔄 SimulationObject '{self.parameters.name}': Process management delegated to FrequencyGrouping")
                return
                
            self._process_active = True
            
            # Start motion update process
            if self.parameters.object_type == ObjectType.DYNAMIC:
                self._motion_process = self.env.process(self._motion_update_loop())
                print(f"🔄 SimulationObject '{self.parameters.name}': Motion process started")
            
            # Start sensor update process if sensors are available
            if self._sensor_manager:
                self._sensor_process = self.env.process(self._sensor_update_loop())
                print(f"🔄 SimulationObject '{self.parameters.name}': Sensor process started")
            
            # Start navigation update process if navigation is available
            if self._navigation_controller:
                self._navigation_process = self.env.process(self._navigation_update_loop())
                print(f"🔄 SimulationObject '{self.parameters.name}': Navigation process started")
    
    def _motion_update_loop(self) -> Generator:
        """Independent SimPy process for object motion updates - Event-driven optimization"""
        dt = 1.0 / self.parameters.update_rate
        idle_timeout = 0.1  # 10Hz when idle
        timeout = idle_timeout
        
        while self._process_active:
            try:
                # Only update if this  has motion                
                if self._has_motion():                
                    self._update_state(dt)
                    # Use normal update interval when active
                    timeout = dt
                    
            except Exception as e:
                print(f"⚠️ SimulationObject '{self.parameters.name}' motion error: {e}")
                timeout = dt  # Fallback to normal interval on error
            
            # Yield control with adaptive timeout
            yield self.env.timeout(timeout)
    
    def _sensor_update_loop(self) -> Generator:
        """Independent SimPy process for sensor updates"""
        sensor_dt = 1.0 / 20.0  # Default 20Hz for sensors
        
        while self._process_active:
            try:
                if self._sensor_manager:
                    current_time = self.time_manager.get_sim_time() if self.time_manager else self.env.now
                    self._sensor_manager.update_all_sensors(current_time, self.pose, self.velocity)
                    
            except Exception as e:
                print(f"⚠️ SimulationObject '{self.parameters.name}' sensor error: {e}")
            
            # Yield control
            yield self.env.timeout(sensor_dt)
    
    def _navigation_update_loop(self) -> Generator:
        """Independent SimPy process for navigation updates"""
        nav_dt = 1.0 / 10.0  # Default 10Hz for navigation
        
        while self._process_active:
            try:
                if self._navigation_controller and hasattr(self._navigation_controller, 'is_active'):
                    if self._navigation_controller.is_active:
                        # Navigation controller will handle its own updates
                        # This loop is mainly for monitoring and cleanup
                        pass
                    
            except Exception as e:
                print(f"⚠️ SimulationObject '{self.parameters.name}' navigation error: {e}")
            
            # Yield control
            yield self.env.timeout(nav_dt)
    
    def _has_motion(self) -> bool:
        """Check if object has non-zero velocity or is connected to moving objects"""
        # Check own velocity
        if np.any(self.velocity.data != 0.0):
            return True
        
        # Check if any connected dynamic objects have motion
        for connected_obj in self.connected_objects:
            if (connected_obj.parameters.object_type == ObjectType.DYNAMIC and
                np.any(connected_obj.velocity.data != 0.0)):
                return True
        
        return False
    
    def _start_unified_update_process(self):
        """Start unified process for all object subsystems"""
        if not self._process_active and self.parameters.object_type == ObjectType.DYNAMIC:
            # Check if motion is managed by FrequencyGrouping
            if self.parameters.frequency_grouping_managed:
                print(f"🔄 SimulationObject '{self.parameters.name}': Unified process management delegated to FrequencyGrouping")
                return
                
            self._process_active = True
            self._unified_process = self.env.process(self._unified_event_driven_loop())
            print(f"🔄 SimulationObject '{self.parameters.name}': Unified process started")
    
    def _unified_event_driven_loop(self) -> Generator:
        """Unified event-driven process combining all object subsystems"""
        last_motion_update = 0.0
        last_sensor_update = 0.0
        last_navigation_update = 0.0
        
        motion_dt = 1.0 / self.parameters.update_rate
        sensor_dt = 1.0 / 20.0  # 20Hz for sensors
        navigation_dt = 1.0 / 10.0  # 10Hz for navigation
        base_timeout = 0.1  # 10Hz base monitoring
        
        while self._process_active:
            current_time = self.time_manager.get_sim_time() if self.time_manager else self.env.now
            active_updates = False
            
            try:
                # 1. Motion updates (periodic with motion detection)
                if (current_time - last_motion_update) >= motion_dt:
                    
                    # Use consistent motion detection method                    
                    if (self.parameters.object_type == ObjectType.DYNAMIC and 
                        self._has_motion()):

                        self._update_state(motion_dt)
                        active_updates = True
                    
                    last_motion_update = current_time
                
                # 2. Sensor updates
                if (self._sensor_manager and 
                    (current_time - last_sensor_update) >= sensor_dt):
                    
                    self._sensor_manager.update_all_sensors(current_time, self.pose, self.velocity)
                    last_sensor_update = current_time
                    active_updates = True
                
                # 3. Navigation updates
                if (self._navigation_controller and 
                    (current_time - last_navigation_update) >= navigation_dt):
                    
                    if hasattr(self._navigation_controller, 'is_active') and self._navigation_controller.is_active:
                        # Navigation controller handles its own updates
                        pass
                    last_navigation_update = current_time
                    active_updates = True
                
                # 4. Allow subclasses to add custom updates (e.g., joint control for robots)
                active_updates = self._unified_process_custom_updates(current_time, active_updates) or active_updates
                
            except Exception as e:
                print(f"⚠️ SimulationObject '{self.parameters.name}' unified process error: {e}")
            
            # Dynamic timeout based on activity
            if active_updates:
                # High frequency when active - use shortest interval
                timeout = min(motion_dt, sensor_dt, navigation_dt) / 2
            else:
                # Lower frequency when idle
                timeout = base_timeout
            
            # Yield control
            yield self.env.timeout(timeout)
    
    def _unified_process_custom_updates(self, current_time: float, active_updates: bool) -> bool:
        """Override in subclasses to add custom unified process updates"""
        return False
    
    def stop_update_process(self):
        """Stop all update processes"""
        self._process_active = False
        
        # Stop separate processes
        if self._motion_process:
            try:
                self._motion_process.interrupt()
                self._motion_process = None
                print(f"🛑 SimulationObject '{self.parameters.name}': Motion process stopped")
            except Exception:
                pass
        
        if self._sensor_process:
            try:
                self._sensor_process.interrupt()
                self._sensor_process = None
                print(f"🛑 SimulationObject '{self.parameters.name}': Sensor process stopped")
            except Exception:
                pass
        
        if self._navigation_process:
            try:
                self._navigation_process.interrupt()
                self._navigation_process = None
                print(f"🛑 SimulationObject '{self.parameters.name}': Navigation process stopped")
            except Exception:
                pass
        
        # Stop unified process
        if self._unified_process:
            try:
                self._unified_process.interrupt()
                self._unified_process = None
                print(f"🛑 SimulationObject '{self.parameters.name}': Unified process stopped")
            except Exception:
                pass
    
    def _update_state(self, dt: float):
        """Update object state with given time step"""
        # Check if connected to any STATIC objects - if so, cannot move
        if self._is_connected_to_static_object():
            return
        
        # Update position (simple integration)
        linear_delta = self.velocity.linear * dt
        # Apply rotation to linear velocity (velocity in local frame)
        world_linear_delta = self.pose.rotation.apply(linear_delta)
        new_position = self.pose.position + world_linear_delta
        
        # Update rotation using angular velocity
        angular_delta = self.velocity.angular * dt
        if np.linalg.norm(angular_delta) > 1e-8:  # Avoid division by zero
            # Create rotation from angular velocity
            delta_rotation = Rotation.from_rotvec(angular_delta)
            new_rotation = self.pose.rotation * delta_rotation
        else:
            new_rotation = self.pose.rotation
        
        # Update pose
        self.pose = Pose.from_position_rotation(new_position, new_rotation)
        
        # Update all connected objects (thread-safe)
        self._update_connected_objects()
    
    def set_velocity(self, velocity: Velocity):
        if self.parameters.object_type == ObjectType.STATIC:
            raise ValueError("Cannot set velocity on static object")
        
        # Check if velocity changed (trigger motion if needed)
        velocity_changed = not np.allclose(self.velocity.data, velocity.data)
        self.velocity = velocity
        self._running = True
    
    def stop(self):
        self.velocity = Velocity.zero()
        self._running = False
    
    def _is_connected_to_static_object(self, visited=None) -> bool:
        """Check if this object is connected to any STATIC objects (directly or indirectly)"""
        if visited is None:
            visited = set()
        
        # Avoid infinite recursion
        if self in visited:
            return False
        visited.add(self)
        
        # Check direct connections
        for connected_obj in self.connected_objects:
            if connected_obj.parameters.object_type == ObjectType.STATIC:
                return True
            # Check indirect connections (recursive)
            if connected_obj._is_connected_to_static_object(visited):
                return True
        
        return False
    
    def _update_connected_objects(self, updated_objects=None):
        """Update all connected objects based on this object's new pose (thread-safe)"""
        if updated_objects is None:
            updated_objects = set()
        
        # Avoid infinite recursion
        if self in updated_objects:
            return
        
        updated_objects.add(self)
        
        with self._connection_lock:
            for connected_obj in self.connected_objects:
                if connected_obj in updated_objects:
                    continue
                    
                # Calculate new pose for connected object
                relative_pose = self.relative_poses[connected_obj]
                new_pose = relative_pose.transform_by(self.pose)
                
                # Update connected object
                connected_obj.pose = new_pose
                
                # Recursively update its other connections
                connected_obj._update_connected_objects(updated_objects)
    
    def connect_to(self, other_object: 'SimulationObject', relative_pose: Optional[Pose] = None):
        """Connect this object to another object with optional relative pose (thread-safe)"""
        # Prevent self-connection
        if other_object is self:
            raise ValueError("Cannot connect object to itself")
        
        with self._connection_lock:
            with other_object._connection_lock:
                # Skip if already connected
                if other_object in self.connected_objects:
                    return
                
                # Calculate relative pose if not provided
                if relative_pose is None:
                    relative_pose = other_object.pose.inverse_transform_by(self.pose)
                else:
                    # Update other object's pose to maintain relative position
                    other_object.pose = relative_pose.transform_by(self.pose)
                
                # Add bidirectional connection
                self.connected_objects.append(other_object)
                self.relative_poses[other_object] = relative_pose
                
                other_object.connected_objects.append(self)
                other_object.relative_poses[self] = self.pose.inverse_transform_by(other_object.pose)
    
    def disconnect_from(self, other_object: 'SimulationObject'):
        """Disconnect from another object (thread-safe)"""
        with self._connection_lock:
            with other_object._connection_lock:
                if other_object in self.connected_objects:
                    self.connected_objects.remove(other_object)
                    del self.relative_poses[other_object]
                    
                    # Remove reverse connection
                    if self in other_object.connected_objects:
                        other_object.connected_objects.remove(self)
                        del other_object.relative_poses[self]
    
    def disconnect_all(self):
        """Disconnect from all connected objects"""
        connected_copy = self.connected_objects.copy()
        for obj in connected_copy:
            self.disconnect_from(obj)
    
    def teleport(self, pose: Pose):
        self.pose = pose.copy()
        self._update_connected_objects()
    
    def get_pose(self) -> Pose:
        return self.pose
    
    def get_velocity(self) -> Velocity:
        return self.velocity
    
    def get_connected_objects(self) -> List['SimulationObject']:
        return self.connected_objects.copy()
    
    def is_connected_to(self, other_object: 'SimulationObject') -> bool:
        return other_object in self.connected_objects
    
    # Navigation functionality - pluggable system
    def set_navigation_controller(self, navigation_controller):
        """Set a navigation controller for autonomous navigation
        
        Args:
            navigation_controller: Instance of Navigation class
        """
        self._navigation_controller = navigation_controller
        if navigation_controller:
            # Connect the navigation controller to this object
            navigation_controller.connect_object(
                get_pose_callback=self.get_pose,
                set_velocity_callback=self.set_velocity
            )
    
    def start_navigation_process(self, targets: Optional[List[Pose]] = None) -> bool:
        """Start autonomous navigation process with list of waypoints
        
        Args:
            targets: List of Pose objects to visit in order
        
        Returns:
            True if navigation started, False if no navigation controller or already running
        """
        if not self._navigation_controller:
            from core.logger import get_logger, log_error
            logger = get_logger(__name__)
            log_error(logger, f"SimulationObject '{self.parameters.name}': No navigation controller configured")
            return False
            
        if targets:
            return self._navigation_controller.start_navigation(targets)
        return False
    
    def stop_navigation_process(self):
        """Stop autonomous navigation process"""
        if self._navigation_controller:
            self._navigation_controller.stop_navigation()
    
    def get_navigation_status(self) -> Optional[dict]:
        """Get current navigation status"""
        if self._navigation_controller:
            return self._navigation_controller.get_status()
        return None
    
    # Sensor functionality - pluggable system
    def initialize_sensor_manager(self):
        """Initialize sensor manager"""
        if self._sensor_manager is None:
            from .sensors import SensorManager
            self._sensor_manager = SensorManager(self.env)
            
    def add_sensor(self, sensor) -> bool:
        """
        Add sensor
        
        Args:
            sensor: Sensor instance inheriting from SensorBase
            
        Returns:
            Success flag
        """
        self.initialize_sensor_manager()
        return self._sensor_manager.add_sensor(sensor)
    
    def remove_sensor(self, sensor_name: str) -> bool:
        """
        Remove sensor
        
        Args:
            sensor_name: Name of sensor to remove
            
        Returns:
            Success flag
        """
        if self._sensor_manager:
            return self._sensor_manager.remove_sensor(sensor_name)
        return False
    
    def get_sensor(self, name: str):
        """Get sensor by name"""
        if self._sensor_manager:
            return self._sensor_manager.get_sensor_by_name(name)
        return None
    
    def get_sensors_by_type(self, sensor_type: str) -> List:
        """Get sensors by type"""
        if self._sensor_manager:
            return self._sensor_manager.get_sensors_by_type(sensor_type)
        return []
    
    def get_latest_sensor_data(self, sensor_name: str):
        """Get latest sensor data"""
        if self._sensor_manager:
            return self._sensor_manager.get_latest_data(sensor_name)
        return None
    
    def get_sensor_data_history(self, sensor_name: str, count: int = None) -> List:
        """Get sensor data history"""
        if self._sensor_manager:
            return self._sensor_manager.get_data_history(sensor_name, count)
        return []
    
    def add_sensor_data_callback(self, callback):
        """Add sensor data callback"""
        self.initialize_sensor_manager()
        self._sensor_manager.add_data_callback(callback)
    
    def get_sensor_manager_status(self) -> Optional[dict]:
        """Get sensor manager status"""
        if self._sensor_manager:
            return self._sensor_manager.get_status()
        return None
    
    def enable_all_sensors(self):
        """Enable all sensors"""
        if self._sensor_manager:
            self._sensor_manager.enable_all_sensors()
    
    def disable_all_sensors(self):
        """Disable all sensors"""
        if self._sensor_manager:
            self._sensor_manager.disable_all_sensors()