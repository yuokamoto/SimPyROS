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
    update_interval: float = 0.1  # seconds


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
        
        # Independent SimPy process for object motion
        self._motion_process = None
        self._process_active = False
        
        # Start motion process for dynamic objects
        if self.parameters.object_type == ObjectType.DYNAMIC:
            self._start_motion_process()
    
    def _start_motion_process(self):
        """Start independent SimPy process for object motion"""
        if not self._process_active and self.parameters.object_type == ObjectType.DYNAMIC:
            self._process_active = True
            self._motion_process = self.env.process(self._motion_process_loop())
            print(f"🔄 SimulationObject '{self.parameters.name}': Motion process started")
    
    def _motion_process_loop(self) -> Generator:
        """Independent SimPy process for object motion updates"""
        dt = self.parameters.update_interval
        
        while self._process_active:
            try:
                # Only update if this is a dynamic object
                if self.parameters.object_type == ObjectType.DYNAMIC:
                    self._update_state(dt)
                    
            except Exception as e:
                print(f"⚠️ SimulationObject '{self.parameters.name}' motion error: {e}")
            
            # Yield control with update interval
            yield self.env.timeout(dt)
    
    def stop_motion_process(self):
        """Stop the motion process"""
        self._process_active = False
        if self._motion_process:
            try:
                self._motion_process.interrupt()
                self._motion_process = None
                print(f"🛑 SimulationObject '{self.parameters.name}': Motion process stopped")
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
    
    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def set_velocity(self, velocity: Velocity):
        if self.parameters.object_type == ObjectType.STATIC:
            raise ValueError("Cannot set velocity on static object")
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