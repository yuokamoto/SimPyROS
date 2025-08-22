#!/usr/bin/env python3
"""
Robot Class for SimPyROS - Advanced URDF-based Robot Implementation
Designed for ROS 2 integration with joint-level control
"""

import simpy
import simpy.rt
import numpy as np
from typing import Dict, List, Optional, Tuple, Any, Union, Generator
from dataclasses import dataclass
from enum import Enum
import warnings
import time
import threading

from core.simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from core.urdf_loader import URDFLoader, URDFLink, URDFJoint
import copy
from core.time_manager import TimeManager, get_global_time_manager
from core.logger import get_logger, log_success, log_warning, log_error, log_debug, log_info
from scipy.spatial.transform import Rotation


# Module logger
logger = get_logger(__name__)


class JointType(Enum):
    """Joint types matching ROS 2 conventions"""
    FIXED = "fixed"
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    FLOATING = "floating"
    PLANAR = "planar"


class JointMode(Enum):
    """Joint control modes for ROS 2 compatibility"""
    POSITION = "position"
    VELOCITY = "velocity"
    EFFORT = "effort"
    IDLE = "idle"


@dataclass
class JointState:
    """Joint state information - compatible with ROS 2 sensor_msgs/JointState"""
    name: str
    position: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0
    
    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary for ROS 2 message compatibility"""
        return {
            'position': self.position,
            'velocity': self.velocity, 
            'effort': self.effort
        }


@dataclass
class JointCommand:
    """Joint command for control - ROS 2 compatible interface"""
    name: str
    mode: JointMode
    target_position: Optional[float] = None
    target_velocity: Optional[float] = None
    target_effort: Optional[float] = None
    max_velocity: Optional[float] = None
    max_effort: Optional[float] = None


class Joint:
    """Individual joint representation with control capabilities"""
    
    def __init__(self, joint_info: URDFJoint):
        self.name = joint_info.name
        self.joint_type = JointType(joint_info.joint_type) if joint_info.joint_type in [jt.value for jt in JointType] else JointType.FIXED
        self.parent_link = joint_info.parent_link
        self.child_link = joint_info.child_link
        self.origin_pos = joint_info.origin_pos.copy()
        self.origin_rot = joint_info.origin_rot
        self.axis = joint_info.axis.copy() if joint_info.axis is not None else np.array([0, 0, 1])
        
        # Joint limits
        self.limits = joint_info.limits or {}
        self.position_limits = (
            self.limits.get('lower', -np.pi),
            self.limits.get('upper', np.pi)
        )
        self.velocity_limit = self.limits.get('velocity', 1.0)
        self.effort_limit = self.limits.get('effort', 100.0)
        
        # Current state
        self.state = JointState(name=self.name)
        
        # Control
        self.mode = JointMode.IDLE
        self.command: Optional[JointCommand] = None
        
        # Control gains (for internal simulation)
        self.position_gain = 10.0
        self.velocity_gain = 1.0
        self.effort_gain = 1.0
    
    def set_command(self, command: JointCommand):
        """Set joint command - ROS 2 compatible interface"""
        if command.name != self.name:
            warnings.warn(f"Command name {command.name} doesn't match joint name {self.name}")
            return
            
        self.command = command
        self.mode = command.mode
    
    def update(self, dt: float):
        """Update joint state based on current command"""
        if self.command is None or self.mode == JointMode.IDLE:
            return
            
        if self.joint_type == JointType.FIXED:
            return
            
        # Simple control implementation for simulation
        if self.mode == JointMode.POSITION and self.command.target_position is not None:
            # Position control with velocity limiting
            position_error = self.command.target_position - self.state.position
            desired_velocity = self.position_gain * position_error
            
            # Apply velocity limit
            max_vel = self.command.max_velocity or self.velocity_limit
            desired_velocity = np.clip(desired_velocity, -max_vel, max_vel)
            
            self.state.velocity = desired_velocity
            self.state.position += self.state.velocity * dt
            
        elif self.mode == JointMode.VELOCITY and self.command.target_velocity is not None:
            # Direct velocity control
            max_vel = self.command.max_velocity or self.velocity_limit
            self.state.velocity = np.clip(self.command.target_velocity, -max_vel, max_vel)
            self.state.position += self.state.velocity * dt
            
        elif self.mode == JointMode.EFFORT and self.command.target_effort is not None:
            # Simplified effort control (would need mass/inertia for realistic simulation)
            max_eff = self.command.max_effort or self.effort_limit
            effort = np.clip(self.command.target_effort, -max_eff, max_eff)
            self.state.effort = effort
            
            # Simple effort to acceleration conversion
            acceleration = effort * self.effort_gain
            self.state.velocity += acceleration * dt
            self.state.velocity = np.clip(self.state.velocity, -self.velocity_limit, self.velocity_limit)
            self.state.position += self.state.velocity * dt
        
        # Apply position limits for revolute joints
        if self.joint_type == JointType.REVOLUTE:
            self.state.position = np.clip(self.state.position, 
                                         self.position_limits[0], 
                                         self.position_limits[1])
        
        # Continuous joints wrap around
        elif self.joint_type == JointType.CONTINUOUS:
            self.state.position = self.state.position % (2 * np.pi)
    
    def get_current_transform(self) -> Tuple[np.ndarray, Rotation]:
        """Get current transformation based on joint position"""
        if self.joint_type == JointType.FIXED:
            return self.origin_pos, self.origin_rot
            
        elif self.joint_type in [JointType.REVOLUTE, JointType.CONTINUOUS]:
            # Rotation about joint axis
            joint_rotation = Rotation.from_rotvec(self.axis * self.state.position)
            combined_rotation = self.origin_rot * joint_rotation
            return self.origin_pos, combined_rotation
            
        elif self.joint_type == JointType.PRISMATIC:
            # Translation along joint axis
            translation = self.axis * self.state.position
            new_position = self.origin_pos + translation
            return new_position, self.origin_rot
            
        else:
            # Default to fixed behavior
            return self.origin_pos, self.origin_rot


class Link:
    """Robot link representation"""
    
    def __init__(self, link_info: URDFLink):
        self.name = link_info.name
        self.geometry_type = link_info.geometry_type
        self.geometry_params = link_info.geometry_params.copy()
        self.color = link_info.color
        self.mesh_path = link_info.mesh_path
        
        # Current transform (computed from joints)
        self.current_pose: Optional[Pose] = None


@dataclass
class RobotParameters(ObjectParameters):
    """Robot-specific parameters extending ObjectParameters"""
    urdf_path: str
    use_urdf_initial_pose: bool = True
    joint_update_rate: float = 10.0  # Hz, reasonable frequency for joint control
    unified_process: bool = True  # New: Use single unified process instead of multiple
    frequency_grouping_managed: bool = False  # If True, disable internal processes (managed by FrequencyGroup)
    
    def __post_init__(self):
        # Ensure this is a dynamic object
        if self.object_type != ObjectType.DYNAMIC:
            self.object_type = ObjectType.DYNAMIC


class Robot(SimulationObject):
    """
    Advanced Robot class with independent SimPy processes for different subsystems
    
    Features:
    - URDF-based robot description loading
    - Independent SimPy processes for joint control, base motion, etc.
    - Event-driven robot behaviors
    - ROS 2 compatible interfaces
    - Real-time forward kinematics
    - Scalable multi-process architecture
    """
    
    def __init__(self, env: simpy.Environment, parameters: RobotParameters, time_manager: Optional[TimeManager] = None):
        # Initialize parent SimulationObject
        super().__init__(env, parameters, time_manager)
        
        self.robot_parameters = parameters
        self.time_manager = time_manager or get_global_time_manager()
        
        # URDF and robot structure
        self.urdf_loader: Optional[URDFLoader] = None
        self.links: Dict[str, Link] = {}
        self.joints: Dict[str, Joint] = {}
        self.joint_names: List[str] = []
        self.link_names: List[str] = []
        
        # Robot state
        self.base_link_name: str = ""
        self.robot_name: str = "unknown_robot"
        
        # Control state - thread-safe for multi-process access
        self.joint_command_queue: List[JointCommand] = []
        self._joint_command_lock = threading.Lock()
        
        # Robot-specific unified process state (joint control)
        
        # Process control flags
        self._processes_active = False
        self._joint_control_frequency = self.robot_parameters.joint_update_rate
        
        # Robot-specific event-driven flags
        self._has_joint_commands = False
        
        # Event lock for robot-specific operations
        self._robot_event_lock = threading.Lock()
        
        # Joint update process (separate from base SimulationObject processes)
        self._joint_process = None
        
        # Load URDF
        self._load_urdf()
        
        # Start independent SimPy processes
        self._start_robot_processes()
    
    def _unified_process_custom_updates(self, current_time: float, active_updates: bool) -> bool:
        """Override to add robot-specific joint control to unified process"""
        joint_dt = 1.0 / self._joint_control_frequency
        joint_active = False
        
        # Joint Control (event-driven + periodic)
        if (self._has_joint_commands or hasattr(self, '_last_joint_update')):
            if not hasattr(self, '_last_joint_update'):
                self._last_joint_update = 0.0
                
            if (self._has_joint_commands or 
                (current_time - self._last_joint_update) >= joint_dt):
                
                with self._joint_command_lock:
                    # Process joint commands if any
                    while self.joint_command_queue:
                        command = self.joint_command_queue.pop(0)
                        if command.name in self.joints:
                            self.joints[command.name].set_command(command)
                            joint_active = True
                    
                    # Update joints if commands processed or time elapsed
                    if joint_active or (current_time - self._last_joint_update) >= joint_dt:
                        for joint in self.joints.values():
                            joint.update(joint_dt)
                        self._update_forward_kinematics()
                        self._last_joint_update = current_time
                        joint_active = True
                        
                        # Clear joint command flag if no more commands
                        self._has_joint_commands = len(self.joint_command_queue) > 0
        
        return joint_active
    
    def _load_urdf(self) -> bool:
        """Load robot from URDF file"""
        try:
            self.urdf_loader = URDFLoader()
            
            if not self.urdf_loader.is_available():
                raise RuntimeError("URDF loader not available")
                
            if not self.urdf_loader.load_urdf(self.robot_parameters.urdf_path):
                raise RuntimeError(f"Failed to load URDF: {self.robot_parameters.urdf_path}")
            
            # Extract robot information
            self.robot_name = self.urdf_loader.robot_name
            
            # Create Link objects
            for link_name, link_info in self.urdf_loader.links.items():
                self.links[link_name] = Link(link_info)
            
            # Create Joint objects  
            for joint_name, joint_info in self.urdf_loader.joints.items():
                self.joints[joint_name] = Joint(joint_info)
            
            # Set up lists for easy access
            self.link_names = list(self.links.keys())
            self.joint_names = list(self.joints.keys())
            
            # Find base link
            self.base_link_name = self._find_base_link()
            
            # Set initial pose from URDF if requested
            if self.robot_parameters.use_urdf_initial_pose:
                # Use the pose as specified in ObjectParameters
                pass
            
            log_debug(logger, f"Robot '{self.robot_name}' loaded successfully")
            log_debug(logger, f"   Links: {len(self.links)}, Joints: {len(self.joints)}")
            log_debug(logger, f"   Base link: {self.base_link_name}")
            
            return True
            
        except Exception as e:
            log_error(logger, f"Failed to load robot from URDF: {e}")
            return False
    
    def _find_base_link(self) -> str:
        """Find the base (root) link of the robot"""
        # Links that are children of joints
        child_links = set(joint.child_link for joint in self.joints.values())
        
        # Base link is not a child of any joint
        for link_name in self.links.keys():
            if link_name not in child_links:
                return link_name
        
        # Fallback to first link
        return self.link_names[0] if self.link_names else "unknown"
    
    
    def _start_robot_processes(self):
        """Start robot processes - unified or separate based on configuration"""
        if not self._processes_active:
            self._processes_active = True
            
            # Check if robot is managed by FrequencyGrouping
            if self.robot_parameters.frequency_grouping_managed:
                log_debug(logger, f"Robot '{self.robot_name}': Process management delegated to FrequencyGrouping")
                return
            
            # Use parent SimulationObject's unified process system
            if self.robot_parameters.unified_process:
                # The unified process is already started by parent SimulationObject
                # Joint control will be handled by _unified_process_custom_updates
                log_debug(logger, f"Robot '{self.robot_name}': Using unified event-driven process from SimulationObject")
            else:
                # Start separate joint update process
                self._joint_process = self.env.process(self._joint_update_loop())
                log_debug(logger, f"Robot '{self.robot_name}': Started separate joint update process")
    
    
    
    
    # The _unified_event_driven_loop method has been moved to SimulationObject
    # Robot-specific functionality is handled by _unified_process_custom_updates
    
    def _joint_update_loop(self) -> Generator:
        """Independent SimPy process for joint control updates"""
        joint_dt = 1.0 / self._joint_control_frequency
        idle_timeout = 0.1  # 10Hz when idle
        
        while self._processes_active:
            try:
                active_updates = False
                
                # Joint Control (event-driven + periodic)
                with self._joint_command_lock:
                    # Process joint commands if any
                    while self.joint_command_queue:
                        command = self.joint_command_queue.pop(0)
                        if command.name in self.joints:
                            self.joints[command.name].set_command(command)
                            active_updates = True
                    
                    # Update joints if commands processed or time elapsed
                    if active_updates or self._has_joint_commands:
                        for joint in self.joints.values():
                            joint.update(joint_dt)
                        self._update_forward_kinematics()
                        active_updates = True
                        
                        # Clear joint command flag if no more commands
                        with self._robot_event_lock:
                            self._has_joint_commands = len(self.joint_command_queue) > 0
                
                # Dynamic timeout based on activity
                if active_updates:
                    timeout = joint_dt / 2  # High frequency when active
                else:
                    timeout = idle_timeout  # Lower frequency when idle
                    
            except Exception as e:
                log_error(logger, f"Robot '{self.robot_name}' joint update error: {e}")
                timeout = joint_dt  # Fallback to normal interval on error
            
            # Yield control
            yield self.env.timeout(timeout)
    
    
    
    
    def _update_forward_kinematics(self):
        """Update all link poses based on current joint positions"""
        # Start with base link at robot pose
        link_poses = {self.base_link_name: self.pose}
        
        # Recursively compute poses for all links
        self._compute_link_poses_recursive(self.base_link_name, self.pose, link_poses)
        
        # Update link poses
        for link_name, pose in link_poses.items():
            if link_name in self.links:
                self.links[link_name].current_pose = pose
        
        # Update connected objects (if link connector is available)
        self._update_link_connections()
    
    def _compute_link_poses_recursive(self, parent_link: str, parent_pose: Pose, link_poses: Dict[str, Pose]):
        """Recursively compute link poses from joint transformations"""
        for joint in self.joints.values():
            if joint.parent_link == parent_link and joint.child_link not in link_poses:
                # Get current joint transformation
                joint_pos, joint_rot = joint.get_current_transform()
                
                # Transform joint position by parent pose
                transformed_pos = parent_pose.position + parent_pose.rotation.apply(joint_pos)
                
                # Combine rotations
                combined_rot = parent_pose.rotation * joint_rot
                
                # Create child link pose
                child_pose = Pose.from_position_rotation(transformed_pos, combined_rot)
                link_poses[joint.child_link] = child_pose
                
                # Recursively process children of this link
                self._compute_link_poses_recursive(joint.child_link, child_pose, link_poses)
    
    def _update_link_connections(self):
        """Update objects connected to robot links (if link connector is available)"""
        try:
            # Import here to avoid circular dependency
            from core.link_connector import get_link_connector
            
            connector = get_link_connector()
            
            # Update all connections for this robot
            if self in connector.connections:
                for link_name, connections in connector.connections[self].items():
                    for connection in connections:
                        if connection.active:
                            connector._update_object_pose(self, link_name, connection)
                            
        except ImportError:
            # Link connector not available, skip
            pass
        except Exception as e:
            # Don't let connection errors break robot updates
            warnings.warn(f"Link connection update failed: {e}")
    
    # ====================
    # Robot-level Control Interface (inherited from SimulationObject)
    # ====================
    
    def set_velocity(self, velocity: Velocity):
        """Set robot base velocity (inherited interface)"""
        super().set_velocity(velocity)
        # Event signaling is now handled by parent SimulationObject
    
    def teleport(self, pose: Pose):
        """Teleport robot base (inherited interface)"""
        super().teleport(pose)
    
    # ====================
    # Joint-level Control Interface (ROS 2 compatible)
    # ====================
    
    def set_joint_position(self, joint_name: str, position: float, max_velocity: Optional[float] = None):
        """Set target position for a joint - ROS 2 style interface (thread-safe)"""
        if joint_name not in self.joints:
            warnings.warn(f"Joint '{joint_name}' not found in robot")
            return
            
        command = JointCommand(
            name=joint_name,
            mode=JointMode.POSITION,
            target_position=position,
            max_velocity=max_velocity
        )
        
        # Thread-safe command queue access
        with self._joint_command_lock:
            self.joint_command_queue.append(command)
        
        # Signal event for unified process
        with self._robot_event_lock:
            self._has_joint_commands = True
    
    def set_joint_velocity(self, joint_name: str, velocity: float, max_effort: Optional[float] = None):
        """Set target velocity for a joint - ROS 2 style interface (thread-safe)"""
        if joint_name not in self.joints:
            warnings.warn(f"Joint '{joint_name}' not found in robot")
            return
            
        command = JointCommand(
            name=joint_name,
            mode=JointMode.VELOCITY,
            target_velocity=velocity,
            max_effort=max_effort
        )
        
        # Thread-safe command queue access
        with self._joint_command_lock:
            self.joint_command_queue.append(command)
        
        # Signal event for unified process
        with self._robot_event_lock:
            self._has_joint_commands = True
    
    def set_joint_effort(self, joint_name: str, effort: float, max_velocity: Optional[float] = None):
        """Set target effort (torque/force) for a joint - ROS 2 style interface (thread-safe)"""
        if joint_name not in self.joints:
            warnings.warn(f"Joint '{joint_name}' not found in robot")
            return
            
        command = JointCommand(
            name=joint_name,
            mode=JointMode.EFFORT,
            target_effort=effort,
            max_velocity=max_velocity
        )
        
        # Thread-safe command queue access
        with self._joint_command_lock:
            self.joint_command_queue.append(command)
        
        # Signal event for unified process
        with self._robot_event_lock:
            self._has_joint_commands = True
    
    def set_joint_positions(self, joint_positions: Dict[str, float], max_velocity: Optional[float] = None):
        """Set multiple joint positions simultaneously - ROS 2 trajectory style"""
        for joint_name, position in joint_positions.items():
            self.set_joint_position(joint_name, position, max_velocity)
    
    def set_joint_velocities(self, joint_velocities: Dict[str, float], max_effort: Optional[float] = None):
        """Set multiple joint velocities simultaneously"""
        for joint_name, velocity in joint_velocities.items():
            self.set_joint_velocity(joint_name, velocity, max_effort)
    
    def stop_joint(self, joint_name: str):
        """Stop a specific joint"""
        self.set_joint_velocity(joint_name, 0.0)
    
    def stop_all_joints(self):
        """Stop all joints (thread-safe)"""
        with self._joint_command_lock:
            # Clear command queue and add stop commands
            self.joint_command_queue.clear()
            for joint_name in self.joint_names:
                self.stop_joint(joint_name)
    
    def shutdown_processes(self):
        """Shutdown all robot processes"""
        self._processes_active = False
        
        # Stop joint process if running separately
        if self._joint_process:
            try:
                self._joint_process.interrupt()
                self._joint_process = None
                log_debug(logger, f"Robot '{self.robot_name}': Joint process stopped")
            except Exception:
                pass
        
        # Use parent SimulationObject's stop method
        self.stop_update_process()
        
        log_debug(logger, f"Robot '{self.robot_name}': Process shutdown")
    
    # ====================
    # State Query Interface (ROS 2 compatible)
    # ====================
    
    def get_joint_names(self) -> List[str]:
        """Get list of all joint names"""
        return self.joint_names
    
    def get_link_names(self) -> List[str]:
        """Get list of all link names"""
        return self.link_names
    
    def get_joint_state(self, joint_name: str) -> Optional[JointState]:
        """Get current state of a joint - ROS 2 compatible"""
        if joint_name in self.joints:
            return self.joints[joint_name].state
        return None
    
    def get_joint_states(self) -> Dict[str, JointState]:
        """Get all joint states - ROS 2 sensor_msgs/JointState compatible"""
        return {name: joint.state for name, joint in self.joints.items()}
    
    def get_joint_positions(self) -> Dict[str, float]:
        """Get current positions of all joints"""
        return {name: joint.state.position for name, joint in self.joints.items()}
    
    def get_joint_velocities(self) -> Dict[str, float]:
        """Get current velocities of all joints"""
        return {name: joint.state.velocity for name, joint in self.joints.items()}
    
    def get_joint_efforts(self) -> Dict[str, float]:
        """Get current efforts of all joints"""
        return {name: joint.state.effort for name, joint in self.joints.items()}
    
    def get_link_pose(self, link_name: str) -> Optional[Pose]:
        """Get current pose of a specific link"""
        if link_name in self.links:
            return self.links[link_name].current_pose
        return None
    
    def get_link_poses(self) -> Dict[str, Pose]:
        """Get current poses of all links"""
        return {name: link.current_pose for name, link in self.links.items() if link.current_pose is not None}
    
    # ====================
    # Robot Information Interface
    # ====================
    
    def get_robot_info(self) -> Dict[str, Any]:
        """Get comprehensive robot information"""
        return {
            'name': self.robot_name,
            'urdf_path': self.robot_parameters.urdf_path,
            'base_link': self.base_link_name,
            'num_links': len(self.links),
            'num_joints': len(self.joints),
            'link_names': self.link_names,
            'joint_names': self.joint_names,
            'joint_types': {name: joint.joint_type.value for name, joint in self.joints.items()},
            'joint_limits': {name: {
                'position': joint.position_limits,
                'velocity': joint.velocity_limit,
                'effort': joint.effort_limit
            } for name, joint in self.joints.items()}
        }
    
    def print_robot_info(self):
        """Print detailed robot information"""
        info = self.get_robot_info()
        log_info(logger, f"Robot Information: {info['name']}")
        log_info(logger, f"URDF: {info['urdf_path']}")
        log_info(logger, f"Base Link: {info['base_link']}")
        log_info(logger, f"Links ({info['num_links']}): {', '.join(info['link_names'])}")
        log_info(logger, f"Joints ({info['num_joints']}):")
        for joint_name in info['joint_names']:
            joint_type = info['joint_types'][joint_name]
            joint_limits = info['joint_limits'][joint_name]
            log_info(logger, f"  - {joint_name}: {joint_type}")
            if joint_type != 'fixed':
                log_info(logger, f"    Position: {joint_limits['position']}")
                log_info(logger, f"    Velocity: {joint_limits['velocity']}")
                log_info(logger, f"    Effort: {joint_limits['effort']}")
    
    # ====================
    # Programmatic Robot Creation (7.1)
    # ====================
    
    def add_link(self, name: str, geometry_type: str, 
                geometry_params: Dict, color: Tuple[float, float, float] = (0.7, 0.7, 0.7),
                pose: Pose = None) -> bool:
        """
        Add a link programmatically to the robot
        
        Args:
            name: Link name
            geometry_type: "box", "cylinder", "sphere", "mesh"
            geometry_params: Parameters for geometry (size, radius, height, etc.)
            color: RGB(A) color tuple
            pose: Optional pose relative to parent
            
        Returns:
            Success status
        """
        if name in self.links:
            warnings.warn(f"Link '{name}' already exists")
            return False
        
        try:
            # Create a mock URDFLink object
            from core.urdf_loader import URDFLink
            
            # Ensure color is RGBA
            if len(color) == 3:
                color = color + (1.0,)  # Add alpha
            
            # Create link object
            link_info = URDFLink(
                name=name,
                geometry_type=geometry_type,
                geometry_params=geometry_params.copy(),
                color=color,
                mesh_path=""  # Empty for programmatic links
            )
            
            # Set pose if provided
            if pose is not None:
                link_info.pose = pose
            
            # Create Link instance
            self.links[name] = Link(link_info)
            self.link_names = list(self.links.keys())
            
            log_debug(logger, f"Added link '{name}': {geometry_type}")
            return True
            
        except Exception as e:
            log_error(logger, f"Failed to add link '{name}': {e}")
            return False
    
    def add_joint(self, name: str, joint_type: Union[JointType, str],
                 parent_link: str, child_link: str,
                 origin_pos: np.ndarray, origin_rot: Rotation,
                 axis: np.ndarray = None, limits: Dict = None) -> bool:
        """
        Add a joint programmatically to the robot
        
        Args:
            name: Joint name
            joint_type: Joint type (JointType enum or string)
            parent_link: Name of parent link
            child_link: Name of child link
            origin_pos: Joint origin position [x, y, z]
            origin_rot: Joint origin rotation (scipy Rotation)
            axis: Joint axis vector (for revolute/prismatic joints)
            limits: Joint limits dict {"lower": float, "upper": float, "velocity": float, "effort": float}
            
        Returns:
            Success status
        """
        if name in self.joints:
            warnings.warn(f"Joint '{name}' already exists")
            return False
            
        if parent_link not in self.links:
            log_error(logger, f"Parent link '{parent_link}' not found")
            return False
            
        if child_link not in self.links:
            log_error(logger, f"Child link '{child_link}' not found")
            return False
        
        try:
            # Create a mock URDFJoint object
            from core.urdf_loader import URDFJoint
            
            # Convert joint_type to string if enum
            if isinstance(joint_type, JointType):
                joint_type_str = joint_type.value
            else:
                joint_type_str = str(joint_type)
            
            # Default axis for different joint types
            if axis is None:
                if joint_type_str in ["revolute", "continuous"]:
                    axis = np.array([0, 0, 1])  # Z-axis rotation
                elif joint_type_str == "prismatic":
                    axis = np.array([0, 0, 1])  # Z-axis translation
                else:
                    axis = np.array([0, 0, 1])  # Default
            
            # Default limits
            if limits is None:
                limits = {
                    "lower": -np.pi,
                    "upper": np.pi,
                    "velocity": 1.0,
                    "effort": 100.0
                }
            
            # Create joint info object
            joint_info = URDFJoint(
                name=name,
                joint_type=joint_type_str,
                parent_link=parent_link,
                child_link=child_link,
                origin_pos=origin_pos.copy(),
                origin_rot=origin_rot,
                axis=axis.copy(),
                limits=limits.copy()
            )
            
            # Create Joint instance
            self.joints[name] = Joint(joint_info)
            self.joint_names = list(self.joints.keys())
            
            log_debug(logger, f"Added joint '{name}': {joint_type_str} ({parent_link} -> {child_link})")
            return True
            
        except Exception as e:
            log_error(logger, f"Failed to add joint '{name}': {e}")
            return False
    
    def finalize_robot(self) -> bool:
        """
        Finalize robot creation and perform consistency checks
        
        Returns:
            Success status
        """
        try:
            # Update link and joint lists
            self.link_names = list(self.links.keys())
            self.joint_names = list(self.joints.keys())
            
            # Find base link (link with no parent joints)
            child_links = set(joint.child_link for joint in self.joints.values())
            base_candidates = [link_name for link_name in self.links.keys() 
                             if link_name not in child_links]
            
            if len(base_candidates) == 1:
                self.base_link_name = base_candidates[0]
            elif len(base_candidates) > 1:
                log_warning(logger, f"Multiple potential base links found: {base_candidates}. Using first: {base_candidates[0]}")
                self.base_link_name = base_candidates[0]
            elif self.link_names:
                log_warning(logger, f"No base link found, using first link: {self.link_names[0]}")
                self.base_link_name = self.link_names[0]
            else:
                log_error(logger, "No links in robot")
                return False
            
            # Update forward kinematics
            self._update_forward_kinematics()
            
            log_debug(logger, f"Robot finalized: {len(self.links)} links, {len(self.joints)} joints")
            log_debug(logger, f"   Base link: {self.base_link_name}")
            
            return True
            
        except Exception as e:
            log_error(logger, f"Failed to finalize robot: {e}")
            return False
    
    def clear_robot(self):
        """Clear all links and joints (for programmatic reconstruction)"""
        self.links.clear()
        self.joints.clear()
        self.link_names.clear()
        self.joint_names.clear()
        self.base_link_name = ""
        log_debug(logger, "Robot cleared")
    
    # ====================
    # Visualization Support
    # ====================
    
    def get_visualization_data(self) -> Dict[str, Any]:
        """Get data for visualization systems (PyVista integration)"""
        if self.urdf_loader is None:
            return {}
            
        return {
            'urdf_loader': self.urdf_loader,
            'current_joint_positions': self.get_joint_positions(),
            'link_poses': self.get_link_poses(),
            'robot_base_pose': self.pose
        }


# ====================
# Factory Functions for Easy Robot Creation
# ====================

def create_robot_from_urdf(env: simpy.Environment, 
                          urdf_path: str,
                          robot_name: str = "robot",
                          initial_pose: Optional[Pose] = None,
                          joint_update_rate: float = 10.0,
                          time_manager: Optional[TimeManager] = None,
                          unified_process: bool = True,
                          frequency_grouping_managed: bool = False) -> Robot:
    """
    Factory function to create a robot from URDF with independent SimPy processes
    
    Args:
        env: SimPy environment (preferably RealtimeEnvironment)
        urdf_path: Path to URDF file
        robot_name: Name for the robot object
        initial_pose: Initial pose (default: origin)
        joint_update_rate: Joint control frequency in Hz
        time_manager: TimeManager instance for centralized time access
        unified_process: Use unified process architecture (default True)
        frequency_grouping_managed: If True, disable individual robot processes (managed by FrequencyGroup)
    
    Returns:
        Robot instance with independent SimPy processes
    """
    if initial_pose is None:
        initial_pose = Pose()
    
    parameters = RobotParameters(
        name=robot_name,
        object_type=ObjectType.DYNAMIC,
        urdf_path=urdf_path,
        initial_pose=initial_pose,
        update_rate=100.0,  # Base motion frequency in Hz
        joint_update_rate=joint_update_rate,
        unified_process=unified_process,
        frequency_grouping_managed=frequency_grouping_managed
    )
    
    return Robot(env, parameters, time_manager)


def create_robot_programmatically(env: simpy.Environment,
                                 robot_name: str = "programmatic_robot",
                                 initial_pose: Optional[Pose] = None,
                                 joint_update_rate: float = 10.0,
                                 time_manager: Optional[TimeManager] = None,
                                 unified_process: bool = True) -> Robot:
    """
    Factory function to create an empty robot for programmatic construction with SimPy processes
    
    Args:
        env: SimPy environment (preferably RealtimeEnvironment)
        robot_name: Name for the robot object
        initial_pose: Initial pose (default: origin)
        joint_update_rate: Joint control frequency in Hz
        time_manager: TimeManager instance for centralized time access
    
    Returns:
        Robot instance ready for programmatic construction with independent processes
    """
    if initial_pose is None:
        initial_pose = Pose()
    
    # Create dummy URDF path for programmatic robots
    parameters = RobotParameters(
        name=robot_name,
        object_type=ObjectType.DYNAMIC,
        urdf_path="",  # Empty for programmatic robots
        initial_pose=initial_pose,
        update_rate=100.0,
        joint_update_rate=joint_update_rate,
        unified_process=unified_process
    )
    
    # Create robot instance but skip URDF loading
    robot = Robot.__new__(Robot)
    robot.env = env
    robot.robot_parameters = parameters
    robot.time_manager = time_manager or get_global_time_manager()
    
    # Initialize parent SimulationObject
    from core.simulation_object import SimulationObject
    SimulationObject.__init__(robot, env, parameters, time_manager)
    
    # Initialize robot-specific attributes
    robot.urdf_loader = None  # No URDF loader for programmatic robots
    robot.links = {}
    robot.joints = {}
    robot.joint_names = []
    robot.link_names = []
    robot.base_link_name = ""
    robot.robot_name = robot_name
    robot.joint_command_queue = []
    robot._joint_command_lock = threading.Lock()
    
    # Process control
    robot._processes_active = False
    robot._joint_control_frequency = joint_update_rate
    # Process references now handled by SimulationObject
    # robot._unified_process = None  # Deprecated - now handled by parent class
    
    log_debug(logger, f"Created empty robot '{robot_name}' for programmatic construction")
    log_debug(logger, "   Use add_link(), add_joint(), finalize_robot() to build, then processes will start automatically")
    
    return robot
