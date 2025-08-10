#!/usr/bin/env python3
"""
Robot Class for SimPyROS - Advanced URDF-based Robot Implementation
Designed for ROS 2 integration with joint-level control
"""

import simpy
import numpy as np
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
import warnings

from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from urdf_loader import URDFLoader, URDFLink, URDFJoint
from scipy.spatial.transform import Rotation


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
    joint_update_rate: float = 100.0  # Hz, for high-frequency joint control
    
    def __post_init__(self):
        # Ensure this is a dynamic object
        if self.object_type != ObjectType.DYNAMIC:
            self.object_type = ObjectType.DYNAMIC


class Robot(SimulationObject):
    """
    Advanced Robot class with URDF loading and joint-level control
    
    Features:
    - URDF-based robot description loading
    - Individual joint control (position, velocity, effort modes)
    - ROS 2 compatible interfaces
    - Real-time forward kinematics
    - Hierarchical control (robot-level + joint-level)
    """
    
    def __init__(self, env: simpy.Environment, parameters: RobotParameters):
        # Initialize parent SimulationObject
        super().__init__(env, parameters)
        
        self.robot_parameters = parameters
        
        # URDF and robot structure
        self.urdf_loader: Optional[URDFLoader] = None
        self.links: Dict[str, Link] = {}
        self.joints: Dict[str, Joint] = {}
        self.joint_names: List[str] = []
        self.link_names: List[str] = []
        
        # Robot state
        self.base_link_name: str = ""
        self.robot_name: str = "unknown_robot"
        
        # Control state
        self.joint_command_queue: List[JointCommand] = []
        
        # Load URDF
        self._load_urdf()
        
        # Start joint control process
        self.joint_process = env.process(self._joint_control_loop())
    
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
            
            print(f"✅ Robot '{self.robot_name}' loaded successfully")
            print(f"   Links: {len(self.links)}, Joints: {len(self.joints)}")
            print(f"   Base link: {self.base_link_name}")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to load robot from URDF: {e}")
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
    
    def _joint_control_loop(self):
        """High-frequency joint control loop"""
        dt = 1.0 / self.robot_parameters.joint_update_rate
        
        while True:
            # Process joint command queue
            while self.joint_command_queue:
                command = self.joint_command_queue.pop(0)
                if command.name in self.joints:
                    self.joints[command.name].set_command(command)
            
            # Update all joints
            for joint in self.joints.values():
                joint.update(dt)
            
            # Update link poses based on current joint states
            self._update_forward_kinematics()
            
            yield self.env.timeout(dt)
    
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
    
    # ====================
    # Robot-level Control Interface (inherited from SimulationObject)
    # ====================
    
    def set_velocity(self, velocity: Velocity):
        """Set robot base velocity (inherited interface)"""
        super().set_velocity(velocity)
    
    def teleport(self, pose: Pose):
        """Teleport robot base (inherited interface)"""
        super().teleport(pose)
    
    # ====================
    # Joint-level Control Interface (ROS 2 compatible)
    # ====================
    
    def set_joint_position(self, joint_name: str, position: float, max_velocity: Optional[float] = None):
        """Set target position for a joint - ROS 2 style interface"""
        if joint_name not in self.joints:
            warnings.warn(f"Joint '{joint_name}' not found in robot")
            return
            
        command = JointCommand(
            name=joint_name,
            mode=JointMode.POSITION,
            target_position=position,
            max_velocity=max_velocity
        )
        self.joint_command_queue.append(command)
    
    def set_joint_velocity(self, joint_name: str, velocity: float, max_effort: Optional[float] = None):
        """Set target velocity for a joint - ROS 2 style interface"""
        if joint_name not in self.joints:
            warnings.warn(f"Joint '{joint_name}' not found in robot")
            return
            
        command = JointCommand(
            name=joint_name,
            mode=JointMode.VELOCITY,
            target_velocity=velocity,
            max_effort=max_effort
        )
        self.joint_command_queue.append(command)
    
    def set_joint_effort(self, joint_name: str, effort: float, max_velocity: Optional[float] = None):
        """Set target effort (torque/force) for a joint - ROS 2 style interface"""
        if joint_name not in self.joints:
            warnings.warn(f"Joint '{joint_name}' not found in robot")
            return
            
        command = JointCommand(
            name=joint_name,
            mode=JointMode.EFFORT,
            target_effort=effort,
            max_velocity=max_velocity
        )
        self.joint_command_queue.append(command)
    
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
        """Stop all joints"""
        for joint_name in self.joint_names:
            self.stop_joint(joint_name)
    
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
        print(f"\n🤖 Robot Information: {info['name']}")
        print(f"URDF: {info['urdf_path']}")
        print(f"Base Link: {info['base_link']}")
        print(f"Links ({info['num_links']}): {', '.join(info['link_names'])}")
        print(f"Joints ({info['num_joints']}):")
        for joint_name in info['joint_names']:
            joint_type = info['joint_types'][joint_name]
            joint_limits = info['joint_limits'][joint_name]
            print(f"  - {joint_name}: {joint_type}")
            if joint_type != 'fixed':
                print(f"    Position: {joint_limits['position']}")
                print(f"    Velocity: {joint_limits['velocity']}")
                print(f"    Effort: {joint_limits['effort']}")
    
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
                          joint_update_rate: float = 100.0) -> Robot:
    """
    Factory function to create a robot from URDF - simplified interface
    
    Args:
        env: SimPy environment
        urdf_path: Path to URDF file
        robot_name: Name for the robot object
        initial_pose: Initial pose (default: origin)
        joint_update_rate: Joint control frequency in Hz
    
    Returns:
        Robot instance
    """
    if initial_pose is None:
        initial_pose = Pose()
    
    parameters = RobotParameters(
        name=robot_name,
        object_type=ObjectType.DYNAMIC,
        urdf_path=urdf_path,
        initial_pose=initial_pose,
        update_interval=0.01,  # 100 Hz for robot base
        joint_update_rate=joint_update_rate
    )
    
    return Robot(env, parameters)


def create_simple_arm_robot(env: simpy.Environment, 
                           robot_name: str = "simple_arm",
                           initial_pose: Optional[Pose] = None) -> Robot:
    """Create a simple arm robot using the default URDF"""
    urdf_path = "examples/robots/simple_robot.urdf"
    return create_robot_from_urdf(env, urdf_path, robot_name, initial_pose)


def create_mobile_robot(env: simpy.Environment,
                       robot_name: str = "mobile_robot", 
                       initial_pose: Optional[Pose] = None) -> Robot:
    """Create a mobile robot using the mobile robot URDF"""
    urdf_path = "examples/robots/mobile_robot.urdf"
    return create_robot_from_urdf(env, urdf_path, robot_name, initial_pose)