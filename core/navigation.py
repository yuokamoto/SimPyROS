#!/usr/bin/env python3
"""
Navigation Module - Pluggable Navigation Implementations

This module provides a base Navigation class and specific implementations
that can be plugged into SimulationObject for autonomous navigation.

Features:
- Pluggable navigation strategies
- Waypoint-based navigation
- Simple point-to-point navigation
- Extensible for complex algorithms (A*, RRT, etc.)
"""

import math
import numpy as np
from typing import List, Optional, Generator, Protocol, Tuple
from abc import ABC, abstractmethod
import simpy

# Import from relative paths
from .simulation_object import Pose, Velocity
from .logger import get_logger, log_info, log_error, log_debug


class NavigationStrategy(ABC):
    """Abstract base class for navigation strategies"""
    
    @abstractmethod
    def navigate_to_targets(self, 
                           current_pose: Pose, 
                           targets: List[Pose], 
                           current_target_index: int) -> Tuple[Velocity, bool, int]:
        """
        Calculate navigation velocity towards targets
        
        Args:
            current_pose: Current pose of the object
            targets: List of target poses
            current_target_index: Index of current target in the list
            
        Returns:
            tuple: (velocity_command, reached_target, new_target_index)
        """
        pass
    
    @abstractmethod
    def get_strategy_name(self) -> str:
        """Get the name of this navigation strategy"""
        pass


class SimplePointToPointNavigation(NavigationStrategy):
    """Simple point-to-point navigation strategy"""
    
    def __init__(self, 
                 linear_speed: float = 0.5, 
                 angular_speed: float = 0.3,
                 arrival_tolerance: float = 0.1):
        """
        Initialize simple navigation
        
        Args:
            linear_speed: Maximum linear velocity (m/s)
            angular_speed: Maximum angular velocity (rad/s) 
            arrival_tolerance: Distance tolerance for reaching targets (m)
        """
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.arrival_tolerance = arrival_tolerance
        
    def navigate_to_targets(self, 
                           current_pose: Pose, 
                           targets: List[Pose], 
                           current_target_index: int) -> Tuple[Velocity, bool, int]:
        """Simple straight-line navigation to targets"""
        
        if current_target_index >= len(targets):
            return Velocity.zero(), True, current_target_index
            
        current_target = targets[current_target_index]
        distance = np.linalg.norm(current_target.position - current_pose.position)
        
        # Check if reached current target
        if distance < self.arrival_tolerance:
            return Velocity.zero(), True, current_target_index + 1
        
        # Calculate direction to target
        direction = (current_target.position - current_pose.position) / distance
        
        # Create velocity command (2D navigation)
        velocity = Velocity(
            linear_x=direction[0] * self.linear_speed,
            linear_y=direction[1] * self.linear_speed,
            angular_z=0.0  # No rotation for simple navigation
        )
        
        return velocity, False, current_target_index
    
    def get_strategy_name(self) -> str:
        return "SimplePointToPoint"


class RotateAndMoveNavigation(NavigationStrategy):
    """Navigation that rotates to face target, then moves forward"""
    
    def __init__(self, 
                 linear_speed: float = 0.5,
                 angular_speed: float = 1.0,
                 arrival_tolerance: float = 0.1,
                 orientation_tolerance: float = 0.1):
        """
        Initialize rotate-and-move navigation
        
        Args:
            linear_speed: Maximum linear velocity (m/s)
            angular_speed: Maximum angular velocity (rad/s)
            arrival_tolerance: Distance tolerance for reaching targets (m)
            orientation_tolerance: Angular tolerance for orientation (rad)
        """
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.arrival_tolerance = arrival_tolerance
        self.orientation_tolerance = orientation_tolerance
        
    def navigate_to_targets(self, 
                           current_pose: Pose, 
                           targets: List[Pose], 
                           current_target_index: int) -> Tuple[Velocity, bool, int]:
        """Rotate to face target, then move forward"""
        
        if current_target_index >= len(targets):
            return Velocity.zero(), True, current_target_index
            
        current_target = targets[current_target_index]
        distance = np.linalg.norm(current_target.position - current_pose.position)
        
        # Check if reached current target
        if distance < self.arrival_tolerance:
            return Velocity.zero(), True, current_target_index + 1
        
        # Calculate angle to target
        direction_vector = current_target.position - current_pose.position
        target_angle = math.atan2(direction_vector[1], direction_vector[0])
        
        # Get current orientation (assuming rotation around Z-axis)
        current_euler = current_pose.rotation.as_euler('xyz')
        current_angle = current_euler[2]  # Z-axis rotation
        
        # Calculate angle difference
        angle_diff = target_angle - current_angle
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # If not properly oriented, rotate first
        if abs(angle_diff) > self.orientation_tolerance:
            angular_velocity = self.angular_speed if angle_diff > 0 else -self.angular_speed
            return Velocity(angular_z=angular_velocity), False, current_target_index
        
        # If oriented correctly, move forward
        return Velocity(linear_x=self.linear_speed), False, current_target_index
    
    def get_strategy_name(self) -> str:
        return "RotateAndMove"


class Navigation:
    """
    Navigation controller that can be plugged into SimulationObject
    
    This class manages autonomous navigation using different strategies.
    """
    
    def __init__(self, 
                 env: simpy.Environment,
                 navigation_strategy: NavigationStrategy,
                 frequency: float = 5.0):
        """
        Initialize navigation controller
        
        Args:
            env: SimPy environment
            navigation_strategy: Strategy for navigation behavior
            frequency: Navigation update frequency (Hz)
        """
        self.env = env
        self.strategy = navigation_strategy
        self.frequency = frequency
        
        # Navigation state
        self.targets: List[Pose] = []
        self.current_target_index = 0
        self.is_active = False
        
        # Process management
        self._navigation_process = None
        
        # Callbacks for getting pose and setting velocity
        self._get_pose_callback = None
        self._set_velocity_callback = None
        
        # Logging
        self.logger = get_logger(f'simpyros.navigation.{self.strategy.get_strategy_name()}')
        
    def connect_object(self, get_pose_callback, set_velocity_callback):
        """
        Connect navigation to simulation object
        
        Args:
            get_pose_callback: Function that returns current Pose
            set_velocity_callback: Function that accepts Velocity command
        """
        self._get_pose_callback = get_pose_callback
        self._set_velocity_callback = set_velocity_callback
        log_debug(self.logger, f"Navigation connected to object with {self.strategy.get_strategy_name()} strategy")
    
    def start_navigation(self, targets: List[Pose]) -> bool:
        """
        Start navigation to list of targets
        
        Args:
            targets: List of Pose objects to visit in order
            
        Returns:
            True if navigation started, False if already running
        """
        if self._navigation_process is not None:
            log_error(self.logger, "Navigation already running")
            return False
            
        if not self._get_pose_callback or not self._set_velocity_callback:
            log_error(self.logger, "Navigation not connected to object")
            return False
            
        self.targets = targets.copy()
        self.current_target_index = 0
        self.is_active = True
        
        self._navigation_process = self.env.process(self._navigation_loop())
        log_info(self.logger, f"Started {self.strategy.get_strategy_name()} navigation to {len(targets)} targets")
        return True
    
    def stop_navigation(self):
        """Stop autonomous navigation"""
        self.is_active = False
        
        if self._navigation_process:
            try:
                self._navigation_process.interrupt()
                self._navigation_process = None
            except Exception as e:
                log_error(self.logger, f"Error stopping navigation: {e}")
        
        # Stop the object
        if self._set_velocity_callback:
            self._set_velocity_callback(Velocity.zero())
            
        log_info(self.logger, f"{self.strategy.get_strategy_name()} navigation stopped")
    
    def _navigation_loop(self) -> Generator:
        """Main navigation process loop"""
        nav_dt = 1.0 / self.frequency
        
        try:
            while self.is_active and self.current_target_index < len(self.targets):
                # Get current pose
                current_pose = self._get_pose_callback()
                
                # Calculate navigation command using current strategy
                velocity_cmd, reached_target, new_target_index = self.strategy.navigate_to_targets(
                    current_pose, self.targets, self.current_target_index
                )
                
                # Apply velocity command
                self._set_velocity_callback(velocity_cmd)
                
                # Handle target completion
                if reached_target:
                    if new_target_index < len(self.targets):
                        log_info(self.logger, f"Reached waypoint {self.current_target_index + 1}/{len(self.targets)}")
                        self.current_target_index = new_target_index
                        log_info(self.logger, f"Moving to waypoint {self.current_target_index + 1}/{len(self.targets)}")
                    else:
                        log_info(self.logger, f"Completed all {len(self.targets)} waypoints")
                        self.is_active = False
                        break
                
                # Yield control
                yield self.env.timeout(nav_dt)
                
        except Exception as e:
            log_error(self.logger, f"Navigation error: {e}")
        finally:
            # Stop when navigation ends
            if self._set_velocity_callback:
                self._set_velocity_callback(Velocity.zero())
            self._navigation_process = None
            log_info(self.logger, f"{self.strategy.get_strategy_name()} navigation completed")
    
    def get_status(self) -> dict:
        """Get current navigation status"""
        return {
            'strategy': self.strategy.get_strategy_name(),
            'active': self.is_active,
            'total_targets': len(self.targets),
            'current_target_index': self.current_target_index,
            'remaining_targets': len(self.targets) - self.current_target_index,
            'frequency': self.frequency
        }
    
    def set_strategy(self, new_strategy: NavigationStrategy):
        """
        Change navigation strategy (only when not running)
        
        Args:
            new_strategy: New NavigationStrategy to use
        """
        if self.is_active:
            log_error(self.logger, "Cannot change strategy while navigation is active")
            return False
            
        old_strategy = self.strategy.get_strategy_name()
        self.strategy = new_strategy
        self.logger = get_logger(f'simpyros.navigation.{self.strategy.get_strategy_name()}')
        log_info(self.logger, f"Navigation strategy changed: {old_strategy} → {new_strategy.get_strategy_name()}")
        return True


# Factory functions for common navigation setups
def create_simple_navigation(env: simpy.Environment, 
                           linear_speed: float = 0.5,
                           frequency: float = 5.0) -> Navigation:
    """Create simple point-to-point navigation"""
    strategy = SimplePointToPointNavigation(linear_speed=linear_speed)
    return Navigation(env, strategy, frequency)


def create_rotate_and_move_navigation(env: simpy.Environment,
                                    linear_speed: float = 0.5,
                                    angular_speed: float = 1.0,
                                    frequency: float = 5.0) -> Navigation:
    """Create rotate-and-move navigation"""
    strategy = RotateAndMoveNavigation(linear_speed=linear_speed, angular_speed=angular_speed)
    return Navigation(env, strategy, frequency)