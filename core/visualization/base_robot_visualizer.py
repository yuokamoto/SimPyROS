#!/usr/bin/env python3
"""
Base Robot Visualizer

Abstract base class for robot visualizers to eliminate code duplication
between URDFRobotVisualizer and ProcessURDFAdapter.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, List
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from ..time_manager import TimeManager


class BaseRobotVisualizer(ABC):
    """
    Abstract base class for robot visualizers
    
    Provides common interface methods used by both URDFRobotVisualizer 
    and ProcessURDFAdapter to eliminate code duplication.
    """
    
    def __init__(self):
        """Initialize base robot visualizer"""
        # Time management connections
        self._connected_time_manager = None
        self._connected_simulation_manager = None
        
        # Robot tracking
        self.robots: Dict[str, Any] = {}
        
        # Compatibility
        self.available = True
    
    def connect_time_manager(self, time_manager: TimeManager):
        """Connect to a TimeManager for centralized time access"""
        self._connected_time_manager = time_manager
        
        # Sync initial realtime factor if supported
        if hasattr(self, 'realtime_factor') and hasattr(time_manager, 'get_real_time_factor'):
            self.realtime_factor = time_manager.get_real_time_factor()
            
        print("🔗 Connected to TimeManager for centralized time access")
    
    def connect_simulation_manager(self, simulation_manager):
        """Connect to a SimulationManager for real-time control synchronization"""
        self._connected_simulation_manager = simulation_manager
        
        # Also connect to its time manager if available
        if hasattr(simulation_manager, 'time_manager') and simulation_manager.time_manager:
            self.connect_time_manager(simulation_manager.time_manager)
        
        # Sync initial realtime factor if supported
        if (hasattr(self, 'realtime_factor') and 
            hasattr(simulation_manager, 'config') and 
            hasattr(simulation_manager.config, 'real_time_factor')):
            self.realtime_factor = simulation_manager.config.real_time_factor
            
        print("🔗 Connected to SimulationManager for real-time factor control")
    
    def disconnect_time_manager(self):
        """Disconnect from TimeManager"""
        self._connected_time_manager = None
        print("🔗 Disconnected from TimeManager")
    
    def disconnect_simulation_manager(self):
        """Disconnect from SimulationManager"""
        self._connected_simulation_manager = None
        self.disconnect_time_manager()
        print("🔗 Disconnected from SimulationManager")
    
    def _validate_robot_instance(self, robot_name: str, robot_instance: Any) -> bool:
        """Common robot instance validation logic"""
        if not hasattr(robot_instance, 'links') or not robot_instance.links:
            print(f"❌ Robot instance '{robot_name}' has no link data loaded")
            return False
        return True
    
    def _get_robot_info_string(self, robot_name: str, robot_instance: Any) -> str:
        """Generate robot info string for logging"""
        link_count = len(robot_instance.links) if hasattr(robot_instance, 'links') else 0
        return f"{robot_name} ({link_count} links)"
    
    # Template method for common robot loading logic
    def load_robot_common(self, robot_name: str, robot_instance: Any) -> bool:
        """
        Common robot loading logic using Template Method pattern
        
        Args:
            robot_name: Robot name
            robot_instance: Robot instance with URDF data
            
        Returns:
            bool: Success status
        """
        try:
            # Step 1: Validate robot instance
            if not self._validate_robot_instance(robot_name, robot_instance):
                return False
            
            # Step 2: Store robot in base registry
            self.robots[robot_name] = robot_instance
            
            # Step 3: Call subclass-specific loading logic
            success = self._load_robot_specific(robot_name, robot_instance)
            
            if success:
                print(f"✅ Robot loaded: {self._get_robot_info_string(robot_name, robot_instance)}")
                return True
            else:
                # Cleanup on failure
                if robot_name in self.robots:
                    del self.robots[robot_name]
                print(f"❌ Failed to load robot: {robot_name}")
                return False
                
        except Exception as e:
            # Cleanup on exception
            if robot_name in self.robots:
                del self.robots[robot_name]
            print(f"❌ Robot loading failed: {robot_name} - {e}")
            return False
    
    @abstractmethod
    def _load_robot_specific(self, robot_name: str, robot_instance: Any) -> bool:
        """Subclass-specific robot loading implementation"""
        pass
    
    # Concrete implementation - no need to override unless custom logic needed
    def load_robot(self, robot_name: str, robot_instance: Any) -> bool:
        """
        Load a robot for visualization - Concrete implementation using template method
        
        Subclasses can override this if they need custom pre/post processing,
        otherwise they only need to implement _load_robot_specific()
        
        Args:
            robot_name: Robot name  
            robot_instance: Robot instance with URDF data
            
        Returns:
            bool: Success status
        """
        # Optional availability check (if subclass has 'available' attribute)
        if hasattr(self, 'available') and not self.available:
            print(f"❌ Visualizer not available")
            return False
        
        print(f"🤖 Loading robot '{robot_name}' using robot data")
        
        # Use template method for common loading logic
        return self.load_robot_common(robot_name, robot_instance)
    
    # Template method for robot visualization updates
    def update_robot_visualization(self, robot_name: str, robot_data: Any = None, force_render: bool = True) -> bool:
        """
        Update robot visualization - Template Method implementation
        
        Common logic for getting robot data and link poses, with subclass-specific
        visualization updates via _update_visualization_specific()
        
        Args:
            robot_name: Robot name to update
            robot_data: Robot instance (optional, will use stored robot if None)
            force_render: Force rendering (compatibility parameter)
            
        Returns:
            bool: Success status
        """
        return self._update_robot_visualization_common(robot_name, robot_data, force_render)
    
    def _update_robot_visualization_common(self, robot_name: str, robot_data: Any = None, force_render: bool = True) -> bool:
        """Common robot visualization update logic"""
        # Step 1: Get robot data
        if robot_data is None:
            if robot_name not in self.robots:
                return False
            robot_data = self.robots[robot_name]
        
        # Step 2: Get link poses (common pattern)
        try:
            if not hasattr(robot_data, 'get_link_poses'):
                print(f"⚠️ Robot {robot_name} has no get_link_poses method")
                return False
            
            link_poses = robot_data.get_link_poses()
            if not link_poses:
                return False
            
            # Step 3: Call subclass-specific visualization update
            return self._update_visualization_specific(robot_name, robot_data, link_poses, force_render)
            
        except Exception as e:
            print(f"❌ Visualization update failed for {robot_name}: {e}")
            return False
    
    @abstractmethod
    def _update_visualization_specific(self, robot_name: str, robot_data: Any, link_poses: Dict, force_render: bool) -> bool:
        """Subclass-specific visualization update implementation"""
        pass
    
    # Concrete implementation using template method - no need to override
    def remove_robot(self, robot_name: str) -> bool:
        """
        Remove robot from visualization - Concrete implementation using template method
        
        Subclasses can override this if they need custom pre/post processing,
        otherwise they only need to implement _remove_robot_specific()
        
        Args:
            robot_name: Robot name to remove
            
        Returns:
            bool: Success status
        """
        return self.remove_robot_common(robot_name)
    
    # Optional methods with default implementations
    
    def get_robot_info(self, robot_name: str) -> Optional[Dict]:
        """Get robot information including joint names and current positions"""
        if robot_name not in self.robots:
            return None
        
        robot = self.robots[robot_name]
        if not hasattr(robot, 'get_joint_names'):
            return None
            
        return {
            'joint_names': robot.get_joint_names(),
            'joint_positions': robot.get_joint_positions() if hasattr(robot, 'get_joint_positions') else {},
            'joint_velocities': robot.get_joint_velocities() if hasattr(robot, 'get_joint_velocities') else {},
            'movable_joints': [name for name in robot.get_joint_names() 
                             if hasattr(robot, 'joints') and robot.joints[name].joint_type.value != 'fixed']
        }
    
    # Common robot management methods - unified from both subclasses
    
    def get_robot_names(self) -> List[str]:
        """Get list of registered robot names"""
        return list(self.robots.keys())
    
    def is_robot_loaded(self, robot_name: str) -> bool:
        """Check if robot is loaded"""
        return robot_name in self.robots
    
    def get_loaded_robot_count(self) -> int:
        """Get number of loaded robots"""
        return len(self.robots)
    
    # Time management helper - unified from ProcessSeparated implementation
    def _get_current_time(self) -> float:
        """Get current time (use TimeManager if available, fallback to system time)"""
        if self._connected_time_manager and hasattr(self._connected_time_manager, 'get_sim_time'):
            return self._connected_time_manager.get_sim_time()
        else:
            import time
            return time.time()
    
    # Common robot removal template method 
    def remove_robot_common(self, robot_name: str) -> bool:
        """
        Common robot removal logic using Template Method pattern
        
        Args:
            robot_name: Robot name to remove
            
        Returns:
            bool: True if removal succeeded
        """
        if robot_name not in self.robots:
            print(f"⚠️ Robot '{robot_name}' not found for removal")
            return False
        
        try:
            # Step 1: Call subclass-specific cleanup
            success = self._remove_robot_specific(robot_name)
            
            # Step 2: Remove from base registry
            if robot_name in self.robots:
                del self.robots[robot_name]
            
            if success:
                print(f"🗑️ Robot removed: {robot_name}")
            else:
                print(f"⚠️ Robot removal partially failed: {robot_name}")
                
            return success
            
        except Exception as e:
            print(f"❌ Robot removal failed: {robot_name} - {e}")
            return False
    
    @abstractmethod
    def _remove_robot_specific(self, robot_name: str) -> bool:
        """Subclass-specific robot removal implementation"""
        pass