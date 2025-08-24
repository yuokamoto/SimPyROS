#!/usr/bin/env python3
"""
Robot Link Connector for SimPyROS

Enhanced connection system that allows objects to be attached to specific robot links.
Connected objects follow both robot base movement and individual link movement from joint motion.

Features:
- Link-specific object attachment
- Hierarchical motion tracking  
- Automatic pose updates
- Multiple attachment modes
"""

import time
from typing import Dict, List, Optional, Tuple, Any, Union
from dataclasses import dataclass
import warnings
import threading

from core.simulation_object import SimulationObject, Pose
from core.robot import Robot


@dataclass
class LinkConnection:
    """Information about an object connected to a robot link"""
    object: SimulationObject
    relative_pose: Pose
    connection_mode: str = "rigid"  # 'rigid', 'flexible', 'sensor'
    update_frequency: float = 60.0  # Hz
    last_update: float = 0.0
    active: bool = True


class RobotLinkConnector:
    """
    Enhanced connection system for attaching objects to robot links
    
    This system extends the basic simulation_object connection system to support
    attachment to specific robot links. Connected objects will follow:
    1. Robot base movement (translation, rotation)
    2. Joint motion that affects the specific link
    
    Connection modes:
    - 'rigid': Object rigidly attached, follows all motion exactly
    - 'flexible': Object attached with some compliance/damping  
    - 'sensor': Object represents a sensor, may have different update rate
    """
    
    def __init__(self):
        """Initialize robot link connector"""
        # Global registry of all link connections
        self.connections: Dict[Robot, Dict[str, List[LinkConnection]]] = {}
        
        # Update management
        self._update_thread: Optional[threading.Thread] = None
        self._running = False
        self._update_frequency = 120.0  # High frequency for smooth motion
        
        # Performance tracking
        self._update_count = 0
        self._last_performance_check = 0.0
        
        print("🔗 Robot Link Connector initialized")
    
    def connect_object_to_link(self,
                              obj: SimulationObject,
                              robot: Robot, 
                              link_name: str,
                              relative_pose: Optional[Pose] = None,
                              connection_mode: str = "rigid",
                              update_frequency: float = 60.0) -> bool:
        """
        Connect object to specific robot link
        
        Args:
            obj: Object to connect
            robot: Target robot
            link_name: Name of the link to attach to
            relative_pose: Pose relative to link (default: identity)
            connection_mode: Connection behavior ('rigid', 'flexible', 'sensor')
            update_frequency: Update frequency in Hz
            
        Returns:
            Success status
        """
        # Validate inputs
        if link_name not in robot.get_link_names():
            print(f"❌ Link '{link_name}' not found in robot {robot.robot_name}")
            print(f"Available links: {robot.get_link_names()}")
            return False
        
        if obj is robot:
            raise ValueError("Cannot connect robot to itself")
        
        if connection_mode not in ['rigid', 'flexible', 'sensor']:
            raise ValueError(f"Invalid connection mode: {connection_mode}")
        
        # Use identity pose if none specified
        if relative_pose is None:
            relative_pose = Pose()
        
        # Create connection info
        connection = LinkConnection(
            object=obj,
            relative_pose=relative_pose.copy(),
            connection_mode=connection_mode,
            update_frequency=update_frequency
        )
        
        # Add to registry
        if robot not in self.connections:
            self.connections[robot] = {}
        
        if link_name not in self.connections[robot]:
            self.connections[robot][link_name] = []
        
        # Check for existing connection
        for existing in self.connections[robot][link_name]:
            if existing.object is obj:
                print(f"⚠️ Object already connected to {robot.robot_name}:{link_name}, updating...")
                existing.relative_pose = relative_pose.copy()
                existing.connection_mode = connection_mode
                existing.update_frequency = update_frequency
                existing.active = True
                return True
        
        # Add new connection
        self.connections[robot][link_name].append(connection)
        
        # Set initial pose
        self._update_object_pose(robot, link_name, connection)
        
        # Start update thread if not running
        if not self._running:
            self.start_update_loop()
        
        print(f"✅ Connected object to {robot.robot_name}:{link_name} ({connection_mode} mode)")
        return True
    
    def disconnect_object_from_link(self,
                                   obj: SimulationObject,
                                   robot: Robot,
                                   link_name: str) -> bool:
        """
        Disconnect object from robot link
        
        Args:
            obj: Object to disconnect
            robot: Robot containing the link
            link_name: Link name
            
        Returns:
            Success status
        """
        if robot not in self.connections:
            return False
        
        if link_name not in self.connections[robot]:
            return False
        
        # Find and remove connection
        connections = self.connections[robot][link_name]
        for i, connection in enumerate(connections):
            if connection.object is obj:
                connections.pop(i)
                print(f"✅ Disconnected object from {robot.robot_name}:{link_name}")
                
                # Clean up empty entries
                if not connections:
                    del self.connections[robot][link_name]
                    if not self.connections[robot]:
                        del self.connections[robot]
                
                return True
        
        print(f"⚠️ Object not found in {robot.robot_name}:{link_name} connections")
        return False
    
    def disconnect_all_from_robot(self, robot: Robot) -> int:
        """Disconnect all objects from robot, return count"""
        if robot not in self.connections:
            return 0
        
        total_disconnected = 0
        for link_name, connections in self.connections[robot].items():
            total_disconnected += len(connections)
        
        del self.connections[robot]
        print(f"✅ Disconnected {total_disconnected} objects from {robot.robot_name}")
        return total_disconnected
    
    def disconnect_all_from_object(self, obj: SimulationObject) -> int:
        """Disconnect object from all robot links, return count"""
        disconnected_count = 0
        
        # Search all connections
        robots_to_update = []
        for robot, robot_connections in self.connections.items():
            links_to_update = []
            for link_name, connections in robot_connections.items():
                connections_to_remove = []
                for i, connection in enumerate(connections):
                    if connection.object is obj:
                        connections_to_remove.append(i)
                
                # Remove in reverse order to maintain indices
                for i in reversed(connections_to_remove):
                    connections.pop(i)
                    disconnected_count += 1
                
                if not connections:
                    links_to_update.append(link_name)
            
            # Clean up empty link entries
            for link_name in links_to_update:
                del robot_connections[link_name]
            
            if not robot_connections:
                robots_to_update.append(robot)
        
        # Clean up empty robot entries
        for robot in robots_to_update:
            del self.connections[robot]
        
        if disconnected_count > 0:
            print(f"✅ Disconnected object from {disconnected_count} robot links")
        
        return disconnected_count
    
    def _update_object_pose(self, robot: Robot, link_name: str, connection: LinkConnection):
        """Update pose of connected object based on current link pose"""
        try:
            # Get current link pose in world coordinates
            link_pose = robot.get_link_pose(link_name)
            if link_pose is None:
                print(f"⚠️ Could not get pose for link {link_name}")
                return
            
            # Calculate world pose of object
            if connection.connection_mode == "rigid":
                # Rigid attachment - exact transformation
                world_pose = connection.relative_pose.transform_by(link_pose)
                
            elif connection.connection_mode == "flexible":
                # Flexible attachment - add some damping/filtering
                target_pose = connection.relative_pose.transform_by(link_pose)
                current_pose = connection.object.get_pose()
                
                # Simple low-pass filter for smoother motion
                alpha = 0.8  # Filter coefficient
                filtered_position = alpha * target_pose.position + (1 - alpha) * current_pose.position
                filtered_rotation = current_pose.rotation.slerp(target_pose.rotation, alpha)
                
                world_pose = Pose.from_position_rotation(filtered_position, filtered_rotation)
                
            elif connection.connection_mode == "sensor":
                # Sensor attachment - may have different behavior
                world_pose = connection.relative_pose.transform_by(link_pose)
                # Could add sensor-specific processing here
                
            else:
                world_pose = connection.relative_pose.transform_by(link_pose)
            
            # Update object pose
            connection.object.teleport(world_pose)
            connection.last_update = time.time()
            
        except Exception as e:
            print(f"⚠️ Error updating object pose for {robot.robot_name}:{link_name}: {e}")
    
    def _update_all_connections(self):
        """Update all active connections"""
        current_time = time.time()
        updates_performed = 0
        
        for robot, robot_connections in self.connections.items():
            for link_name, connections in robot_connections.items():
                for connection in connections:
                    if not connection.active:
                        continue
                    
                    # Check if update is due based on frequency
                    update_interval = 1.0 / connection.update_frequency
                    if (current_time - connection.last_update) >= update_interval:
                        self._update_object_pose(robot, link_name, connection)
                        updates_performed += 1
        
        self._update_count += updates_performed
        
        # Performance reporting
        if current_time - self._last_performance_check > 5.0:  # Every 5 seconds
            if self._last_performance_check > 0:
                avg_updates = self._update_count / 5.0
                connection_count = sum(len(connections) 
                                     for robot_conns in self.connections.values()
                                     for connections in robot_conns.values())
                print(f"🔗 Link connector: {connection_count} connections, {avg_updates:.1f} updates/sec")
            
            self._update_count = 0
            self._last_performance_check = current_time
    
    def _update_loop(self):
        """Main update loop running in separate thread"""
        dt = 1.0 / self._update_frequency
        
        while self._running:
            loop_start = time.time()
            
            try:
                self._update_all_connections()
            except Exception as e:
                print(f"⚠️ Link connector update error: {e}")
            
            # Maintain update rate
            elapsed = time.time() - loop_start
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def start_update_loop(self):
        """Start the connection update loop"""
        if self._running:
            return
        
        self._running = True
        self._update_thread = threading.Thread(
            target=self._update_loop,
            name="LinkConnectorUpdate"
        )
        self._update_thread.daemon = True
        self._update_thread.start()
        
        print(f"🚀 Link connector update loop started ({self._update_frequency} Hz)")
    
    def stop_update_loop(self):
        """Stop the connection update loop"""
        if not self._running:
            return
        
        self._running = False
        
        if self._update_thread and self._update_thread.is_alive():
            self._update_thread.join(timeout=1.0)
            if self._update_thread.is_alive():
                print("⚠️ Link connector thread didn't shut down gracefully")
        
        print("🛑 Link connector update loop stopped")
    
    def get_connection_info(self, robot: Robot, link_name: str) -> List[Dict]:
        """Get information about connections to specific link"""
        if robot not in self.connections or link_name not in self.connections[robot]:
            return []
        
        info = []
        for connection in self.connections[robot][link_name]:
            info.append({
                'object_name': getattr(connection.object, 'name', 'unnamed'),
                'relative_pose': connection.relative_pose,
                'connection_mode': connection.connection_mode,
                'update_frequency': connection.update_frequency,
                'last_update': connection.last_update,
                'active': connection.active
            })
        
        return info
    
    def get_all_connections(self) -> Dict[str, Dict[str, List[Dict]]]:
        """Get information about all connections"""
        all_connections = {}
        
        for robot, robot_connections in self.connections.items():
            robot_name = getattr(robot, 'robot_name', 'unnamed')
            all_connections[robot_name] = {}
            
            for link_name, connections in robot_connections.items():
                all_connections[robot_name][link_name] = self.get_connection_info(robot, link_name)
        
        return all_connections
    
    def print_connection_status(self):
        """Print comprehensive connection status"""
        total_connections = sum(len(connections)
                              for robot_conns in self.connections.values()
                              for connections in robot_conns.values())
        
        print(f"\n🔗 Link Connector Status")
        print(f"Total connections: {total_connections}")
        print(f"Update frequency: {self._update_frequency} Hz")
        print(f"Running: {self._running}")
        print("=" * 50)
        
        for robot, robot_connections in self.connections.items():
            robot_name = getattr(robot, 'robot_name', 'unnamed')
            print(f"\n🤖 Robot: {robot_name}")
            
            for link_name, connections in robot_connections.items():
                print(f"  📎 Link: {link_name} ({len(connections)} objects)")
                
                for i, connection in enumerate(connections):
                    obj_name = getattr(connection.object, 'name', f'object_{i}')
                    status = "✅" if connection.active else "⏸️"
                    print(f"    {status} {obj_name} ({connection.connection_mode}, "
                          f"{connection.update_frequency}Hz)")
    
    def set_connection_active(self, obj: SimulationObject, robot: Robot, 
                            link_name: str, active: bool) -> bool:
        """Enable/disable specific connection"""
        if robot not in self.connections or link_name not in self.connections[robot]:
            return False
        
        for connection in self.connections[robot][link_name]:
            if connection.object is obj:
                connection.active = active
                status = "enabled" if active else "disabled"
                print(f"🔄 Connection {status}: {robot.robot_name}:{link_name}")
                return True
        
        return False
    
    def cleanup(self):
        """Clean up all connections and stop update loop"""
        self.stop_update_loop()
        
        total_cleaned = 0
        for robot_connections in self.connections.values():
            for connections in robot_connections.values():
                total_cleaned += len(connections)
        
        self.connections.clear()
        print(f"🗑️ Cleaned up {total_cleaned} link connections")


# Global instance for convenience
_global_link_connector: Optional[RobotLinkConnector] = None


def get_link_connector() -> RobotLinkConnector:
    """Get or create global link connector instance"""
    global _global_link_connector
    if _global_link_connector is None:
        _global_link_connector = RobotLinkConnector()
    return _global_link_connector


def connect_to_robot_link(obj: SimulationObject,
                         robot: Robot,
                         link_name: str,
                         relative_pose: Optional[Pose] = None,
                         connection_mode: str = "rigid") -> bool:
    """
    Convenience function to connect object to robot link
    
    Args:
        obj: Object to connect
        robot: Target robot
        link_name: Link name
        relative_pose: Relative pose (default: identity)
        connection_mode: Connection mode
        
    Returns:
        Success status
    """
    connector = get_link_connector()
    return connector.connect_object_to_link(obj, robot, link_name, relative_pose, connection_mode)


def disconnect_from_robot_link(obj: SimulationObject,
                              robot: Robot,
                              link_name: str) -> bool:
    """Convenience function to disconnect object from robot link"""
    connector = get_link_connector()
    return connector.disconnect_object_from_link(obj, robot, link_name)


# Extension methods for Robot class
def _robot_connect_object_to_link(self: Robot,
                                 obj: SimulationObject,
                                 link_name: str,
                                 relative_pose: Optional[Pose] = None,
                                 connection_mode: str = "rigid") -> bool:
    """Connect external object to specific link (Robot method extension)"""
    return connect_to_robot_link(obj, self, link_name, relative_pose, connection_mode)


def _robot_disconnect_object_from_link(self: Robot,
                                      obj: SimulationObject,
                                      link_name: str) -> bool:
    """Disconnect object from specific link (Robot method extension)"""
    return disconnect_from_robot_link(obj, self, link_name)


def _robot_get_link_connections(self: Robot, link_name: str) -> List[Dict]:
    """Get objects connected to specific link (Robot method extension)"""
    connector = get_link_connector()
    return connector.get_connection_info(self, link_name)


def _robot_disconnect_all_objects(self: Robot) -> int:
    """Disconnect all objects from this robot (Robot method extension)"""
    connector = get_link_connector()
    return connector.disconnect_all_from_robot(self)


# Monkey patch Robot class with new methods
def extend_robot_class():
    """Add link connection methods to Robot class"""
    Robot.connect_object_to_link = _robot_connect_object_to_link
    Robot.disconnect_object_from_link = _robot_disconnect_object_from_link  
    Robot.get_link_connections = _robot_get_link_connections
    Robot.disconnect_all_objects = _robot_disconnect_all_objects
    
    print("🔧 Extended Robot class with link connection methods")


# Auto-extend Robot class when module is imported
extend_robot_class()