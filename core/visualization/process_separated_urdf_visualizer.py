#!/usr/bin/env python3
"""
Process-Separated URDF Robot Visualizer

Process-separated PyVista visualizer compliant with URDFRobotVisualizer interface
Optimized geometry + pose update architecture
"""

import numpy as np
from typing import Dict, List, Optional, Any
import warnings

# Add parent directory to path
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from .process_separated_pyvista import ProcessManager, SharedMemoryConfig
from ..simulation_object import Pose
from .base_robot_visualizer import BaseRobotVisualizer


class ProcessURDFAdapter(ProcessManager, BaseRobotVisualizer):
    """
    Process-separated visualizer compliant with URDFRobotVisualizer interface
    
    Features:
    - Full compatibility with SimulationManager
    - Uses standard PyVistaVisualizer logic
    - Optimized data transmission (geometry once + continuous pose updates)
    """
    
    def __init__(self, config: Optional[SharedMemoryConfig] = None):
        ProcessManager.__init__(self, config)
        BaseRobotVisualizer.__init__(self)
        
        # URDF robot tracking (self.robots inherited from BaseRobotVisualizer)
        self.urdf_robots = {}  # robot_name -> robot_instance (compatibility alias)
        self.robot_link_poses = {}  # robot_name -> {link_name -> pose}
        
        # Performance tracking
        self.update_count = 0
        self.last_update_time = 0
        
        # Compatibility attributes (available inherited from BaseRobotVisualizer)
        
    # Connection methods inherited from BaseRobotVisualizer
    # Store references for compatibility
    @property
    def time_manager(self):
        return self._connected_time_manager
    
    @property
    def simulation_manager(self):
        return self._connected_simulation_manager
    
    # load_robot() inherited from BaseRobotVisualizer - no override needed
    
    def _load_robot_specific(self, robot_name: str, robot_data: Any) -> bool:
        """ProcessURDFAdapter-specific loading implementation"""
        try:
            # Store compatibility alias for this visualizer
            self.urdf_robots[robot_name] = robot_data
            
            # Initialization needed
            if not self.is_initialized:
                print("🚀 Initializing visualizer...")
                init_success = self.initialize()
                if not init_success:
                    print(f"❌ Failed to initialize visualizer")
                    return False
            
            # Add robot (send geometry to process)
            success = self.add_robot(robot_name, robot_data)
            
            if success:
                # Initialize link pose management
                self.robot_link_poses[robot_name] = {}
                for link_name in robot_data.links.keys():
                    self.robot_link_poses[robot_name][link_name] = Pose()
                return True
            else:
                print(f"❌ Failed to add robot to visualizer: {robot_name}")
                return False
                
        except Exception as e:
            print(f"❌ ProcessURDFAdapter specific loading failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    # update_robot_visualization() inherited from BaseRobotVisualizer - no override needed
    # Uses Template Method pattern with _update_visualization_specific()
    
    def _update_visualization_specific(self, robot_name: str, robot_data: Any, link_poses: Dict, force_render: bool) -> bool:
        """ProcessURDFAdapter-specific visualization update implementation"""
        if robot_name not in self.urdf_robots:
            return False
            
        try:
            # Fast pose update (via shared memory)
            success = self.update_robot_poses(robot_name, link_poses)
            
            if success:
                self.update_count += 1
                self.last_update_time = self._get_current_time()
                
                # Debug output (throttled)
                # if self.update_count <= 5 or self.update_count % 100 == 0:
                #     print(f"🔄 Pose update #{self.update_count}: {robot_name} ({len(link_poses)} links)")
                
                return True
            else:
                return False
                
        except Exception as e:
            print(f"❌ ProcessURDFAdapter specific visualization update failed: {e}")
            return False
    
    # remove_robot(), get_robot_names(), is_robot_loaded() inherited from BaseRobotVisualizer
    # Uses Template Method pattern with _remove_robot_specific()
    
    def _remove_robot_specific(self, robot_name: str) -> bool:
        """ProcessURDFAdapter-specific removal implementation"""
        try:
            # Remove compatibility alias
            if robot_name in self.urdf_robots:
                del self.urdf_robots[robot_name]
                
            # Remove link poses
            if robot_name in self.robot_link_poses:
                del self.robot_link_poses[robot_name]
            
            return True
            
        except Exception as e:
            print(f"❌ ProcessURDFAdapter specific removal failed: {e}")
            return False
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """Get performance statistics"""
        if not self.is_initialized:
            return {}
            
        current_time = self._get_current_time()
        
        return {
            'update_count': self.update_count,
            'last_update_time': self.last_update_time,
            'total_robots': len(self.urdf_robots),
            'active_robots': len([r for r in self.urdf_robots.keys() if r in self.robots]),
            'current_time': current_time,
            'architecture': 'process_separated'
        }
    
    def print_performance_summary(self):
        """Display performance statistics"""
        stats = self.get_performance_stats()
        
        print("📊 ProcessURDFAdapter Performance:")
        print(f"   Total robots: {stats.get('total_robots', 0)}")
        print(f"   Active robots: {stats.get('active_robots', 0)}")
        print(f"   Update count: {stats.get('update_count', 0)}")
        print(f"   Architecture: {stats.get('architecture', 'unknown')}")
        
        if self.pose_manager:
            print(f"   Shared memory: {self.pose_manager.shm_name}")
    
    # _get_current_time() inherited from BaseRobotVisualizer


def create_process_separated_urdf_visualizer(config: Optional[SharedMemoryConfig] = None):
    """
    Factory function to create the URDF visualizer
    
    Args:
        config: Shared memory configuration
        
    Returns:
        ProcessURDFAdapter instance
    """
    return ProcessURDFAdapter(config)


if __name__ == "__main__":
    # Test the URDF visualizer
    print("🧪 Testing ProcessURDFAdapter")
    
    config = SharedMemoryConfig(max_robots=2, max_links_per_robot=8)
    visualizer = ProcessURDFAdapter(config)
    
    if visualizer.initialize():
        print("✅ Test passed - URDF visualizer initialized")
        
        # Print initial stats
        visualizer.print_performance_summary()
        
        import time
        time.sleep(3.0)
        
        visualizer.shutdown()
        print("✅ Test completed")
    else:
        print("❌ Test failed - initialization error")