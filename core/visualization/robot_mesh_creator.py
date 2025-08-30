#!/usr/bin/env python3
"""
Robot Mesh Creation Utility

Shared mesh creation logic for both URDFRobotVisualizer and ProcessURDFVisualizer
to eliminate code duplication in geometry processing.
"""

import numpy as np
from typing import Dict, Any, Tuple, Optional
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from ..simulation_object import Pose


class RobotMeshCreator:
    """
    Utility class for creating robot meshes with unified logic
    
    Provides shared mesh creation functionality for both direct visualization
    and process-separated visualization contexts.
    """
    
    def __init__(self, pv_module, logger=None):
        """
        Initialize mesh creator with PyVista module
        
        Args:
            pv_module: PyVista module reference (pyvista)
            logger: Optional logger for debug messages
        """
        self.pv = pv_module
        self.logger = logger
        
    def _log_debug(self, message: str):
        """Log debug message if logger available"""
        if self.logger:
            from ..utils.logger import log_debug
            log_debug(self.logger, message)
        
    def _log_info(self, message: str):
        """Log info message if logger available"""
        if self.logger:
            from ..utils.logger import log_info
            log_info(self.logger, message)
    
    def create_geometry_mesh(self, geometry_type: str, geometry_params: Dict[str, Any]) -> Optional[Any]:
        """
        Create PyVista mesh based on geometry type and parameters
        
        Args:
            geometry_type: Geometry type ('box', 'cylinder', 'sphere')
            geometry_params: Dictionary of geometry parameters
            
        Returns:
            PyVista mesh object or None if creation fails
        """
        mesh = None
        
        self._log_debug(f"Creating {geometry_type} mesh with params: {geometry_params}")
        
        try:
            if geometry_type == "box":
                size = geometry_params.get('size', [0.3, 0.3, 0.1])
                mesh = self.pv.Cube(x_length=size[0], y_length=size[1], z_length=size[2])
                
            elif geometry_type == "cylinder":
                radius = geometry_params.get('radius', 0.05)
                length = geometry_params.get('length', 0.35)
                mesh = self.pv.Cylinder(radius=radius, height=length, direction=(0, 0, 1))
                
            elif geometry_type == "sphere":
                radius = geometry_params.get('radius', 0.08)
                mesh = self.pv.Sphere(radius=radius)
            
        except Exception as e:
            self._log_debug(f"Failed to create {geometry_type} mesh: {e}")
            return None
            
        return mesh
    
    def apply_visual_origin_transform(self, mesh: Any, urdf_link_data: Dict[str, Any]) -> bool:
        """
        Apply URDF visual origin transformation to mesh
        
        Args:
            mesh: PyVista mesh object to transform
            urdf_link_data: URDF link data containing pose information
            
        Returns:
            bool: True if transformation applied successfully
        """
        try:
            if not urdf_link_data.get('has_pose', False):
                return True
                
            position = np.array(urdf_link_data['position'])
            quaternion = np.array(urdf_link_data['quaternion'])
            
            # Recreate Pose and transform matrix
            pose = Pose.from_position_quaternion(position, quaternion)
            transform_matrix = pose.to_transformation_matrix()
            mesh.transform(transform_matrix, inplace=True)
            
            self._log_debug(f"Applied visual origin transform: pos={position}")
            return True
            
        except Exception as e:
            self._log_debug(f"Failed to apply visual origin transform: {e}")
            return False
    
    def create_robot_link_mesh(self, link_name: str, link_data: Dict[str, Any], 
                             urdf_link_data: Optional[Dict[str, Any]] = None) -> Tuple[Optional[Any], Tuple[float, float, float]]:
        """
        Create complete robot link mesh with transformations
        
        Args:
            link_name: Name of the robot link
            link_data: Link data containing geometry_type, geometry_params, color
            urdf_link_data: Optional URDF link data for visual origin transform
            
        Returns:
            Tuple of (mesh object or None, color RGB tuple)
        """
        geometry_type = link_data.get('geometry_type', 'box')
        geometry_params = link_data.get('geometry_params', {})
        color = link_data.get('color', [0.6, 0.6, 0.6, 1.0])
        
        # Step 1: Create base mesh
        mesh = self.create_geometry_mesh(geometry_type, geometry_params)
        if mesh is None:
            return None, (0.6, 0.6, 0.6)
        
        # Step 2: Apply visual origin transformation if available
        if urdf_link_data:
            success = self.apply_visual_origin_transform(mesh, urdf_link_data)
            if not success:
                self._log_debug(f"Visual origin transform failed for {link_name}")
        
        # Step 3: Extract RGB color
        color_rgb = color[:3] if len(color) >= 3 else (0.6, 0.6, 0.6)
        
        self._log_debug(f"Created {geometry_type} mesh for {link_name} with color {color_rgb}")
        return mesh, color_rgb
    
    def add_mesh_to_plotter(self, plotter: Any, mesh: Any, robot_name: str, 
                          link_name: str, color_rgb: Tuple[float, float, float]) -> Any:
        """
        Add mesh to PyVista plotter with standardized parameters
        
        Args:
            plotter: PyVista plotter object
            mesh: PyVista mesh to add
            robot_name: Robot name for actor naming
            link_name: Link name for actor naming  
            color_rgb: RGB color tuple
            
        Returns:
            PyVista actor object
        """
        actor = plotter.add_mesh(
            mesh,
            color=color_rgb,
            opacity=1.0,  # Standardized opaque rendering
            name=f"{robot_name}_{link_name}"
        )
        
        self._log_info(f"Added mesh to plotter: {robot_name}_{link_name} with color {color_rgb}")
        return actor


def create_robot_mesh_creator(pv_module, logger=None) -> RobotMeshCreator:
    """
    Factory function to create RobotMeshCreator instance
    
    Args:
        pv_module: PyVista module reference
        logger: Optional logger for debug output
        
    Returns:
        RobotMeshCreator instance
    """
    return RobotMeshCreator(pv_module, logger)