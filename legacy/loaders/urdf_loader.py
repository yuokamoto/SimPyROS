#!/usr/bin/env python3
"""
URDF/SDF Loader for SimPyROS
Provides functionality to load robot models from URDF/SDF files
"""

import os
import numpy as np
from typing import Optional, Dict, List, Tuple, Union
from dataclasses import dataclass
import warnings

# Handle optional dependencies gracefully
try:
    import urdfpy
    URDFPY_AVAILABLE = True
except ImportError:
    URDFPY_AVAILABLE = False
    urdfpy = None

try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False
    trimesh = None

from simulation_object import Pose, Velocity


@dataclass
class RobotLink:
    """Represents a robot link with mesh and transform information"""
    name: str
    mesh_path: Optional[str] = None
    mesh: Optional[object] = None  # Will hold trimesh or pyvista mesh
    pose: Optional[Pose] = None
    color: Tuple[float, float, float, float] = (0.8, 0.4, 0.0, 1.0)  # RGBA
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    

@dataclass
class RobotJoint:
    """Represents a robot joint"""
    name: str
    joint_type: str  # 'fixed', 'revolute', 'prismatic', etc.
    parent_link: str
    child_link: str
    axis: np.ndarray = None
    limits: Tuple[float, float] = None  # (lower, upper)
    pose: Optional[Pose] = None


class URDFLoader:
    """
    URDF/SDF file loader for robot models
    
    Features:
    - Load URDF files using urdfpy
    - Extract mesh information and convert to PyVista
    - Handle different mesh formats (STL, DAE, OBJ)
    - Convert to SimPyROS compatible format
    """
    
    def __init__(self, package_path: Optional[str] = None):
        """
        Initialize URDF loader
        
        Args:
            package_path: Optional path to ROS package directory for resolving package:// URIs
        """
        self.package_path = package_path
        self.robot = None
        self.links = {}
        self.joints = {}
        
        # Check dependencies
        if not URDFPY_AVAILABLE:
            warnings.warn("urdfpy not available. Install with: pip install urdfpy")
        if not TRIMESH_AVAILABLE:
            warnings.warn("trimesh not available. Install with: pip install trimesh")
    
    def is_available(self) -> bool:
        """Check if URDF loading is available"""
        return URDFPY_AVAILABLE and TRIMESH_AVAILABLE
    
    def load_urdf(self, urdf_path: str) -> bool:
        """
        Load a URDF file
        
        Args:
            urdf_path: Path to the URDF file
            
        Returns:
            True if successfully loaded, False otherwise
        """
        if not self.is_available():
            print("URDF loading not available. Missing dependencies.")
            return False
            
        try:
            print(f"Loading URDF file: {urdf_path}")
            self.robot = urdfpy.URDF.load(urdf_path)
            
            # Extract links
            for link in self.robot.links:
                self._process_link(link)
            
            # Extract joints  
            for joint in self.robot.joints:
                self._process_joint(joint)
                
            print(f"Successfully loaded URDF with {len(self.links)} links and {len(self.joints)} joints")
            return True
            
        except Exception as e:
            print(f"Error loading URDF {urdf_path}: {e}")
            return False
    
    def _process_link(self, urdf_link):
        """Process a URDF link and extract mesh information"""
        link = RobotLink(name=urdf_link.name)
        
        # Process visual elements
        if urdf_link.visuals:
            for visual in urdf_link.visuals:
                if visual.geometry and hasattr(visual.geometry, 'mesh') and visual.geometry.mesh:
                    mesh_info = visual.geometry.mesh
                    
                    # Get mesh file path
                    filename = mesh_info.filename
                    if filename:
                        # Resolve package:// URIs if needed
                        resolved_path = self._resolve_mesh_path(filename)
                        if resolved_path and os.path.exists(resolved_path):
                            link.mesh_path = resolved_path
                            
                            # Load mesh using trimesh
                            try:
                                mesh = trimesh.load(resolved_path)
                                if mesh_info.scale is not None:
                                    scale = np.array(mesh_info.scale)
                                    link.scale = tuple(scale)
                                    mesh.apply_scale(scale)
                                link.mesh = mesh
                                print(f"  Loaded mesh for link '{urdf_link.name}': {resolved_path}")
                            except Exception as e:
                                print(f"  Warning: Could not load mesh {resolved_path}: {e}")
                
                # Get material color if available
                if visual.material and visual.material.color is not None:
                    link.color = tuple(visual.material.color)
        
        # Get link pose (transform from parent)
        # Note: In URDF, link poses are typically defined by joint transforms
        link.pose = Pose()  # Identity pose by default
        
        self.links[urdf_link.name] = link
    
    def _process_joint(self, urdf_joint):
        """Process a URDF joint"""
        joint = RobotJoint(
            name=urdf_joint.name,
            joint_type=urdf_joint.joint_type,
            parent_link=urdf_joint.parent,
            child_link=urdf_joint.child
        )
        
        # Get joint axis
        if urdf_joint.axis is not None:
            joint.axis = np.array(urdf_joint.axis)
        
        # Get joint limits
        if urdf_joint.limit is not None:
            joint.limits = (urdf_joint.limit.lower, urdf_joint.limit.upper)
        
        # Get joint origin (pose)
        if urdf_joint.origin is not None:
            # Convert from transformation matrix to Pose
            transform = urdf_joint.origin
            position = transform[:3, 3]
            
            # Extract rotation from transformation matrix
            from scipy.spatial.transform import Rotation
            rotation_matrix = transform[:3, :3]
            rotation = Rotation.from_matrix(rotation_matrix)
            
            joint.pose = Pose.from_position_rotation(position, rotation)
        else:
            joint.pose = Pose()
            
        self.joints[urdf_joint.name] = joint
    
    def _resolve_mesh_path(self, mesh_filename: str) -> Optional[str]:
        """
        Resolve mesh file path, handling package:// URIs
        
        Args:
            mesh_filename: Original mesh filename from URDF
            
        Returns:
            Resolved absolute path or None if not found
        """
        if mesh_filename.startswith('package://'):
            if self.package_path is None:
                print(f"Warning: package:// URI found but no package_path set: {mesh_filename}")
                return None
            
            # Remove package:// and resolve relative to package path
            relative_path = mesh_filename[10:]  # Remove 'package://'
            # Take the first part as package name
            parts = relative_path.split('/', 1)
            if len(parts) < 2:
                return None
            
            package_name = parts[0]
            file_path = parts[1]
            
            # Try to find the file
            full_path = os.path.join(self.package_path, package_name, file_path)
            if os.path.exists(full_path):
                return full_path
            
            # Alternative: look in common mesh directories
            for mesh_dir in ['meshes', 'mesh', 'models']:
                alt_path = os.path.join(self.package_path, package_name, mesh_dir, file_path)
                if os.path.exists(alt_path):
                    return alt_path
            
            print(f"Warning: Could not resolve package path: {mesh_filename}")
            return None
        
        elif os.path.isabs(mesh_filename):
            # Absolute path
            return mesh_filename if os.path.exists(mesh_filename) else None
        
        else:
            # Relative path - assume relative to URDF file location
            return mesh_filename if os.path.exists(mesh_filename) else None
    
    def get_combined_mesh(self, pv_module) -> Optional[object]:
        """
        Create a combined PyVista mesh from all robot links
        
        Args:
            pv_module: PyVista module instance
            
        Returns:
            Combined PyVista mesh or None
        """
        if not self.links:
            print("No links loaded")
            return None
        
        combined_mesh = None
        
        for link_name, link in self.links.items():
            if link.mesh is None:
                continue
                
            try:
                # Convert trimesh to PyVista
                if hasattr(link.mesh, 'vertices') and hasattr(link.mesh, 'faces'):
                    # Create PyVista mesh from vertices and faces
                    vertices = link.mesh.vertices
                    faces = link.mesh.faces
                    
                    # PyVista faces format: [n_points, point1, point2, ..., point_n]
                    pv_faces = []
                    for face in faces:
                        pv_faces.extend([len(face)] + face.tolist())
                    
                    pv_mesh = pv_module.PolyData(vertices, pv_faces)
                    
                    # Apply link pose if needed
                    if link.pose and not (np.allclose(link.pose.position, 0) and 
                                        np.allclose(link.pose.rotation.as_matrix(), np.eye(3))):
                        transform_matrix = link.pose.to_transformation_matrix()
                        pv_mesh.transform(transform_matrix, inplace=True)
                    
                    # Combine with existing mesh
                    if combined_mesh is None:
                        combined_mesh = pv_mesh
                    else:
                        combined_mesh = combined_mesh + pv_mesh
                        
                    print(f"  Added mesh for link: {link_name}")
                    
            except Exception as e:
                print(f"Error converting mesh for link {link_name}: {e}")
                continue
        
        return combined_mesh
    
    def get_link_meshes(self, pv_module) -> Dict[str, object]:
        """
        Get individual PyVista meshes for each link
        
        Args:
            pv_module: PyVista module instance
            
        Returns:
            Dictionary of link_name -> PyVista mesh
        """
        link_meshes = {}
        
        for link_name, link in self.links.items():
            if link.mesh is None:
                continue
                
            try:
                # Convert trimesh to PyVista
                vertices = link.mesh.vertices
                faces = link.mesh.faces
                
                pv_faces = []
                for face in faces:
                    pv_faces.extend([len(face)] + face.tolist())
                
                pv_mesh = pv_module.PolyData(vertices, pv_faces)
                
                # Apply link pose
                if link.pose and not (np.allclose(link.pose.position, 0) and 
                                    np.allclose(link.pose.rotation.as_matrix(), np.eye(3))):
                    transform_matrix = link.pose.to_transformation_matrix()
                    pv_mesh.transform(transform_matrix, inplace=True)
                
                link_meshes[link_name] = pv_mesh
                
            except Exception as e:
                print(f"Error converting mesh for link {link_name}: {e}")
                continue
        
        return link_meshes
    
    def print_info(self):
        """Print information about loaded robot"""
        if not self.robot:
            print("No robot loaded")
            return
            
        print(f"\nRobot Model Information:")
        print(f"Name: {getattr(self.robot, 'name', 'Unknown')}")
        print(f"Links: {len(self.links)}")
        print(f"Joints: {len(self.joints)}")
        
        print(f"\nLinks:")
        for name, link in self.links.items():
            mesh_info = f"mesh: {link.mesh_path}" if link.mesh_path else "no mesh"
            print(f"  - {name}: {mesh_info}")
        
        print(f"\nJoints:")
        for name, joint in self.joints.items():
            print(f"  - {name}: {joint.joint_type} ({joint.parent_link} -> {joint.child_link})")


def create_simple_robot_urdf(output_path: str = "simple_robot.urdf") -> str:
    """
    Create a simple test URDF file for demonstration
    
    Args:
        output_path: Where to save the URDF file
        
    Returns:
        Path to created URDF file
    """
    urdf_content = '''<?xml version="1.0"?>
<robot name="simple_robot">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="orange">
        <color rgba="1.0 0.5 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.6"/>
      </geometry>
    </collision>
  </link>
  
  <!-- End Effector Link -->
  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Joints -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <joint name="arm_to_end" type="revolute">
    <parent link="arm_link"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.78" upper="0.78" effort="50" velocity="1.0"/>
  </joint>
  
</robot>'''
    
    try:
        with open(output_path, 'w') as f:
            f.write(urdf_content)
        print(f"Created simple test URDF: {output_path}")
        return output_path
    except Exception as e:
        print(f"Error creating URDF file: {e}")
        return ""


if __name__ == "__main__":
    # Test the URDF loader
    loader = URDFLoader()
    
    if not loader.is_available():
        print("URDF loader dependencies not available")
        print("Install with: pip install urdfpy trimesh")
    else:
        # Create a test URDF
        test_urdf = create_simple_robot_urdf("test_robot.urdf")
        if test_urdf:
            # Load and display info
            if loader.load_urdf(test_urdf):
                loader.print_info()