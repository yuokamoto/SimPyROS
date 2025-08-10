#!/usr/bin/env python3
"""
Simple URDF Loader for SimPyROS
Lightweight URDF parsing without heavy dependencies
"""

import os
import xml.etree.ElementTree as ET
import numpy as np
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass
from simulation_object import Pose
from scipy.spatial.transform import Rotation


@dataclass
class SimpleRobotLink:
    """Simple representation of a robot link"""
    name: str
    geometry_type: str  # 'box', 'cylinder', 'sphere', 'mesh'
    geometry_data: dict  # Contains size/radius/etc parameters
    color: Tuple[float, float, float, float] = (0.8, 0.4, 0.0, 1.0)
    pose: Optional[Pose] = None


@dataclass 
class SimpleRobotJoint:
    """Simple representation of a robot joint"""
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    pose: Optional[Pose] = None


class SimpleURDFLoader:
    """
    Simple URDF loader using only XML parsing
    No dependency on urdfpy or trimesh
    """
    
    def __init__(self):
        self.robot_name = ""
        self.links = {}
        self.joints = {}
        self.loaded = False
    
    def load_urdf(self, urdf_path: str) -> bool:
        """Load URDF file using XML parser"""
        if not os.path.exists(urdf_path):
            print(f"URDF file not found: {urdf_path}")
            return False
        
        try:
            print(f"Loading URDF file: {urdf_path}")
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            
            if root.tag != 'robot':
                print("Invalid URDF: root element must be 'robot'")
                return False
            
            self.robot_name = root.get('name', 'unknown')
            print(f"Robot name: {self.robot_name}")
            
            # Parse links
            for link_elem in root.findall('link'):
                link = self._parse_link(link_elem)
                if link:
                    self.links[link.name] = link
            
            # Parse joints
            for joint_elem in root.findall('joint'):
                joint = self._parse_joint(joint_elem)
                if joint:
                    self.joints[joint.name] = joint
            
            self.loaded = True
            print(f"Successfully loaded {len(self.links)} links and {len(self.joints)} joints")
            return True
            
        except Exception as e:
            print(f"Error parsing URDF: {e}")
            return False
    
    def _parse_link(self, link_elem) -> Optional[SimpleRobotLink]:
        """Parse a link element"""
        name = link_elem.get('name')
        if not name:
            return None
        
        link = SimpleRobotLink(name=name, geometry_type='box', geometry_data={})
        
        # Parse visual elements
        visual = link_elem.find('visual')
        if visual is not None:
            geometry = visual.find('geometry')
            if geometry is not None:
                # Check for different geometry types
                box = geometry.find('box')
                cylinder = geometry.find('cylinder')
                sphere = geometry.find('sphere')
                mesh = geometry.find('mesh')
                
                if box is not None:
                    size_str = box.get('size', '1 1 1')
                    size = [float(x) for x in size_str.split()]
                    link.geometry_type = 'box'
                    link.geometry_data = {'size': size}
                    
                elif cylinder is not None:
                    radius = float(cylinder.get('radius', 0.1))
                    length = float(cylinder.get('length', 1.0))
                    link.geometry_type = 'cylinder'
                    link.geometry_data = {'radius': radius, 'length': length}
                    
                elif sphere is not None:
                    radius = float(sphere.get('radius', 0.1))
                    link.geometry_type = 'sphere'
                    link.geometry_data = {'radius': radius}
                    
                elif mesh is not None:
                    filename = mesh.get('filename', '')
                    scale = mesh.get('scale', '1 1 1')
                    scale_vals = [float(x) for x in scale.split()]
                    link.geometry_type = 'mesh'
                    link.geometry_data = {'filename': filename, 'scale': scale_vals}
            
            # Parse material/color
            material = visual.find('material')
            if material is not None:
                color = material.find('color')
                if color is not None:
                    rgba_str = color.get('rgba', '0.8 0.4 0.0 1.0')
                    rgba = [float(x) for x in rgba_str.split()]
                    link.color = tuple(rgba)
        
        return link
    
    def _parse_joint(self, joint_elem) -> Optional[SimpleRobotJoint]:
        """Parse a joint element"""
        name = joint_elem.get('name')
        joint_type = joint_elem.get('type')
        
        if not name or not joint_type:
            return None
        
        parent = joint_elem.find('parent')
        child = joint_elem.find('child')
        
        if parent is None or child is None:
            return None
        
        parent_link = parent.get('link')
        child_link = child.get('link')
        
        joint = SimpleRobotJoint(
            name=name,
            joint_type=joint_type,
            parent_link=parent_link,
            child_link=child_link
        )
        
        # Parse origin (pose)
        origin = joint_elem.find('origin')
        if origin is not None:
            xyz_str = origin.get('xyz', '0 0 0')
            rpy_str = origin.get('rpy', '0 0 0')
            
            xyz = [float(x) for x in xyz_str.split()]
            rpy = [float(x) for x in rpy_str.split()]
            
            joint.pose = Pose(x=xyz[0], y=xyz[1], z=xyz[2], 
                             roll=rpy[0], pitch=rpy[1], yaw=rpy[2])
        
        return joint
    
    def create_pyvista_mesh(self, pv_module):
        """Create PyVista mesh from parsed URDF"""
        if not self.loaded:
            return None
        
        combined_mesh = None
        
        for joint_name, joint in self.joints.items():
            child_link = self.links.get(joint.child_link)
            if not child_link:
                continue
            
            # Create mesh based on geometry type
            mesh_part = None
            
            if child_link.geometry_type == 'box':
                size = child_link.geometry_data.get('size', [1, 1, 1])
                mesh_part = pv_module.Box(bounds=[
                    -size[0]/2, size[0]/2,
                    -size[1]/2, size[1]/2, 
                    -size[2]/2, size[2]/2
                ])
                
            elif child_link.geometry_type == 'cylinder':
                radius = child_link.geometry_data.get('radius', 0.1)
                length = child_link.geometry_data.get('length', 1.0)
                mesh_part = pv_module.Cylinder(
                    center=[0, 0, 0],
                    direction=[0, 0, 1],
                    radius=radius,
                    height=length
                )
                
            elif child_link.geometry_type == 'sphere':
                radius = child_link.geometry_data.get('radius', 0.1)
                mesh_part = pv_module.Sphere(radius=radius)
            
            if mesh_part and joint.pose:
                # Apply joint transform
                transform_matrix = joint.pose.to_transformation_matrix()
                mesh_part.transform(transform_matrix, inplace=True)
                
                if combined_mesh is None:
                    combined_mesh = mesh_part
                else:
                    combined_mesh = combined_mesh + mesh_part
        
        return combined_mesh
    
    def print_info(self):
        """Print robot information"""
        if not self.loaded:
            print("No robot loaded")
            return
        
        print(f"\n🤖 Robot Information (Simple Parser)")
        print(f"Name: {self.robot_name}")
        print(f"Links: {len(self.links)}")
        print(f"Joints: {len(self.joints)}")
        
        print(f"\n📦 Links:")
        for name, link in self.links.items():
            geometry = f"{link.geometry_type}"
            if link.geometry_type == 'box':
                size = link.geometry_data.get('size', [0, 0, 0])
                geometry += f" ({size[0]:.2f} × {size[1]:.2f} × {size[2]:.2f})"
            elif link.geometry_type == 'cylinder':
                r = link.geometry_data.get('radius', 0)
                h = link.geometry_data.get('length', 0)
                geometry += f" (r={r:.2f}, h={h:.2f})"
            elif link.geometry_type == 'sphere':
                r = link.geometry_data.get('radius', 0)
                geometry += f" (r={r:.2f})"
            
            print(f"  - {name}: {geometry}")
        
        print(f"\n🔗 Joints:")
        for name, joint in self.joints.items():
            print(f"  - {name}: {joint.joint_type} ({joint.parent_link} → {joint.child_link})")


def test_simple_urdf_loader():
    """Test the simple URDF loader"""
    loader = SimpleURDFLoader()
    
    # Test with simple_robot.urdf
    urdf_path = "examples/robots/simple_robot.urdf"
    if os.path.exists(urdf_path):
        success = loader.load_urdf(urdf_path)
        if success:
            loader.print_info()
            
            # Test mesh creation
            try:
                import pyvista as pv
                mesh = loader.create_pyvista_mesh(pv)
                if mesh:
                    print(f"\n✅ PyVista mesh created: {mesh.n_points} points, {mesh.n_faces} faces")
                else:
                    print("\n⚠️  No mesh created")
            except ImportError:
                print("\n⚠️  PyVista not available for mesh test")
        else:
            print("❌ Failed to load URDF")
    else:
        print(f"❌ URDF file not found: {urdf_path}")


if __name__ == "__main__":
    test_simple_urdf_loader()