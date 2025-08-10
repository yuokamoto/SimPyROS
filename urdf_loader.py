#!/usr/bin/env python3
"""
Advanced URDF Loader for SimPyROS
Using yourdfpy for modern URDF parsing
"""

import os
import numpy as np
from typing import Optional, Dict, List, Tuple, Union
from dataclasses import dataclass
import warnings

from simulation_object import Pose
from scipy.spatial.transform import Rotation

# Try to import yourdfpy
URDF_LIBRARY = None
URDF_MODULE = None

try:
    import yourdfpy as urdf_lib
    URDF_LIBRARY = "yourdfpy"
    URDF_MODULE = urdf_lib
    print("✅ Using yourdfpy for URDF parsing")
except ImportError:
    print("❌ yourdfpy not available. Install with: pip install yourdfpy")

# Try trimesh for mesh handling
try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False


@dataclass
class URDFLink:
    """URDF link representation with material support"""
    name: str
    geometry_type: str  # 'box', 'cylinder', 'sphere', 'mesh'
    geometry_params: Dict  # dimensions, radius, etc.
    color: Tuple[float, float, float, float] = (0.7, 0.7, 0.7, 1.0)  # RGBA
    mesh_path: Optional[str] = None
    pose: Optional[Pose] = None


@dataclass  
class URDFJoint:
    """URDF joint representation"""
    name: str
    joint_type: str  # 'fixed', 'revolute', 'prismatic', etc.
    parent_link: str
    child_link: str
    origin_pos: np.ndarray
    origin_rot: Rotation
    axis: Optional[np.ndarray] = None
    limits: Optional[Dict] = None


class URDFLoader:
    """URDF loader supporting multiple libraries and features"""
    
    def __init__(self, package_path: Optional[str] = None):
        self.package_path = package_path
        self.urdf_object = None
        self.links: Dict[str, URDFLink] = {}
        self.joints: Dict[str, URDFJoint] = {}
        self.robot_name = "unknown_robot"
        
    def is_available(self) -> bool:
        """Check if URDF loading is available"""
        return URDF_LIBRARY is not None
        
    def get_library_name(self) -> str:
        """Get the name of the URDF library being used"""
        return URDF_LIBRARY or "none"
        
    def load_urdf(self, urdf_path: str) -> bool:
        """Load URDF file with automatic library detection"""
        if not self.is_available():
            print("❌ No URDF parsing library available")
            return False
            
        if not os.path.exists(urdf_path):
            print(f"❌ URDF file not found: {urdf_path}")
            return False
        
        try:
            if URDF_LIBRARY == "yourdfpy":
                return self._load_with_yourdfpy(urdf_path)
            else:
                print(f"❌ Unsupported URDF library: {URDF_LIBRARY}")
                return False
                
        except Exception as e:
            print(f"❌ URDF loading failed: {e}")
            return False
    
    def _load_with_yourdfpy(self, urdf_path: str) -> bool:
        """Load URDF using yourdfpy"""
        try:
            print(f"🔄 Loading URDF with yourdfpy: {urdf_path}")
            self.urdf_object = urdf_lib.URDF.load(urdf_path)
            self.robot_name = getattr(self.urdf_object, 'name', 'unknown_robot')
            
            self._extract_links_joints_yourdfpy()
            
            print(f"✅ Successfully loaded with yourdfpy")
            return True
        except Exception as e:
            print(f"❌ yourdfpy loading failed: {e}")
            return False
    
    def _extract_links_joints_yourdfpy(self):
        """Extract links and joints from yourdfpy URDF object"""
        # Extract links
        for link in self.urdf_object.robot.links:
            self._process_link_yourdfpy(link)
        
        # Extract joints  
        for joint in self.urdf_object.robot.joints:
            self._process_joint_yourdfpy(joint)
    
    def _process_link_yourdfpy(self, link):
        """Process a link from yourdfpy"""
        urdf_link = URDFLink(
            name=link.name,
            geometry_type="unknown",
            geometry_params={}
        )
        
        # Extract visual information
        if hasattr(link, 'visuals') and link.visuals:
            for visual in link.visuals:
                # Extract geometry
                if hasattr(visual, 'geometry') and visual.geometry:
                    geom = visual.geometry
                    
                    if hasattr(geom, 'box') and geom.box:
                        urdf_link.geometry_type = "box"
                        urdf_link.geometry_params = {
                            'size': getattr(geom.box, 'size', [1.0, 1.0, 1.0])
                        }
                    elif hasattr(geom, 'cylinder') and geom.cylinder:
                        urdf_link.geometry_type = "cylinder"
                        urdf_link.geometry_params = {
                            'radius': getattr(geom.cylinder, 'radius', 0.1),
                            'length': getattr(geom.cylinder, 'length', 1.0)
                        }
                    elif hasattr(geom, 'sphere') and geom.sphere:
                        urdf_link.geometry_type = "sphere"
                        urdf_link.geometry_params = {
                            'radius': getattr(geom.sphere, 'radius', 0.1)
                        }
                    elif hasattr(geom, 'mesh') and geom.mesh:
                        urdf_link.geometry_type = "mesh"
                        urdf_link.mesh_path = getattr(geom.mesh, 'filename', None)
                        urdf_link.geometry_params = {
                            'scale': getattr(geom.mesh, 'scale', [1.0, 1.0, 1.0])
                        }
                
                # Extract material/color
                if hasattr(visual, 'material') and visual.material:
                    if hasattr(visual.material, 'color') and visual.material.color:
                        if hasattr(visual.material.color, 'rgba'):
                            rgba = visual.material.color.rgba
                            urdf_link.color = tuple(rgba)
                            print(f"    Material color for {link.name}: rgba={rgba}")
        
        self.links[link.name] = urdf_link
    
    def _process_joint_yourdfpy(self, joint):
        """Process a joint from yourdfpy"""
        # Extract origin (pose transformation)
        origin_pos = np.array([0.0, 0.0, 0.0])
        origin_rot = Rotation.identity()
        
        if hasattr(joint, 'origin') and joint.origin is not None:
            try:
                # yourdfpy provides origin as 4x4 transformation matrix
                if hasattr(joint.origin, 'reshape') and joint.origin.size == 16:
                    # It's a transformation matrix
                    transform_matrix = joint.origin.reshape(4, 4)
                    origin_pos = transform_matrix[:3, 3]
                    rotation_matrix = transform_matrix[:3, :3]
                    origin_rot = Rotation.from_matrix(rotation_matrix)
                elif hasattr(joint.origin, 'xyz') and hasattr(joint.origin, 'rpy'):
                    # It's xyz and rpy attributes
                    origin_pos = np.array(joint.origin.xyz) if joint.origin.xyz is not None else origin_pos
                    if joint.origin.rpy is not None:
                        origin_rot = Rotation.from_euler('xyz', joint.origin.rpy)
                else:
                    # Try to extract from matrix directly
                    origin_matrix = np.array(joint.origin)
                    if origin_matrix.shape == (4, 4):
                        origin_pos = origin_matrix[:3, 3]
                        rotation_matrix = origin_matrix[:3, :3]
                        origin_rot = Rotation.from_matrix(rotation_matrix)
            except Exception as e:
                warnings.warn(f"Failed to extract joint origin for {joint.name}: {e}")
        
        # Extract joint type with debug info
        joint_type = getattr(joint, 'type', getattr(joint, 'joint_type', 'fixed'))
        parent_link = getattr(joint, 'parent', 'unknown') 
        child_link = getattr(joint, 'child', 'unknown')
        
        # Debug output for joint type  
        print(f"  Joint {joint.name}: type={joint_type}, parent={parent_link}, child={child_link}")
        
        # Extract axis and limits
        axis = getattr(joint, 'axis', None)
        if axis is not None:
            axis = np.array(axis)
        
        limits = None
        if hasattr(joint, 'limit') and joint.limit is not None:
            limit_obj = joint.limit
            limits = {
                'lower': getattr(limit_obj, 'lower', -np.pi),
                'upper': getattr(limit_obj, 'upper', np.pi),
                'velocity': getattr(limit_obj, 'velocity', 1.0),
                'effort': getattr(limit_obj, 'effort', 100.0)
            }
        
        # Create URDF joint
        urdf_joint = URDFJoint(
            name=joint.name,
            joint_type=joint_type,
            parent_link=parent_link,
            child_link=child_link, 
            origin_pos=origin_pos,
            origin_rot=origin_rot,
            axis=axis,
            limits=limits
        )
        
        # Debug output
        pos_str = f"pos={origin_pos}"
        rot_euler = origin_rot.as_euler('xyz', degrees=True)
        rot_str = f"rot={rot_euler}°"
        print(f"  Joint {joint.name} origin: {pos_str}, {rot_str}")
        
        self.joints[joint.name] = urdf_joint
    
    def get_mesh_files(self) -> List[str]:
        """Get list of mesh files referenced in URDF"""
        mesh_files = []
        
        if URDF_LIBRARY in ["yourdfpy"] and hasattr(self.urdf_object, 'meshes'):
            for mesh_path in self.urdf_object.meshes:
                if os.path.exists(mesh_path):
                    mesh_files.append(mesh_path)
        
        return mesh_files
    
    def create_pyvista_meshes(self, pv_module):
        """Create separate PyVista meshes for each link with proper colors"""
        meshes = []
        link_poses = self._compute_link_poses()
        
        for link_name, link in self.links.items():
            pv_mesh = None
            
            # Create PyVista mesh based on geometry type
            if link.geometry_type == "box":
                size = link.geometry_params.get('size', [1.0, 1.0, 1.0])
                pv_mesh = pv_module.Cube(x_length=size[0], y_length=size[1], z_length=size[2])
                
            elif link.geometry_type == "cylinder":
                radius = link.geometry_params.get('radius', 0.1)
                height = link.geometry_params.get('length', 1.0)
                pv_mesh = pv_module.Cylinder(radius=radius, height=height, direction=(0, 0, 1))
                
            elif link.geometry_type == "sphere":
                radius = link.geometry_params.get('radius', 0.1)
                pv_mesh = pv_module.Sphere(radius=radius)
                
            elif link.geometry_type == "mesh" and link.mesh_path:
                try:
                    if TRIMESH_AVAILABLE:
                        # Load with trimesh then convert to PyVista
                        trimesh_mesh = trimesh.load(link.mesh_path)
                        pv_mesh = pv_module.wrap(trimesh_mesh.vertices, trimesh_mesh.faces)
                    else:
                        # Fallback to box
                        pv_mesh = pv_module.Cube()
                        print(f"⚠️  Mesh file not loaded (trimesh unavailable): {link.mesh_path}")
                except Exception as e:
                    # Fallback to box
                    pv_mesh = pv_module.Cube()
                    print(f"⚠️  Mesh loading failed: {e}")
            
            # Apply transformation if mesh was created successfully
            if pv_mesh is not None and link_name in link_poses:
                pose = link_poses[link_name]
                transform_matrix = self._pose_to_matrix(pose)
                pv_mesh.transform(transform_matrix)
            
            # Store mesh info
            meshes.append({
                'mesh': pv_mesh,
                'name': link_name,
                'color': link.color[:3],  # RGB only for PyVista
                'geometry_type': link.geometry_type,
                'pose': link_poses.get(link_name, None)
            })
            
            # Debug output
            if pv_mesh is not None:
                print(f"  Created {link.geometry_type} mesh for {link_name} with color {link.color[:3]}")
        
        print(f"✅ Created {len(meshes)} individual meshes")
        return meshes
    
    def _compute_link_poses(self):
        """Compute absolute poses for all links based on joint transformations"""
        link_poses = {}
        
        # Start with root link at origin
        root_link = self._find_root_link()
        link_poses[root_link] = Pose.from_position_rotation(
            np.array([0.0, 0.0, 0.0]), 
            Rotation.identity()
        )
        
        print(f"  Root link: {root_link} at origin")
        
        # Recursively compute poses for all connected links
        self._compute_child_poses(root_link, link_poses[root_link], link_poses)
        
        return link_poses
    
    def _find_root_link(self) -> str:
        """Find the root link (link that is not a child of any joint)"""
        child_links = set()
        for joint in self.joints.values():
            child_links.add(joint.child_link)
        
        for link_name in self.links.keys():
            if link_name not in child_links:
                return link_name
                
        # Fallback: return first link
        return next(iter(self.links.keys())) if self.links else "unknown"
    
    def _compute_child_poses(self, parent_link: str, parent_pose: Pose, link_poses: Dict[str, Pose]):
        """Recursively compute poses for child links"""
        for joint in self.joints.values():
            if joint.parent_link == parent_link and joint.child_link not in link_poses:
                # Compute child pose by manual transformation
                # Transform joint origin position by parent pose
                transformed_pos = parent_pose.position + parent_pose.rotation.apply(joint.origin_pos)
                # Combine rotations
                combined_rot = parent_pose.rotation * joint.origin_rot
                child_pose = Pose.from_position_rotation(transformed_pos, combined_rot)
                link_poses[joint.child_link] = child_pose
                
                # Debug output
                pos_str = f"pos={child_pose.position}"
                rot_euler = child_pose.rotation.as_euler('xyz', degrees=True)
                rot_str = f"rot={rot_euler}°"
                print(f"  Link {joint.child_link}: {pos_str}, {rot_str}, joint={joint.name}")
                
                # Recursively compute poses for children of this link
                self._compute_child_poses(joint.child_link, child_pose, link_poses)
    
    def _pose_to_matrix(self, pose: Pose) -> np.ndarray:
        """Convert Pose to 4x4 transformation matrix"""
        matrix = np.eye(4)
        matrix[:3, :3] = pose.rotation.as_matrix()
        matrix[:3, 3] = pose.position
        return matrix
        
    def print_info(self):
        """Print comprehensive robot information"""
        print(f"\n🤖 Robot Information")
        print(f"Library: {URDF_LIBRARY}")
        print(f"Name: {self.robot_name}")
        print(f"Links: {len(self.links)}")
        print(f"Joints: {len(self.joints)}")
        
        print(f"\n📦 Links:")
        for name, link in self.links.items():
            print(f"  - {name}: {link.geometry_type}")
            
        print(f"\n🔗 Joints:")
        for name, joint in self.joints.items():
            print(f"  - {name}: {joint.joint_type} ({joint.parent_link} → {joint.child_link})")
    
    def get_combined_mesh(self, pv_module):
        """Create a single combined PyVista mesh (legacy support)"""
        individual_meshes = self.create_pyvista_meshes(pv_module)
        
        if not individual_meshes:
            return None
            
        # For now, return the first mesh as combined mesh
        # In future, could actually combine all meshes into one
        combined_mesh = individual_meshes[0]['mesh']
        
        # Attach individual meshes info for animation system
        combined_mesh.individual_meshes = individual_meshes
        
        return combined_mesh


def is_urdf_loader_available() -> bool:
    """Check if URDF loading capabilities are available"""
    return URDF_LIBRARY is not None


def print_urdf_loader_info():
    """Print information about available URDF loading capabilities"""
    print(f"\n📄 URDF Loader Information")
    print(f"Available: {is_urdf_loader_available()}")
    if is_urdf_loader_available():
        print(f"Library: {URDF_LIBRARY}")
        print(f"Trimesh: {'Available' if TRIMESH_AVAILABLE else 'Not available'}")
    else:
        print("Install with: pip install yourdfpy")
        print("Optional: pip install trimesh  # for mesh file support")