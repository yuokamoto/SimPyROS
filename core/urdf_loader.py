#!/usr/bin/env python3
"""
Unified URDF Loader for SimPyROS

Integrates basic URDF loading with enhanced mesh support and external repository handling.
Supports both primitive geometries and 3D mesh files from local and external sources.

Features:
- yourdfpy-based URDF parsing
- Primitive geometries (box, cylinder, sphere)
- 3D mesh loading with multiple formats (STL, OBJ, DAE)
- ROS package path resolution
- Mesh optimization and simplification
- Material and color extraction
"""

import os
import sys
import urllib.parse
import numpy as np
from typing import Optional, Dict, List, Tuple, Union, Any
from dataclasses import dataclass
from pathlib import Path
import warnings

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from core.simulation_object import Pose
from core.logger import get_logger, log_success, log_warning, log_error, log_debug, log_info
from scipy.spatial.transform import Rotation
import copy
import time

# Try to import yourdfpy
URDF_LIBRARY = None
URDF_MODULE = None

try:
    import yourdfpy as urdf_lib
    URDF_LIBRARY = "yourdfpy"
    URDF_MODULE = urdf_lib
    # Initialize logger after import
    _module_logger = get_logger(__name__)
    log_success(_module_logger, "Using yourdfpy for URDF parsing")
except ImportError:
    _module_logger = get_logger(__name__)
    log_error(_module_logger, "yourdfpy not available. Install with: pip install yourdfpy")

# Try trimesh for mesh handling
try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False

# Global URDF template cache
_URDF_TEMPLATE_CACHE = {}


@dataclass
class URDFLink:
    """URDF link representation with enhanced material and mesh support"""
    name: str
    geometry_type: str  # 'box', 'cylinder', 'sphere', 'mesh'
    geometry_params: Dict  # dimensions, radius, etc.
    color: Tuple[float, float, float, float] = (0.7, 0.7, 0.7, 1.0)  # RGBA
    mesh_path: Optional[str] = None
    original_mesh_path: Optional[str] = None  # Original path from URDF
    pose: Optional[Pose] = None
    mesh_metadata: Dict = None  # Mesh analysis data


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
    """Unified URDF loader with mesh loading, optimization, and template caching support"""
    
    def __init__(self, package_path: Optional[str] = None, enable_mesh_optimization: bool = True, use_template_cache: bool = True):
        self.package_path = package_path
        self.urdf_object = None
        self.links: Dict[str, URDFLink] = {}
        self.joints: Dict[str, URDFJoint] = {}
        self.robot_name = "unknown_robot"
        
        # Mesh processing settings
        self.enable_mesh_optimization = enable_mesh_optimization
        self.max_mesh_faces = 10000  # Face limit for performance
        self.mesh_simplification_ratio = 0.5  # Reduction ratio
        
        # Path resolution context
        self._urdf_dir = None
        
        # Template caching settings
        self.use_template_cache = use_template_cache
        
        # Initialize logger for this instance
        self.logger = get_logger(f"{__name__}.{self.__class__.__name__}")
        
    def is_available(self) -> bool:
        """Check if URDF loading is available"""
        return URDF_LIBRARY is not None
        
    def get_library_name(self) -> str:
        """Get the name of the URDF library being used"""
        return URDF_LIBRARY or "none"
        
    def load_urdf(self, urdf_path: str) -> bool:
        """Load URDF file with enhanced mesh support and template caching"""
        if not self.is_available():
            log_error(self.logger, "No URDF parsing library available")
            return False
            
        if not os.path.exists(urdf_path):
            log_error(self.logger, f"URDF file not found: {urdf_path}")
            return False
        
        # Store URDF directory for relative path resolution
        self._urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        
        # Create cache key from absolute path and modification time
        abs_urdf_path = os.path.abspath(urdf_path)
        cache_key = self._get_cache_key(abs_urdf_path)
        
        # Try to load from template cache first
        if self.use_template_cache and cache_key in _URDF_TEMPLATE_CACHE:
            return self._load_from_template_cache(cache_key)
        
        try:
            if URDF_LIBRARY == "yourdfpy":
                success = self._load_with_yourdfpy(urdf_path)
                # Cache the successfully loaded template
                if success and self.use_template_cache:
                    self._cache_template(cache_key, abs_urdf_path)
                return success
            else:
                log_error(self.logger, f"Unsupported URDF library: {URDF_LIBRARY}")
                return False
                
        except Exception as e:
            log_error(self.logger, f"URDF loading failed: {e}")
            return False
    
    def _load_with_yourdfpy(self, urdf_path: str) -> bool:
        """Load URDF using yourdfpy"""
        try:
            log_info(self.logger, f"Loading URDF with yourdfpy: {urdf_path}")
            self.urdf_object = urdf_lib.URDF.load(urdf_path)
            self.robot_name = getattr(self.urdf_object, 'name', 'unknown_robot')
            
            self._extract_links_joints_yourdfpy()
            
            log_success(self.logger, "Successfully loaded with yourdfpy")
            return True
        except Exception as e:
            log_error(self.logger, f"yourdfpy loading failed: {e}")
            return False
    
    def _get_cache_key(self, urdf_path: str) -> str:
        """Generate cache key from URDF path and modification time"""
        try:
            mtime = os.path.getmtime(urdf_path)
            return f"{urdf_path}:{mtime}"
        except OSError:
            return f"{urdf_path}:0"
    
    def _load_from_template_cache(self, cache_key: str) -> bool:
        """Load URDF data from template cache"""
        try:
            cached_data = _URDF_TEMPLATE_CACHE[cache_key]
            self.urdf_object = cached_data['urdf_object']
            self.robot_name = cached_data['robot_name'] 
            self.links = copy.deepcopy(cached_data['links'])
            self.joints = copy.deepcopy(cached_data['joints'])
            self._urdf_dir = cached_data['urdf_dir']
            log_info(self.logger, f"Loaded from template cache: {cached_data['urdf_path']}")
            return True
        except Exception as e:
            log_warning(self.logger, f"Template cache load failed: {e}")
            # Remove corrupted cache entry
            if cache_key in _URDF_TEMPLATE_CACHE:
                del _URDF_TEMPLATE_CACHE[cache_key]
            return False
    
    def _cache_template(self, cache_key: str, urdf_path: str):
        """Cache the successfully loaded URDF template"""
        try:
            _URDF_TEMPLATE_CACHE[cache_key] = {
                'urdf_object': self.urdf_object,
                'robot_name': self.robot_name,
                'links': copy.deepcopy(self.links),
                'joints': copy.deepcopy(self.joints), 
                'urdf_dir': self._urdf_dir,
                'urdf_path': urdf_path,
                'cache_time': time.time()
            }
            log_info(self.logger, f"Cached URDF template: {urdf_path}")
        except Exception as e:
            log_warning(self.logger, f"Template caching failed: {e}")
    
    @staticmethod
    def get_cache_stats() -> Dict[str, Any]:
        """Get template cache statistics"""
        return {
            'total_templates': len(_URDF_TEMPLATE_CACHE),
            'cached_files': [data['urdf_path'] for data in _URDF_TEMPLATE_CACHE.values()],
            'cache_size_mb': sys.getsizeof(_URDF_TEMPLATE_CACHE) / (1024 * 1024)
        }
    
    @staticmethod
    def clear_template_cache():
        """Clear the entire template cache"""
        global _URDF_TEMPLATE_CACHE
        _URDF_TEMPLATE_CACHE.clear()
        # Use module logger for static method
        log_info(_module_logger, "URDF template cache cleared")
    
    def _extract_links_joints_yourdfpy(self):
        """Extract links and joints from yourdfpy URDF object"""
        # Extract links
        for link in self.urdf_object.robot.links:
            self._process_link_yourdfpy(link)
        
        # Extract joints  
        for joint in self.urdf_object.robot.joints:
            self._process_joint_yourdfpy(joint)
    
    def _process_link_yourdfpy(self, link):
        """Enhanced link processing with mesh support and optimization"""
        urdf_link = URDFLink(
            name=link.name,
            geometry_type="unknown",
            geometry_params={},
            mesh_metadata={}
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
                        # Enhanced mesh processing
                        urdf_link.geometry_type = "mesh"
                        original_path = getattr(geom.mesh, 'filename', None)
                        urdf_link.original_mesh_path = original_path
                        urdf_link.mesh_path = self._resolve_mesh_path(original_path)
                        urdf_link.geometry_params = {
                            'scale': getattr(geom.mesh, 'scale', [1.0, 1.0, 1.0]),
                            'original_path': original_path
                        }
                        
                        # Analyze mesh file if found
                        if urdf_link.mesh_path:
                            urdf_link.mesh_metadata = self._analyze_mesh_file(urdf_link.mesh_path)
                
                # Extract visual origin (pose)
                self._extract_visual_origin(visual, urdf_link)
                
                # Enhanced material/color extraction
                self._extract_material_info(visual, urdf_link)
        
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
        log_debug(self.logger, f"Joint {joint.name}: type={joint_type}, parent={parent_link}, child={child_link}")
        
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
        log_debug(self.logger, f"Joint {joint.name} origin: {pos_str}, {rot_str}")
        
        self.joints[joint.name] = urdf_joint
    
    def _resolve_mesh_path(self, mesh_path: str) -> Optional[str]:
        """
        Resolve mesh file path from URDF reference
        
        Args:
            mesh_path: Original mesh path from URDF (may be package:// URL or relative)
            
        Returns:
            Absolute path to mesh file or None if not found
        """
        if not mesh_path:
            return None
        
        # Handle package:// URLs
        if mesh_path.startswith('package://'):
            # Remove package:// prefix and get relative path
            package_relative = mesh_path[10:]  # Remove 'package://'
            
            # Try to find in current URDF directory tree
            if self._urdf_dir:
                # Look for common ROS structure patterns
                potential_paths = [
                    os.path.join(self._urdf_dir, '..', package_relative),  # Go up one level
                    os.path.join(self._urdf_dir, package_relative),       # Same level
                    os.path.join(self._urdf_dir, '..', '..', package_relative),  # Up two levels
                ]
                
                for path in potential_paths:
                    abs_path = os.path.abspath(path)
                    if os.path.exists(abs_path):
                        log_debug(self.logger, f"Resolved package path: {mesh_path} -> {abs_path}")
                        return abs_path
        
        # Handle absolute paths
        if os.path.isabs(mesh_path) and os.path.exists(mesh_path):
            return mesh_path
        
        # Handle relative paths
        if self._urdf_dir:
            relative_path = os.path.join(self._urdf_dir, mesh_path)
            if os.path.exists(relative_path):
                return os.path.abspath(relative_path)
        
        log_warning(self.logger, f"Could not resolve mesh path: {mesh_path}")
        return None
    
    def _analyze_mesh_file(self, mesh_path: str) -> Dict:
        """
        Analyze mesh file and extract metadata for optimization
        
        Args:
            mesh_path: Path to mesh file
            
        Returns:
            Dictionary with mesh analysis results
        """
        metadata = {
            'file_size_mb': 0.0,
            'face_count': 0,
            'vertex_count': 0,
            'format': 'unknown',
            'needs_simplification': False,
            'loadable': False
        }
        
        try:
            # Get file size
            file_size = os.path.getsize(mesh_path)
            metadata['file_size_mb'] = round(file_size / (1024 * 1024), 2)
            
            # Get file format
            _, ext = os.path.splitext(mesh_path)
            metadata['format'] = ext.lower().lstrip('.')
            
            # Analyze with trimesh if available
            if TRIMESH_AVAILABLE and trimesh:
                try:
                    mesh = trimesh.load(mesh_path)
                    if hasattr(mesh, 'faces') and hasattr(mesh, 'vertices'):
                        metadata['face_count'] = len(mesh.faces)
                        metadata['vertex_count'] = len(mesh.vertices)
                        metadata['loadable'] = True
                        
                        # Check if simplification needed
                        if metadata['face_count'] > self.max_mesh_faces:
                            metadata['needs_simplification'] = True
                            
                        log_debug(self.logger, f"Mesh: {metadata['vertex_count']} vertices, "
                              f"{metadata['face_count']} faces, {metadata['file_size_mb']} MB")
                    else:
                        log_warning(self.logger, "Loaded mesh has no geometry data")
                except Exception as e:
                    log_warning(self.logger, f"Mesh analysis failed: {e}")
            else:
                # Assume loadable if trimesh not available
                metadata['loadable'] = True
                log_debug(self.logger, f"Mesh file found: {os.path.basename(mesh_path)} ({metadata['file_size_mb']} MB)")
            
        except Exception as e:
            log_error(self.logger, f"Could not analyze mesh file {mesh_path}: {e}")
        
        return metadata
    
    def _extract_visual_origin(self, visual, urdf_link: URDFLink):
        """Extract visual origin (pose) information from URDF visual element"""
        from core.simulation_object import Pose
        
        # Default pose (no transformation)
        origin_pos = np.array([0.0, 0.0, 0.0])
        origin_rot = Rotation.identity()
        
        # Extract origin if present
        if hasattr(visual, 'origin') and visual.origin is not None:
            try:
                # Handle different origin representations
                if hasattr(visual.origin, 'xyz') and hasattr(visual.origin, 'rpy'):
                    # xyz and rpy attributes
                    if visual.origin.xyz is not None:
                        origin_pos = np.array(visual.origin.xyz)
                    if visual.origin.rpy is not None:
                        origin_rot = Rotation.from_euler('xyz', visual.origin.rpy)
                elif hasattr(visual.origin, 'reshape'):
                    # Transformation matrix
                    transform_matrix = visual.origin.reshape(4, 4)
                    origin_pos = transform_matrix[:3, 3]
                    rotation_matrix = transform_matrix[:3, :3]
                    origin_rot = Rotation.from_matrix(rotation_matrix)
                else:
                    # Try to extract from matrix directly
                    origin_matrix = np.array(visual.origin)
                    if origin_matrix.shape == (4, 4):
                        origin_pos = origin_matrix[:3, 3]
                        rotation_matrix = origin_matrix[:3, :3]
                        origin_rot = Rotation.from_matrix(rotation_matrix)
            except Exception as e:
                log_warning(self.logger, f"Could not process visual origin for {urdf_link.name}: {e}")
        
        # Store pose in URDFLink
        urdf_link.pose = Pose.from_position_rotation(origin_pos, origin_rot)
        
        # Debug output
        if not np.allclose(origin_pos, [0, 0, 0]) or not np.allclose(origin_rot.as_euler('xyz'), [0, 0, 0]):
            pos_str = f"pos={origin_pos}"
            rot_euler = origin_rot.as_euler('xyz', degrees=True)
            rot_str = f"rot={rot_euler}°"
            log_debug(self.logger, f"Visual origin for {urdf_link.name}: {pos_str}, {rot_str}")
    
    def _extract_material_info(self, visual, urdf_link: URDFLink):
        """Enhanced material and color extraction with opacity fix"""
        # Try standard material color first
        if hasattr(visual, 'material') and visual.material:
            if hasattr(visual.material, 'color') and visual.material.color:
                if hasattr(visual.material.color, 'rgba'):
                    rgba = visual.material.color.rgba
                    # Fix transparency issue: ensure opacity = 1.0 unless explicitly transparent
                    if len(rgba) >= 4:
                        # Keep RGB, set alpha to 1.0 for opaque rendering
                        fixed_rgba = (rgba[0], rgba[1], rgba[2], 1.0)
                        urdf_link.color = fixed_rgba
                        log_debug(self.logger, f"Material color for {urdf_link.name}: rgba={rgba} -> fixed={fixed_rgba}")
                    else:
                        urdf_link.color = tuple(rgba) + (1.0,)  # Add opaque alpha
                    return
        
        # Assign default colors based on geometry type
        default_colors = {
            'box': (0.8, 0.4, 0.2, 1.0),      # Orange
            'cylinder': (0.2, 0.8, 0.4, 1.0), # Green  
            'sphere': (0.4, 0.2, 0.8, 1.0),   # Purple
            'mesh': (0.6, 0.6, 0.6, 1.0)      # Gray
        }
        
        urdf_link.color = default_colors.get(urdf_link.geometry_type, (0.7, 0.7, 0.7, 1.0))
    
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
                        
                        # Apply scaling if specified
                        scale = link.geometry_params.get('scale', [1.0, 1.0, 1.0])
                        if scale != [1.0, 1.0, 1.0]:
                            trimesh_mesh.apply_scale(scale)
                        
                        # Optimize mesh if needed
                        if (self.enable_mesh_optimization and 
                            link.mesh_metadata and 
                            link.mesh_metadata.get('needs_simplification', False)):
                            # Simplify mesh for better performance
                            original_faces = len(trimesh_mesh.faces)
                            simplified_mesh = trimesh_mesh.simplify_quadrics(
                                face_count=int(original_faces * self.mesh_simplification_ratio)
                            )
                            if simplified_mesh is not None:
                                trimesh_mesh = simplified_mesh
                                log_info(self.logger, f"Simplified mesh: {original_faces} -> {len(trimesh_mesh.faces)} faces")
                        
                        # Convert to PyVista
                        if hasattr(trimesh_mesh, 'vertices') and hasattr(trimesh_mesh, 'faces'):
                            pv_mesh = pv_module.wrap(trimesh_mesh.vertices, trimesh_mesh.faces)
                        else:
                            raise Exception("Mesh has no geometry data")
                    else:
                        # Fallback to box
                        pv_mesh = pv_module.Cube()
                        log_warning(self.logger, f"Mesh file not loaded (trimesh unavailable): {link.mesh_path}")
                except Exception as e:
                    # Fallback to box
                    pv_mesh = pv_module.Cube()
                    log_warning(self.logger, f"Mesh loading failed: {e}")
            
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
                log_debug(self.logger, f"Created {link.geometry_type} mesh for {link_name} with color {link.color[:3]}")
        
        log_success(self.logger, f"Created {len(meshes)} individual meshes")
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
        
        log_debug(self.logger, f"Root link: {root_link} at origin")
        
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
                log_debug(self.logger, f"Link {joint.child_link}: {pos_str}, {rot_str}, joint={joint.name}")
                
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
        log_info(self.logger, "\n🤖 Robot Information")
        log_info(self.logger, f"Library: {URDF_LIBRARY}")
        log_info(self.logger, f"Name: {self.robot_name}")
        log_info(self.logger, f"Links: {len(self.links)}")
        log_info(self.logger, f"Joints: {len(self.joints)}")
        
        log_info(self.logger, "\n📦 Links:")
        for name, link in self.links.items():
            log_info(self.logger, f"  - {name}: {link.geometry_type}")
            
        log_info(self.logger, "\n🔗 Joints:")
        for name, joint in self.joints.items():
            log_info(self.logger, f"  - {name}: {joint.joint_type} ({joint.parent_link} → {joint.child_link})")
    
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
    module_logger = get_logger(f"{__name__}.info")
    log_info(module_logger, "\n📄 URDF Loader Information")
    log_info(module_logger, f"Available: {is_urdf_loader_available()}")
    if is_urdf_loader_available():
        log_info(module_logger, f"Library: {URDF_LIBRARY}")
        log_info(module_logger, f"Trimesh: {'Available' if TRIMESH_AVAILABLE else 'Not available'}")
    else:
        log_info(module_logger, "Install with: pip install yourdfpy")
        log_info(module_logger, "Optional: pip install trimesh  # for mesh file support")