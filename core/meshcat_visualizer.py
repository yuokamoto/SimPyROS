#!/usr/bin/env python3
"""
MeshCat Visualizer for SimPyROS - Lightweight Web-based Alternative to PyVista

MeshCat Features:
1. Web-based visualization - runs in browser
2. Zero installation requirements 
3. High performance WebGL rendering
4. Real-time remote control
5. URDF and mesh support
6. Lightweight network protocol

Benefits over PyVista:
- No VTK dependency or installation issues
- Better performance for real-time applications
- Cross-platform compatibility
- No threading limitations
- Web-based UI with better interactivity
"""

import os
import numpy as np
import math
import time
import threading
from typing import Optional, Tuple, List, Dict, Union, Any
import warnings

# Add parent directory to path
import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

# Try to import MeshCat
try:
    import meshcat
    import meshcat.geometry as g
    import meshcat.transformations as tf
    MESHCAT_AVAILABLE = True
    print("✅ MeshCat available for lightweight visualization")
except ImportError:
    MESHCAT_AVAILABLE = False
    print("❌ MeshCat not available. Install with: pip install meshcat")
    # Create dummy classes for type hints
    class DummyVisualizer:
        pass
    meshcat = type('meshcat', (), {'Visualizer': DummyVisualizer})()

# Import URDF loaders
try:
    from core.urdf_loader import URDFLoader
    URDF_SUPPORT = True
except ImportError:
    URDF_SUPPORT = False

# Import time management
try:
    from core.time_manager import TimeManager, get_global_time_manager
    TIME_MANAGER_SUPPORT = True
except ImportError:
    TIME_MANAGER_SUPPORT = False

from core.simulation_object import Pose


class MeshCatVisualizer:
    """
    Lightweight web-based 3D visualizer using MeshCat
    
    Advantages over PyVista:
    - No VTK dependency
    - Web-based interface
    - Better real-time performance
    - No threading limitations
    - Zero installation visualization
    """
    
    def __init__(self, port: int = 7000, open_browser: bool = True):
        """
        Initialize MeshCat visualizer
        
        Args:
            port: Port for MeshCat server
            open_browser: Whether to open browser automatically
        """
        self.available = MESHCAT_AVAILABLE
        self.port = port
        self.open_browser = open_browser
        
        # Performance monitoring
        self.performance_stats = {
            'update_count': 0,
            'last_update_time': 0.0,
            'avg_update_time': 0.0,
            'start_time': time.time(),
            'total_objects': 0
        }
        
        # State management
        self.robots = {}
        self.objects = {}
        self.link_paths = {}  # robot_name -> {link_name -> meshcat_path}
        
        if self.available:
            try:
                # Create MeshCat visualizer
                self.vis = meshcat.Visualizer(zmq_url=f"tcp://127.0.0.1:{port}")
                print(f"🌐 MeshCat server started on port {port}")
                print(f"📱 View at: http://127.0.0.1:{port}/static/")
                
                # Clear any existing visualization
                self.vis.delete()
                
                # Set up scene
                self._setup_scene()
                
                if open_browser:
                    self._open_browser()
                    
            except Exception as e:
                print(f"⚠️ Failed to initialize MeshCat: {e}")
                self.available = False
        else:
            self.vis = None
    
    def _setup_scene(self):
        """Setup basic scene elements"""
        if not self.available:
            return
        
        try:
            # Add coordinate frame at origin
            self._add_coordinate_frame("world", scale=0.5)
            
            # Add grid (optional)
            # self._add_grid()
            
            # Set camera position
            self.vis["/Cameras/default"].set_transform(
                tf.translation_matrix([3, 3, 2]) @ 
                tf.rotation_matrix(-np.pi/4, [0, 0, 1]) @
                tf.rotation_matrix(-np.pi/6, [1, 0, 0])
            )
            
            print("🎬 MeshCat scene initialized")
            
        except Exception as e:
            print(f"⚠️ Scene setup error: {e}")
    
    def _add_coordinate_frame(self, name: str, scale: float = 1.0):
        """Add coordinate frame visualization"""
        if not self.available:
            return
        
        try:
            # X axis (red)
            self.vis[f"{name}/x_axis"].set_object(
                g.Cylinder(height=scale, radius=scale*0.02),
                g.MeshLambertMaterial(color=0xff0000)
            )
            self.vis[f"{name}/x_axis"].set_transform(
                tf.translation_matrix([scale/2, 0, 0]) @
                tf.rotation_matrix(np.pi/2, [0, 1, 0])
            )
            
            # Y axis (green)
            self.vis[f"{name}/y_axis"].set_object(
                g.Cylinder(height=scale, radius=scale*0.02),
                g.MeshLambertMaterial(color=0x00ff00)
            )
            self.vis[f"{name}/y_axis"].set_transform(
                tf.translation_matrix([0, scale/2, 0]) @
                tf.rotation_matrix(-np.pi/2, [1, 0, 0])
            )
            
            # Z axis (blue)
            self.vis[f"{name}/z_axis"].set_object(
                g.Cylinder(height=scale, radius=scale*0.02),
                g.MeshLambertMaterial(color=0x0000ff)
            )
            self.vis[f"{name}/z_axis"].set_transform(
                tf.translation_matrix([0, 0, scale/2])
            )
            
        except Exception as e:
            print(f"⚠️ Coordinate frame error: {e}")
    
    def _open_browser(self):
        """Open browser to MeshCat URL"""
        try:
            import webbrowser
            url = f"http://127.0.0.1:{self.port}/static/"
            webbrowser.open(url)
            print(f"🌐 Opened browser to {url}")
        except Exception as e:
            print(f"⚠️ Could not open browser: {e}")
    
    def add_robot(self, robot_name: str, robot_data: Any) -> bool:
        """
        Add robot to visualization
        
        Args:
            robot_name: Unique name for the robot
            robot_data: Robot instance with URDF data
            
        Returns:
            True if successful
        """
        if not self.available:
            return False
        
        try:
            print(f"🤖 Adding robot '{robot_name}' to MeshCat visualization...")
            
            # Store robot reference
            self.robots[robot_name] = robot_data
            self.link_paths[robot_name] = {}
            
            # Create robot links from URDF data
            if hasattr(robot_data, 'urdf_loader') and robot_data.urdf_loader:
                self._create_robot_links(robot_name, robot_data)
            else:
                print(f"⚠️ Robot {robot_name} has no URDF data")
                return False
            
            # Update performance stats
            self.performance_stats['total_objects'] += 1
            
            print(f"✅ Robot '{robot_name}' added to MeshCat")
            return True
            
        except Exception as e:
            print(f"❌ Failed to add robot '{robot_name}': {e}")
            return False
    
    def _create_robot_links(self, robot_name: str, robot_data: Any):
        """Create visual representation of robot links"""
        if not self.available:
            return
        
        try:
            urdf_loader = robot_data.urdf_loader
            
            # Iterate through all links
            for link_name, link_data in urdf_loader.links.items():
                
                # Skip links without visual elements
                if not hasattr(link_data, 'visuals') or not link_data.visuals:
                    continue
                
                # Create MeshCat path for this link
                link_path = f"robots/{robot_name}/{link_name}"
                self.link_paths[robot_name][link_name] = link_path
                
                # Process visual elements
                for i, visual in enumerate(link_data.visuals):
                    visual_path = f"{link_path}/visual_{i}" if len(link_data.visuals) > 1 else link_path
                    
                    # Create geometry based on visual type
                    geometry, material = self._create_geometry_from_visual(visual)
                    
                    if geometry and material:
                        # Add to MeshCat
                        self.vis[visual_path].set_object(geometry, material)
                        
                        # Apply visual origin transform
                        if hasattr(visual, 'origin') and visual.origin is not None:
                            transform = self._visual_origin_to_transform(visual.origin)
                            self.vis[visual_path].set_transform(transform)
            
            print(f"   📦 Created {len(self.link_paths[robot_name])} links for {robot_name}")
            
        except Exception as e:
            print(f"⚠️ Error creating robot links: {e}")
    
    def _create_geometry_from_visual(self, visual) -> Tuple[Optional[Any], Optional[Any]]:
        """
        Create MeshCat geometry and material from URDF visual element
        
        Returns:
            Tuple of (geometry, material)
        """
        try:
            geometry = None
            material = None
            
            # Get geometry from visual
            if hasattr(visual, 'geometry') and visual.geometry:
                geom = visual.geometry
                
                # Handle different geometry types
                if hasattr(geom, 'box') and geom.box:
                    # Box geometry
                    size = geom.box.size
                    geometry = g.Box([size[0], size[1], size[2]])
                    
                elif hasattr(geom, 'cylinder') and geom.cylinder:
                    # Cylinder geometry
                    radius = geom.cylinder.radius
                    length = geom.cylinder.length
                    geometry = g.Cylinder(height=length, radius=radius)
                    
                elif hasattr(geom, 'sphere') and geom.sphere:
                    # Sphere geometry
                    radius = geom.sphere.radius
                    geometry = g.Sphere(radius)
                    
                elif hasattr(geom, 'mesh') and geom.mesh:
                    # Mesh geometry - try to load mesh file
                    mesh_filename = geom.mesh.filename
                    if mesh_filename:
                        # For now, use a placeholder box for mesh
                        # TODO: Implement actual mesh loading
                        geometry = g.Box([0.1, 0.1, 0.1])
                        print(f"   📄 Mesh placeholder for {mesh_filename}")
                
                # Create material
                if hasattr(visual, 'material') and visual.material:
                    material = self._create_material_from_urdf(visual.material)
                else:
                    # Default material
                    material = g.MeshLambertMaterial(color=0x888888)
            
            return geometry, material
            
        except Exception as e:
            print(f"⚠️ Geometry creation error: {e}")
            return None, None
    
    def _create_material_from_urdf(self, material_data) -> Any:
        """Create MeshCat material from URDF material data"""
        try:
            color = 0x888888  # Default gray
            
            # Try to get color from material
            if hasattr(material_data, 'color') and material_data.color is not None:
                rgba = material_data.color
                # Convert RGBA to RGB hex
                r = int(rgba[0] * 255) if rgba[0] <= 1.0 else int(rgba[0])
                g = int(rgba[1] * 255) if rgba[1] <= 1.0 else int(rgba[1])
                b = int(rgba[2] * 255) if rgba[2] <= 1.0 else int(rgba[2])
                color = (r << 16) | (g << 8) | b
            
            return g.MeshLambertMaterial(color=color)
            
        except Exception as e:
            print(f"⚠️ Material creation error: {e}")
            return g.MeshLambertMaterial(color=0x888888)
    
    def _visual_origin_to_transform(self, origin) -> np.ndarray:
        """Convert URDF visual origin to transformation matrix"""
        try:
            # Get translation
            xyz = origin.xyz if hasattr(origin, 'xyz') else [0, 0, 0]
            translation = tf.translation_matrix(xyz)
            
            # Get rotation (in RPY format)
            rpy = origin.rpy if hasattr(origin, 'rpy') else [0, 0, 0]
            rotation = tf.euler_matrix(rpy[0], rpy[1], rpy[2], 'sxyz')
            
            return translation @ rotation
            
        except Exception as e:
            print(f"⚠️ Transform creation error: {e}")
            return np.eye(4)
    
    def update_robot_pose(self, robot_name: str, pose: Pose) -> bool:
        """
        Update robot base pose
        
        Args:
            robot_name: Name of robot to update
            pose: New pose for robot base
            
        Returns:
            True if successful
        """
        if not self.available or robot_name not in self.robots:
            return False
        
        try:
            update_start = time.time()
            
            # Create transformation matrix from pose
            transform = pose.to_transformation_matrix()
            
            # Update robot base transform
            robot_path = f"robots/{robot_name}"
            self.vis[robot_path].set_transform(transform)
            
            # Update performance stats
            update_time = time.time() - update_start
            self.performance_stats['update_count'] += 1
            self.performance_stats['last_update_time'] = update_time
            
            # Update average
            count = self.performance_stats['update_count']
            if count > 1:
                old_avg = self.performance_stats['avg_update_time']
                self.performance_stats['avg_update_time'] = old_avg + (update_time - old_avg) / count
            else:
                self.performance_stats['avg_update_time'] = update_time
            
            return True
            
        except Exception as e:
            print(f"⚠️ Error updating robot pose: {e}")
            return False
    
    def update_robot_joints(self, robot_name: str, joint_positions: Dict[str, float]) -> bool:
        """
        Update robot joint positions
        
        Args:
            robot_name: Name of robot to update
            joint_positions: Dictionary of joint_name -> position
            
        Returns:
            True if successful
        """
        if not self.available or robot_name not in self.robots:
            return False
        
        try:
            robot = self.robots[robot_name]
            
            # Update joint positions in robot model
            if hasattr(robot, 'set_joint_positions'):
                robot.set_joint_positions(joint_positions)
            
            # Get updated link poses from forward kinematics
            if hasattr(robot, 'get_link_poses'):
                link_poses = robot.get_link_poses()
                
                # Update MeshCat transforms for each link
                for link_name, pose in link_poses.items():
                    if pose is not None and link_name in self.link_paths[robot_name]:
                        link_path = self.link_paths[robot_name][link_name]
                        transform = pose.to_transformation_matrix()
                        self.vis[link_path].set_transform(transform)
            
            return True
            
        except Exception as e:
            print(f"⚠️ Error updating robot joints: {e}")
            return False
    
    def add_simple_object(self, name: str, shape: str, pose: Pose, 
                         size: Union[float, List[float]] = 1.0, 
                         color: int = 0x888888) -> bool:
        """
        Add simple geometric object
        
        Args:
            name: Object name
            shape: 'box', 'sphere', 'cylinder'
            pose: Object pose
            size: Size parameter(s)
            color: RGB color as hex
            
        Returns:
            True if successful
        """
        if not self.available:
            return False
        
        try:
            # Create geometry based on shape
            if shape == 'box':
                if isinstance(size, (list, tuple)):
                    geometry = g.Box(size)
                else:
                    geometry = g.Box([size, size, size])
            elif shape == 'sphere':
                radius = size if isinstance(size, (int, float)) else size[0]
                geometry = g.Sphere(radius)
            elif shape == 'cylinder':
                if isinstance(size, (list, tuple)) and len(size) >= 2:
                    geometry = g.Cylinder(height=size[0], radius=size[1])
                else:
                    geometry = g.Cylinder(height=size, radius=size*0.5)
            else:
                print(f"⚠️ Unknown shape: {shape}")
                return False
            
            # Create material
            material = g.MeshLambertMaterial(color=color)
            
            # Add to scene
            object_path = f"objects/{name}"
            self.vis[object_path].set_object(geometry, material)
            
            # Set pose
            transform = pose.to_transformation_matrix()
            self.vis[object_path].set_transform(transform)
            
            # Store object
            self.objects[name] = {
                'shape': shape,
                'pose': pose,
                'size': size,
                'color': color
            }
            
            self.performance_stats['total_objects'] += 1
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to add object '{name}': {e}")
            return False
    
    def update_object_pose(self, name: str, pose: Pose) -> bool:
        """Update object pose"""
        if not self.available or name not in self.objects:
            return False
        
        try:
            object_path = f"objects/{name}"
            transform = pose.to_transformation_matrix()
            self.vis[object_path].set_transform(transform)
            
            # Update stored pose
            self.objects[name]['pose'] = pose
            
            return True
            
        except Exception as e:
            print(f"⚠️ Error updating object pose: {e}")
            return False
    
    def remove_object(self, name: str) -> bool:
        """Remove object from visualization"""
        if not self.available:
            return False
        
        try:
            object_path = f"objects/{name}"
            self.vis[object_path].delete()
            
            if name in self.objects:
                del self.objects[name]
                self.performance_stats['total_objects'] -= 1
            
            return True
            
        except Exception as e:
            print(f"⚠️ Error removing object: {e}")
            return False
    
    def clear_all(self):
        """Clear all visualization"""
        if not self.available:
            return
        
        try:
            self.vis.delete()
            self.robots.clear()
            self.objects.clear()
            self.link_paths.clear()
            self.performance_stats['total_objects'] = 0
            
            # Re-setup scene
            self._setup_scene()
            
        except Exception as e:
            print(f"⚠️ Error clearing visualization: {e}")
    
    def get_performance_stats(self) -> Dict:
        """Get performance statistics"""
        total_time = time.time() - self.performance_stats['start_time']
        
        stats = self.performance_stats.copy()
        stats['total_time'] = total_time
        stats['avg_update_rate'] = self.performance_stats['update_count'] / total_time if total_time > 0 else 0
        
        return stats
    
    def print_performance_summary(self):
        """Print performance summary"""
        stats = self.get_performance_stats()
        
        print(f"\n📊 MeshCat Performance Summary")
        print("=" * 40)
        print(f"Total time: {stats['total_time']:.1f}s")
        print(f"Update count: {stats['update_count']}")
        print(f"Total objects: {stats['total_objects']}")
        print(f"Avg update rate: {stats['avg_update_rate']:.1f} Hz")
        print(f"Avg update time: {stats['avg_update_time']:.4f}s")
        print(f"Server URL: http://127.0.0.1:{self.port}/static/")
    
    def close(self):
        """Close MeshCat visualizer"""
        if self.available and self.vis:
            try:
                self.vis.delete()
                print("🌐 MeshCat visualizer closed")
            except Exception as e:
                print(f"⚠️ Error closing MeshCat: {e}")


class MeshCatURDFRobotVisualizer(MeshCatVisualizer):
    """
    MeshCat visualizer specialized for URDF robots
    Provides interface compatible with SimulationManager
    """
    
    def __init__(self, port: int = 7000, open_browser: bool = True):
        """Initialize MeshCat URDF robot visualizer"""
        super().__init__(port, open_browser)
        
        # Time management
        self.time_manager = None
        self.simulation_manager = None
        
        print("🤖 MeshCat URDF Robot Visualizer initialized")
    
    def connect_time_manager(self, time_manager):
        """Connect time manager for synchronized updates"""
        self.time_manager = time_manager
        print("🔗 Connected to TimeManager for centralized time access")
    
    def connect_simulation_manager(self, simulation_manager):
        """Connect simulation manager for real-time factor control"""
        self.simulation_manager = simulation_manager
        print("🔗 Connected to SimulationManager for real-time factor control")
    
    def load_robot(self, robot_name: str, robot_data: Any) -> bool:
        """
        Load robot compatible with SimulationManager interface
        
        Args:
            robot_name: Name of robot
            robot_data: Robot instance with URDF data
            
        Returns:
            True if successful
        """
        success = self.add_robot(robot_name, robot_data)
        
        if success:
            print(f"\n🤖 Robot Information")
            print(f"Name: {robot_name}")
            
            if hasattr(robot_data, 'urdf_loader'):
                urdf_loader = robot_data.urdf_loader
                print(f"Links: {len(urdf_loader.links)}")
                print(f"Joints: {len(urdf_loader.joints)}")
                
                print(f"\n📦 Links:")
                for link_name in urdf_loader.links.keys():
                    print(f"  - {link_name}")
                
                print(f"\n🔗 Joints:")
                for joint_name, joint in urdf_loader.joints.items():
                    joint_type = getattr(joint, 'type', 'unknown')
                    print(f"  - {joint_name}: {joint_type}")
        
        return success
    
    def update_robot_visualization(self, robot_name: str) -> bool:
        """
        Update robot visualization compatible with SimulationManager
        
        Args:
            robot_name: Name of robot to update
            
        Returns:
            True if successful
        """
        if robot_name not in self.robots:
            return False
        
        try:
            robot = self.robots[robot_name]
            
            # Get current joint positions
            if hasattr(robot, 'get_joint_positions'):
                joint_positions = robot.get_joint_positions()
                return self.update_robot_joints(robot_name, joint_positions)
            
            return True
            
        except Exception as e:
            print(f"⚠️ Error updating robot visualization: {e}")
            return False


# Factory function
def create_meshcat_visualizer(port: int = 7000, open_browser: bool = True) -> MeshCatURDFRobotVisualizer:
    """Create MeshCat URDF robot visualizer"""
    return MeshCatURDFRobotVisualizer(port, open_browser)


if __name__ == "__main__":
    print("🌐 MeshCat Visualizer Test")
    print("=" * 40)
    
    if not MESHCAT_AVAILABLE:
        print("❌ MeshCat not available. Install with:")
        print("   pip install meshcat")
        exit(1)
    
    try:
        # Create visualizer
        viz = MeshCatVisualizer(port=7001, open_browser=True)
        
        if viz.available:
            print("✅ MeshCat visualizer created")
            
            # Add test objects
            from core.simulation_object import Pose
            
            # Add some test shapes
            viz.add_simple_object("box1", "box", Pose(1, 0, 0), [0.5, 0.3, 0.2], 0xff0000)
            viz.add_simple_object("sphere1", "sphere", Pose(0, 1, 0), 0.3, 0x00ff00)
            viz.add_simple_object("cylinder1", "cylinder", Pose(-1, 0, 0), [0.4, 0.2], 0x0000ff)
            
            print("🎯 Test objects added")
            print("🌐 Open http://127.0.0.1:7001/static/ to view")
            
            # Keep running for demonstration
            print("Press Ctrl+C to exit...")
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\n🛑 Shutting down...")
                viz.close()
        else:
            print("❌ Failed to create MeshCat visualizer")
    
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()