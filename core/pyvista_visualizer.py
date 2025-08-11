#!/usr/bin/env python3
"""
PyVista Visualizer for SimPyROS
Provides reusable 3D visualization functionality for robot simulations
"""

import os
import numpy as np
import math
import time
from typing import Optional, Tuple, List, Dict, Union

# Add parent directory to path
import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

# Import URDF loaders in order of preference
try:
    from core.urdf_loader import URDFLoader
    URDF_SUPPORT = True
except ImportError:
    URDF_SUPPORT = False


class PyVistaVisualizer:
    """
    Main PyVista visualizer class for robotics simulation
    
    Features:
    - Automatic headless/interactive mode detection
    - Multiple robot mesh types
    - Scene management (ground, axes, lighting)
    - Interactive and off-screen rendering
    - Animation and trajectory support
    """
    
    def __init__(self, interactive: bool = True, window_size: Tuple[int, int] = (1200, 800)):
        """
        Initialize PyVista visualizer
        
        Args:
            interactive: Enable interactive window (False for headless)
            window_size: Window dimensions (width, height)
        """
        self.interactive = interactive
        self.window_size = window_size
        self.plotter = None
        self.available = False
        
        # Initialize PyVista
        self._initialize_pyvista()
        
        if self.available:
            self._create_plotter()
            
    def _initialize_pyvista(self):
        """Initialize PyVista with proper configuration"""
        try:
            # Configure PyVista environment
            if not self.interactive:
                os.environ['PYVISTA_OFF_SCREEN'] = 'true'
                os.environ['PYVISTA_USE_PANEL'] = 'false'
                os.environ['VTK_SILENCE_GET_VOID_POINTER_WARNINGS'] = '1'
            else:
                os.environ['PYVISTA_OFF_SCREEN'] = 'false'
                os.environ['PYVISTA_USE_PANEL'] = 'false'
            
            # Check if display is available
            self.display_available = bool(os.environ.get('DISPLAY', ''))
            
            # Import PyVista
            import pyvista as pv
            import vtk
            
            self.pv = pv
            self.vtk = vtk
            
            # Configure VTK for headless operation if needed
            if not self.display_available or not self.interactive:
                try:
                    vtk.vtkOpenGLRenderWindow.SetGlobalMaximumNumberOfMultiSamples(0)
                    pv.start_xvfb()  # Start virtual framebuffer if available
                except:
                    pass
                    
                # Force off-screen rendering
                pv.OFF_SCREEN = True
                pv.set_plot_theme('document')
            else:
                pv.OFF_SCREEN = False
                pv.set_plot_theme('document')
            
            self.available = True
            mode = "Interactive" if self.interactive and self.display_available else "Headless"
            print(f"PyVista visualizer initialized ({mode} mode)")
            
        except ImportError as e:
            self.available = False
            print(f"PyVista not available: {e}")
        except Exception as e:
            self.available = False
            print(f"PyVista initialization error: {e}")
            
    def _create_plotter(self):
        """Create PyVista plotter"""
        if not self.available:
            return
            
        try:
            off_screen = not self.interactive or not self.display_available
            self.plotter = self.pv.Plotter(
                off_screen=off_screen, 
                window_size=self.window_size
            )
            self.plotter.set_background('lightblue')
            
        except Exception as e:
            print(f"Plotter creation error: {e}")
            self.available = False
    
    def __del__(self):
        if self.plotter:
            self.plotter.close()



class RobotMeshFactory:
    """Factory class for creating robot meshes"""
    
    @staticmethod
    def create_from_urdf(pv_module, urdf_path: str, package_path: Optional[str] = None) -> Optional:
        """
        Create robot mesh from URDF file
        
        Args:
            pv_module: PyVista module instance
            urdf_path: Path to URDF file
            package_path: Optional path to ROS package directory
            
        Returns:
            Combined PyVista mesh or None
        """
        # Try URDF loader first (supports yourdfpy)
        if URDF_SUPPORT:
            try:
                print("Using URDF loader...")
                loader = URDFLoader(package_path)
                if loader.is_available() and loader.load_urdf(urdf_path):
                    
                    # Create individual meshes with colors
                    meshes = loader.create_pyvista_meshes(pv_module)
                    if meshes:
                        loader.print_info()
                        # For now, return combined mesh but with individual color info stored
                        combined_mesh = None
                        for mesh_info in meshes:
                            mesh = mesh_info['mesh']
                            if combined_mesh is None:
                                combined_mesh = mesh
                            else:
                                combined_mesh = combined_mesh + mesh
                        
                        # Store color information in the combined mesh
                        if combined_mesh and meshes:
                            combined_mesh.mesh_colors = [m['color'] for m in meshes]
                            combined_mesh.mesh_names = [m['name'] for m in meshes]
                            combined_mesh.individual_meshes = meshes
                        
                        return combined_mesh
                    else:
                        print("URDF loader: individual mesh creation failed")
                        
                        # Fallback to original combined mesh method
                        mesh = loader.get_combined_mesh(pv_module)
                        if mesh:
                            loader.print_info()
                            return mesh
                        else:
                            print("URDF loader: mesh creation failed")
                else:
                    print("URDF loader: loading failed")
            except Exception as e:
                print(f"URDF loader error: {e}")
    
    @staticmethod
    def _create_geometric_from_urdf(pv_module, loader: 'URDFLoader') -> Optional:
        """
        Create geometric representation from URDF links when no mesh files are available
        """
        try:
            combined_mesh = None
            
            for link_name, link in loader.links.items():
                # Get link from original URDF
                urdf_link = None
                for ul in loader.robot.links:
                    if ul.name == link_name:
                        urdf_link = ul
                        break
                
                if not urdf_link or not urdf_link.visuals:
                    continue
                
                for visual in urdf_link.visuals:
                    geom = visual.geometry
                    mesh_part = None
                    
                    # Create geometry based on type
                    if hasattr(geom, 'box') and geom.box:
                        size = geom.box.size
                        mesh_part = pv_module.Box(bounds=[
                            -size[0]/2, size[0]/2,
                            -size[1]/2, size[1]/2,
                            -size[2]/2, size[2]/2
                        ])
                    elif hasattr(geom, 'cylinder') and geom.cylinder:
                        radius = geom.cylinder.radius
                        length = geom.cylinder.length
                        mesh_part = pv_module.Cylinder(
                            center=[0, 0, 0],
                            direction=[0, 0, 1],
                            radius=radius,
                            height=length
                        )
                    elif hasattr(geom, 'sphere') and geom.sphere:
                        radius = geom.sphere.radius
                        mesh_part = pv_module.Sphere(radius=radius)
                    
                    if mesh_part:
                        # Apply visual origin transform if available
                        if visual.origin is not None:
                            transform = visual.origin
                            mesh_part.transform(transform, inplace=True)
                        
                        # Apply joint transforms to position link correctly
                        # This is a simplified approach - full FK would be more accurate
                        joint_transform = RobotMeshFactory._get_joint_transform_to_base(loader, link_name)
                        if joint_transform is not None:
                            mesh_part.transform(joint_transform, inplace=True)
                        
                        if combined_mesh is None:
                            combined_mesh = mesh_part
                        else:
                            combined_mesh = combined_mesh + mesh_part
            
            return combined_mesh
            
        except Exception as e:
            print(f"Error creating geometric representation from URDF: {e}")
            return None
    
    @staticmethod
    def _get_joint_transform_to_base(loader: 'URDFLoader', link_name: str) -> Optional[np.ndarray]:
        """Get cumulative transform from base to given link"""
        # This is a simplified version - full implementation would do proper forward kinematics
        try:
            for joint_name, joint in loader.joints.items():
                if joint.child_link == link_name and joint.pose:
                    return joint.pose.to_transformation_matrix()
            return None
        except:
            return None
    


class SceneBuilder:
    """Helper class for building 3D scenes"""
    
    @staticmethod
    def add_ground_plane(plotter, pv_module, 
                        center: Tuple[float, float, float] = (0, 0, -0.2),
                        size: float = 10.0,
                        resolution: int = 20,
                        color: str = 'lightgray',
                        opacity: float = 0.6):
        """Add ground plane to scene"""
        try:
            ground = pv_module.Plane(
                center=center, 
                direction=[0, 0, 1], 
                i_size=size, 
                j_size=size, 
                i_resolution=resolution, 
                j_resolution=resolution
            )
            plotter.add_mesh(ground, color=color, opacity=opacity)
            return True
        except Exception as e:
            print(f"Error adding ground plane: {e}")
            return False
            
    @staticmethod
    def add_coordinate_axes(plotter, pv_module,
                           length: float = 1.0,
                           origin: Tuple[float, float, float] = (0, 0, 0)):
        """Add coordinate axes to scene"""
        try:
            x_axis = pv_module.Arrow(
                start=origin, 
                direction=[1, 0, 0], 
                scale=length
            )
            y_axis = pv_module.Arrow(
                start=origin, 
                direction=[0, 1, 0], 
                scale=length
            )
            z_axis = pv_module.Arrow(
                start=origin, 
                direction=[0, 0, 1], 
                scale=length
            )
            
            plotter.add_mesh(x_axis, color='red')
            plotter.add_mesh(y_axis, color='green')
            plotter.add_mesh(z_axis, color='blue')
            return True
            
        except Exception as e:
            print(f"Error adding coordinate axes: {e}")
            return False
            
    @staticmethod
    def set_camera_view(plotter, 
                       position: Tuple[float, float, float] = (6, 6, 4),
                       focal_point: Tuple[float, float, float] = (0, 0, 1),
                       up: Tuple[float, float, float] = (0, 0, 1)):
        """Set camera position and orientation"""
        try:
            plotter.camera_position = [position, focal_point, up]
            return True
        except Exception as e:
            print(f"Error setting camera view: {e}")
            return False


class AnimationController:
    """Controller for managing animations and trajectories - Optimized version"""
    
    def __init__(self, plotter, pv_module):
        self.plotter = plotter
        self.pv_module = pv_module
        self.trajectories = {}
        self.actors = {}
        self.original_meshes = {}  # Store original meshes for efficient copying
        self.robot_properties = {}  # Store color, opacity, etc.
        
    def add_robot(self, name: str, mesh, 
                  color: str = 'orange', 
                  opacity: float = 0.9):
        """Add a robot to the animation - stores original mesh for efficient updates"""
        try:
            # Check if mesh has individual color information from URDF
            if hasattr(mesh, 'individual_meshes') and mesh.individual_meshes:
                print(f"🎨 Adding robot {name} with individual link colors")
                # Add each link as separate mesh with its own color
                actors = {}
                for i, mesh_info in enumerate(mesh.individual_meshes):
                    link_mesh = mesh_info['mesh']
                    link_color = mesh_info['color']
                    link_name = mesh_info['name']
                    
                    actor_name = f"{name}_{link_name}"
                    actor = self.plotter.add_mesh(
                        link_mesh, 
                        color=link_color, 
                        opacity=opacity, 
                        name=actor_name
                    )
                    actors[link_name] = actor
                    print(f"  Added {link_name} with color {link_color}")
                
                # Store individual actors
                self.actors[name] = actors
                self.original_meshes[name] = mesh.copy()
                self.robot_properties[name] = {'color': 'individual', 'opacity': opacity}
                self.trajectories[name] = []
                
            else:
                # Fallback to single color
                print(f"🎨 Adding robot {name} with uniform color {color}")
                # Store the original mesh and properties
                self.original_meshes[name] = mesh.copy()
                self.robot_properties[name] = {'color': color, 'opacity': opacity}
                
                # Add to plotter
                actor = self.plotter.add_mesh(mesh, color=color, opacity=opacity, name=name)
                self.actors[name] = actor
                self.trajectories[name] = []
                
            return True
        except Exception as e:
            print(f"Error adding robot {name}: {e}")
            return False
    
    def update_robot_pose(self, name: str, pose):
        """Efficiently update robot pose using transformation matrix only"""
        if name not in self.actors or name not in self.original_meshes:
            return False
            
        try:
            # Get transformation matrix
            transform_matrix = pose.to_transformation_matrix()
            
            # Debug: Print position changes (only occasionally)
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0
            
            if self._debug_counter % 50 == 0:  # Every 50 calls
                print(f"Updating robot {name} to position: {pose.position}")
            
            # Method 1: Try to update actor's transformation matrix directly
            try:
                # Convert numpy array to VTK matrix
                try:
                    vtk_matrix = self.pv_module.vtk.vtkMatrix4x4()
                except AttributeError:
                    import vtk
                    vtk_matrix = vtk.vtkMatrix4x4()
                for i in range(4):
                    for j in range(4):
                        vtk_matrix.SetElement(i, j, transform_matrix[i, j])
                
                # Set the transformation matrix on the actor(s)
                actors = self.actors[name]
                if isinstance(actors, dict):
                    # Individual link actors - need to preserve relative positions
                    # Get the robot's mesh info to access original link poses
                    robot_mesh = self.original_meshes.get(name)
                    if hasattr(robot_mesh, 'individual_meshes') and robot_mesh.individual_meshes:
                        # Apply robot transformation while preserving relative positions
                        for i, mesh_info in enumerate(robot_mesh.individual_meshes):
                            link_name = mesh_info['name']
                            if link_name in actors:
                                # Get original relative pose for this link
                                link_pose = mesh_info.get('pose')
                                if link_pose:
                                    # Combine robot pose with link's relative pose
                                    combined_transform = transform_matrix @ link_pose.to_transformation_matrix()
                                else:
                                    # No relative pose info, use robot transform directly
                                    combined_transform = transform_matrix
                                
                                # Create VTK matrix for this specific link
                                try:
                                    link_vtk_matrix = self.pv_module.vtk.vtkMatrix4x4()
                                except AttributeError:
                                    import vtk
                                    link_vtk_matrix = vtk.vtkMatrix4x4()
                                for r in range(4):
                                    for c in range(4):
                                        link_vtk_matrix.SetElement(r, c, combined_transform[r, c])
                                
                                actors[link_name].SetUserMatrix(link_vtk_matrix)
                    else:
                        # Fallback: apply same transform to all links (will collapse robot structure)
                        for link_name, actor in actors.items():
                            actor.SetUserMatrix(vtk_matrix)
                else:
                    # Single actor
                    actors.SetUserMatrix(vtk_matrix)
                
            except Exception:
                # Fallback: Copy and transform mesh (still more efficient than recreating)
                original_mesh = self.original_meshes[name]
                transformed_mesh = original_mesh.copy()
                transformed_mesh.transform(transform_matrix, inplace=True)
                
                # Update the mesh data in the existing actor(s)
                actors = self.actors[name]
                if isinstance(actors, dict):
                    # For individual actors, this fallback is more complex
                    # For now, skip transformation (individual meshes are already positioned)
                    pass
                else:
                    # Single actor
                    mapper = actors.GetMapper()
                    mapper.SetInputData(transformed_mesh)
                    mapper.Modified()
            
            # Update trajectory
            pos = pose.position
            self.trajectories[name].append([pos[0], pos[1], pos[2]])
            
            # Limit trajectory length
            if len(self.trajectories[name]) > 100:
                self.trajectories[name].pop(0)
                
            return True
            
        except Exception as e:
            print(f"Error updating robot {name}: {e}")
            return False
                
    def add_trajectory_trail(self, name: str, 
                           color: str = 'yellow', 
                           line_width: int = 3):
        """Add trajectory trail for a robot"""
        if name not in self.trajectories or len(self.trajectories[name]) < 2:
            return False
            
        try:
            trail_points = np.array(self.trajectories[name])
            trail = self.pv_module.Spline(trail_points, n_points=len(trail_points))
            self.plotter.add_mesh(trail, color=color, line_width=line_width, name=f'{name}_trail')
            return True
        except Exception as e:
            print(f"Error adding trajectory trail for {name}: {e}")
            return False


# Convenience functions for quick setup
def create_interactive_visualizer(window_size: Tuple[int, int] = (1200, 800)) -> PyVistaVisualizer:
    """Create an interactive PyVista visualizer"""
    return PyVistaVisualizer(interactive=True, window_size=window_size)


def create_headless_visualizer(window_size: Tuple[int, int] = (800, 600)) -> PyVistaVisualizer:
    """Create a headless PyVista visualizer for image generation"""
    return PyVistaVisualizer(interactive=False, window_size=window_size)


def setup_basic_scene(visualizer: PyVistaVisualizer) -> bool:
    """Setup a basic scene with ground plane and coordinate axes"""
    if not visualizer.available:
        return False
        
    success = True
    success &= SceneBuilder.add_ground_plane(visualizer.plotter, visualizer.pv)
    success &= SceneBuilder.add_coordinate_axes(visualizer.plotter, visualizer.pv)
    success &= SceneBuilder.set_camera_view(visualizer.plotter)
    
    return success


# Robot mesh creation functions
def create_robot_mesh_from_urdf(visualizer: PyVistaVisualizer, urdf_path: str = None, package_path: str = None):
    """Create a robot mesh from URDF file"""
    if not visualizer.available:
        return None
        
    if not urdf_path:
        print("URDF path required for robot_type='urdf'")
        return None
    return RobotMeshFactory.create_from_urdf(visualizer.pv, urdf_path, package_path)


class URDFRobotVisualizer(PyVistaVisualizer):
    """
    Extended PyVista visualizer with integrated URDF robot loading and visualization
    
    Features:
    - Load robots directly from URDF files
    - Real-time joint motion visualization  
    - Individual link coloring and management
    - Simple load -> set_joint_command workflow
    """
    
    def __init__(self, interactive: bool = True, window_size: Tuple[int, int] = (1200, 800)):
        """Initialize URDF robot visualizer"""
        super().__init__(interactive, window_size)
        
        # Robot management
        self.robots: Dict[str, Any] = {}  # Store robot objects
        self.urdf_loaders: Dict[str, Any] = {}  # Store URDF loaders
        self.link_actors: Dict[str, Dict[str, Any]] = {}  # Store link actors per robot
        self.animation_controllers: Dict[str, AnimationController] = {}
        
        # Setup basic scene if available
        if self.available:
            setup_basic_scene(self)
    
    def load_robot(self, robot_name: str, robot_instance, urdf_path: str) -> bool:
        """
        Load a robot for visualization and control
        
        Args:
            robot_name: Unique name for this robot instance
            robot_instance: Robot object from robot.py
            urdf_path: Path to URDF file
            
        Returns:
            bool: Success status
        """
        if not self.available:
            print("❌ PyVista not available")
            return False
        
        print(f"🤖 Loading robot '{robot_name}' from {urdf_path}")
        
        try:
            # Import URDF loader locally to avoid circular imports
            from core.urdf_loader import URDFLoader
            
            # Create URDF loader
            urdf_loader = URDFLoader()
            if not urdf_loader.load_urdf(urdf_path):
                print(f"❌ Failed to load URDF: {urdf_path}")
                return False
            
            # Store robot and loader
            self.robots[robot_name] = robot_instance
            self.urdf_loaders[robot_name] = urdf_loader
            
            # Create individual link meshes with real-time updates
            self._create_robot_link_actors(robot_name)
            
            # Create animation controller for this robot
            animation_controller = AnimationController(self.plotter, self.pv)
            self.animation_controllers[robot_name] = animation_controller
            
            print(f"✅ Robot '{robot_name}' loaded successfully")
            urdf_loader.print_info()
            return True
            
        except Exception as e:
            print(f"❌ Failed to load robot '{robot_name}': {e}")
            return False
    
    def _create_robot_link_actors(self, robot_name: str):
        """Create individual link actors for a robot"""
        urdf_loader = self.urdf_loaders[robot_name]
        link_actors = {}
        
        print(f"🎨 Creating individual link meshes for '{robot_name}'...")
        
        for link_name, link_info in urdf_loader.links.items():
            # Create mesh based on geometry type
            mesh = None
            
            if link_info.geometry_type == "box":
                size = link_info.geometry_params.get('size', [0.3, 0.3, 0.1])
                mesh = self.pv.Cube(x_length=size[0], y_length=size[1], z_length=size[2])
                
            elif link_info.geometry_type == "cylinder":
                radius = link_info.geometry_params.get('radius', 0.05)
                length = link_info.geometry_params.get('length', 0.35)
                mesh = self.pv.Cylinder(radius=radius, height=length, direction=(0, 0, 1))
                
            elif link_info.geometry_type == "sphere":
                radius = link_info.geometry_params.get('radius', 0.08)
                mesh = self.pv.Sphere(radius=radius)
            
            if mesh is not None:
                # Apply visual origin transformation if present
                if hasattr(link_info, 'pose') and link_info.pose is not None:
                    transform_matrix = link_info.pose.to_transformation_matrix()
                    mesh.transform(transform_matrix, inplace=True)
                    
                    # Debug output for non-identity transformations
                    pos = link_info.pose.position
                    if not all(abs(x) < 0.001 for x in pos):
                        print(f"    🔧 Applied visual origin to {link_name}: pos={pos}")
                
                # Add to scene with link color
                color = link_info.color[:3]  # RGB only
                actor = self.plotter.add_mesh(
                    mesh, 
                    color=color, 
                    opacity=0.8, 
                    name=f"{robot_name}_{link_name}"
                )
                link_actors[link_name] = {
                    'actor': actor,
                    'mesh': mesh.copy(),  # Store original mesh
                    'color': color,
                    'geometry_type': link_info.geometry_type
                }
                print(f"  Added {link_name}: {link_info.geometry_type} with color {color}")
        
        self.link_actors[robot_name] = link_actors
        print(f"✅ Created {len(link_actors)} individual links for '{robot_name}'")
    
    def update_robot_visualization(self, robot_name: str):
        """Update robot visualization based on current joint positions"""
        if robot_name not in self.robots or robot_name not in self.link_actors:
            return False
        
        try:
            robot = self.robots[robot_name]
            link_actors = self.link_actors[robot_name]
            
            # Get current link poses from robot's forward kinematics
            link_poses = robot.get_link_poses()
            
            for link_name, pose in link_poses.items():
                if link_name in link_actors and pose is not None:
                    # Create transformation matrix
                    transform_matrix = pose.to_transformation_matrix()
                    
                    # Update mesh position
                    original_mesh = link_actors[link_name]['mesh'].copy()
                    original_mesh.transform(transform_matrix)
                    
                    # Update the actor
                    actor = link_actors[link_name]['actor']
                    if hasattr(actor, 'GetMapper'):
                        mapper = actor.GetMapper()
                        mapper.SetInputData(original_mesh)
                        mapper.Modified()
            
            return True
            
        except Exception as e:
            print(f"⚠️ Error updating robot visualization '{robot_name}': {e}")
            return False
    
    def set_joint_command(self, robot_name: str, joint_name: str, position: float, max_velocity: Optional[float] = None):
        """
        Simple interface to set joint position and update visualization
        
        Args:
            robot_name: Name of the robot
            joint_name: Name of the joint to move
            position: Target position in radians
            max_velocity: Maximum velocity (optional)
        """
        if robot_name not in self.robots:
            print(f"❌ Robot '{robot_name}' not found")
            return False
        
        robot = self.robots[robot_name]
        robot.set_joint_position(joint_name, position, max_velocity)
        
        # Update visualization immediately
        return self.update_robot_visualization(robot_name)
    
    def set_joint_commands(self, robot_name: str, joint_positions: Dict[str, float], max_velocity: Optional[float] = None):
        """
        Set multiple joint positions simultaneously
        
        Args:
            robot_name: Name of the robot
            joint_positions: Dictionary of joint_name -> position
            max_velocity: Maximum velocity for all joints
        """
        if robot_name not in self.robots:
            print(f"❌ Robot '{robot_name}' not found")
            return False
        
        robot = self.robots[robot_name]
        robot.set_joint_positions(joint_positions, max_velocity)
        
        # Update visualization immediately
        return self.update_robot_visualization(robot_name)
    
    def get_robot_info(self, robot_name: str) -> Optional[Dict]:
        """Get robot information including joint names and current positions"""
        if robot_name not in self.robots:
            return None
        
        robot = self.robots[robot_name]
        return {
            'joint_names': robot.get_joint_names(),
            'joint_positions': robot.get_joint_positions(),
            'joint_velocities': robot.get_joint_velocities(),
            'movable_joints': [name for name in robot.get_joint_names() 
                             if robot.joints[name].joint_type.value != 'fixed']
        }
    
    def start_continuous_update(self, robot_name: str, update_rate: float = 60.0):
        """Start continuous visualization updates (for real-time animation)"""
        if robot_name not in self.robots:
            return False
        
        import threading
        import time
        
        def update_loop():
            dt = 1.0 / update_rate
            while robot_name in self.robots:
                self.update_robot_visualization(robot_name)
                time.sleep(dt)
        
        thread = threading.Thread(target=update_loop)
        thread.daemon = True
        thread.start()
        return True


# Convenience functions for URDF robot visualization
def create_urdf_robot_visualizer(interactive: bool = True, window_size: Tuple[int, int] = (1200, 800)) -> URDFRobotVisualizer:
    """Create a URDF robot visualizer with basic scene setup"""
    return URDFRobotVisualizer(interactive, window_size)

