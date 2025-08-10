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

# Import URDF loaders in order of preference
try:
    from urdf_loader import URDFLoader
    URDF_SUPPORT = True
except ImportError:
    URDF_SUPPORT = False

try:
    from legacy.loaders.urdf_loader import URDFLoader as LegacyURDFLoader
    LEGACY_URDF_SUPPORT = True
except ImportError:
    LEGACY_URDF_SUPPORT = False

try:
    from simple_urdf_loader import SimpleURDFLoader
    SIMPLE_URDF_SUPPORT = True
except ImportError:
    SIMPLE_URDF_SUPPORT = False


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
        
        # Fallback to simple URDF loader (no heavy dependencies)
        if SIMPLE_URDF_SUPPORT:
            try:
                print("Using simple URDF loader...")
                simple_loader = SimpleURDFLoader()
                if simple_loader.load_urdf(urdf_path):
                    mesh = simple_loader.create_pyvista_mesh(pv_module)
                    if mesh:
                        print(f"✅ Successfully created robot mesh from URDF (simple loader): {urdf_path}")
                        simple_loader.print_info()
                        return mesh
                    else:
                        print("⚠️  Simple loader: No mesh created")
            except Exception as e:
                print(f"Simple URDF loader failed: {e}")
        
        # Fallback to legacy URDF loader
        if LEGACY_URDF_SUPPORT:
            try:
                print("Trying legacy URDF loader...")
                loader = LegacyURDFLoader(package_path=package_path)
                if not loader.is_available():
                    print("❌ Legacy URDF loader dependencies not available")
                    return None
                    
                if not loader.load_urdf(urdf_path):
                    print(f"❌ Failed to load URDF: {urdf_path}")
                    return None
                
                # Get combined mesh
                mesh = loader.get_combined_mesh(pv_module)
                if mesh:
                    print(f"✅ Successfully created robot mesh from URDF (legacy loader): {urdf_path}")
                    return mesh
                else:
                    print(f"⚠️  No meshes found in URDF file: {urdf_path}")
                    # Fallback: create geometric representation from URDF structure
                    return RobotMeshFactory._create_geometric_from_urdf(pv_module, loader)
                    
            except Exception as e:
                print(f"❌ Legacy URDF loader error: {e}")
        
        print("❌ All URDF loaders failed. Install yourdfpy and trimesh.")
        return None
    
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
    
    @staticmethod
    def create_basic_robot(pv_module) -> Optional:
        """Create a basic robot with base, arm, and gripper"""
        try:
            # Robot base (box)
            base = pv_module.Box(bounds=[-0.5, 0.5, -0.3, 0.3, 0, 0.2])
            
            # Robot arm (cylinder)
            arm = pv_module.Cylinder(
                center=[0.3, 0, 0.4], 
                direction=[0, 0, 1], 
                radius=0.1, 
                height=0.6,
                resolution=16
            )
            
            # End effector (sphere)
            gripper = pv_module.Sphere(
                center=[0.3, 0, 0.8], 
                radius=0.08,
                phi_resolution=16, 
                theta_resolution=16
            )
            
            # Combine meshes
            robot = base + arm + gripper
            return robot
            
        except Exception as e:
            print(f"Error creating basic robot mesh: {e}")
            return None
            
    @staticmethod
    def create_wheeled_robot(pv_module) -> Optional:
        """Create a wheeled robot (like a mobile base)"""
        try:
            # Robot base
            base = pv_module.Box(bounds=[-0.5, 0.5, -0.3, 0.3, 0, 0.2])
            
            # Robot arm
            arm = pv_module.Cylinder(
                center=[0.3, 0, 0.4], 
                direction=[0, 0, 1], 
                radius=0.1, 
                height=0.6,
                resolution=16
            )
            
            # End effector
            gripper = pv_module.Sphere(
                center=[0.3, 0, 0.8], 
                radius=0.08,
                phi_resolution=16, 
                theta_resolution=16
            )
            
            # Wheels
            wheel_positions = [[-0.4, -0.35, -0.1], [-0.4, 0.35, -0.1], 
                             [0.4, -0.35, -0.1], [0.4, 0.35, -0.1]]
            wheels = []
            for pos in wheel_positions:
                wheel = pv_module.Cylinder(
                    center=pos, 
                    direction=[0, 1, 0], 
                    radius=0.12, 
                    height=0.05, 
                    resolution=12
                )
                wheels.append(wheel)
            
            # Combine all parts
            robot = base + arm + gripper
            for wheel in wheels:
                robot = robot + wheel
                
            return robot
            
        except Exception as e:
            print(f"Error creating wheeled robot mesh: {e}")
            return None
            
    @staticmethod
    def create_quadcopter(pv_module) -> Optional:
        """Create a quadcopter mesh"""
        try:
            # Center body
            center = pv_module.Sphere(
                center=[0, 0, 0], 
                radius=0.05, 
                phi_resolution=10, 
                theta_resolution=10
            )
            
            # Propeller arms and props
            arms = []
            for angle in [0, 90, 180, 270]:
                rad = math.radians(angle)
                arm_end = [0.2 * math.cos(rad), 0.2 * math.sin(rad), 0]
                
                arm = pv_module.Cylinder(
                    center=[arm_end[0]/2, arm_end[1]/2, 0], 
                    direction=arm_end, 
                    radius=0.01, 
                    height=0.2, 
                    resolution=6
                )
                prop = pv_module.Cylinder(
                    center=arm_end, 
                    direction=[0, 0, 1], 
                    radius=0.05, 
                    height=0.005, 
                    resolution=8
                )
                arms.append(arm + prop)
            
            result = center
            for arm in arms:
                result = result + arm
            return result
            
        except Exception as e:
            print(f"Error creating quadcopter mesh: {e}")
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
                           length: float = 1.5,
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
    
    def update_robot_pose_efficient(self, name: str, pose):
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
    
    def update_robot_pose(self, name: str, pose, mesh_creator_func=None):
        """Update robot pose - optimized version that doesn't recreate meshes"""
        # Use the efficient method by default
        return self.update_robot_pose_efficient(name, pose)
    
    def update_robot_pose_fallback(self, name: str, pose, mesh_creator_func):
        """Fallback method for complex updates that require mesh recreation"""
        if name not in self.actors:
            return False
            
        try:
            # Create new mesh at updated pose (only when necessary)
            new_mesh = mesh_creator_func()
            if new_mesh is None:
                return False
                
            # Transform mesh to new pose
            transform_matrix = pose.to_transformation_matrix()
            new_mesh.transform(transform_matrix, inplace=True)
            
            # Remove old actor and add new one
            if self.actors[name]:
                self.plotter.remove_actor(self.actors[name])
            
            # Get original properties
            props = self.robot_properties.get(name, {'color': 'orange', 'opacity': 0.9})
            
            actor = self.plotter.add_mesh(new_mesh, 
                                        color=props['color'], 
                                        opacity=props['opacity'], 
                                        name=name)
            self.actors[name] = actor
            
            # Update stored mesh
            self.original_meshes[name] = mesh_creator_func()
            
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


# Example robot creation functions using the factory
def create_robot_mesh(visualizer: PyVistaVisualizer, robot_type: str = 'basic', urdf_path: str = None, package_path: str = None):
    """
    Create a robot mesh using the factory
    
    Args:
        visualizer: PyVista visualizer instance
        robot_type: Type of robot ('basic', 'wheeled', 'quadcopter', 'urdf')
        urdf_path: Path to URDF file (required if robot_type='urdf')
        package_path: Optional ROS package path for resolving package:// URIs
    """
    if not visualizer.available:
        return None
        
    if robot_type == 'urdf':
        if not urdf_path:
            print("URDF path required for robot_type='urdf'")
            return None
        return RobotMeshFactory.create_from_urdf(visualizer.pv, urdf_path, package_path)
    elif robot_type == 'basic':
        return RobotMeshFactory.create_basic_robot(visualizer.pv)
    elif robot_type == 'wheeled':
        return RobotMeshFactory.create_wheeled_robot(visualizer.pv)
    elif robot_type == 'quadcopter':
        return RobotMeshFactory.create_quadcopter(visualizer.pv)
    else:
        return RobotMeshFactory.create_basic_robot(visualizer.pv)


if __name__ == "__main__":
    # Simple test
    viz = create_interactive_visualizer()
    if viz.available:
        setup_basic_scene(viz)
        robot = create_robot_mesh(viz, 'wheeled')
        if robot:
            viz.plotter.add_mesh(robot, color='orange')
        print("Test visualization created. Close window to exit.")
        viz.plotter.show()
    else:
        print("PyVista not available for testing")