#!/usr/bin/env python3
"""
PyVista Visualizer for SimPyROS
Provides reusable 3D visualization functionality for robot simulations
"""

import os
import numpy as np
import math
import time
import warnings
from typing import Optional, Tuple, List, Dict, Union, Any

# Add parent directory to path
import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

# Remove unused common imports - no longer needed

# Import URDF loaders in order of preference
try:
    from ..urdf_loader import URDFLoader
    URDF_SUPPORT = True
except ImportError:
    URDF_SUPPORT = False

# Import time management
try:
    from ..time_manager import TimeManager, get_global_time_manager
    TIME_MANAGER_SUPPORT = True
except ImportError:
    TIME_MANAGER_SUPPORT = False


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
        
        # Use PyVista core handler (composition pattern)
        self._pyvista_core = PyVistaCoreHandler()
        
        # Initialize PyVista using core handler
        self.available = self._pyvista_core.initialize_pyvista(interactive=interactive)
        
        if self.available:
            success = self._pyvista_core.create_plotter(window_size=window_size)
            if success:
                # Expose core handler's modules and plotter for compatibility
                self.pv = self._pyvista_core.pv
                self.vtk = self._pyvista_core.vtk
                self.plotter = self._pyvista_core.plotter
            else:
                self.available = False
        
        # Setup camera controls and basic scene for interactive mode
        if self.available and self.interactive:
            self._pyvista_core.setup_camera_controls()
            self._pyvista_core.setup_basic_scene(add_ground=True, add_axes=False)
            
    # PyVista initialization now handled by PyVistaCoreHandler (composition pattern)
    
    def batch_mode(self):
        """Context manager for batch rendering to improve performance"""
        return BatchRenderingContext(self)
    
    def update_simulation_object(self, object_name: str, obj) -> bool:
        """Update visualization for a simulation object (box, sphere, etc.)
        
        Args:
            object_name: Name of the simulation object
            obj: SimulationObject instance
            
        Returns:
            bool: True if update succeeded
        """
        if not self.available or not self.plotter:
            return False
        
        try:
            # Check if object has an associated mesh actor
            if hasattr(obj, '_mesh_actor') and obj._mesh_actor is not None:
                # Update transformation matrix
                transform_matrix = obj.pose.to_transformation_matrix()
                
                # Set transformation on the mesh actor
                try:
                    vtk_matrix = self.pv.vtk.vtkMatrix4x4()
                    for i in range(4):
                        for j in range(4):
                            vtk_matrix.SetElement(i, j, transform_matrix[i, j])
                    obj._mesh_actor.SetUserMatrix(vtk_matrix)
                    return True
                except Exception as e:
                    print(f"⚠️ Failed to update {object_name} transformation: {e}")
                    return False
            else:
                # Object not yet visualized, create mesh if needed
                return self.create_object_mesh(object_name, obj)
                
        except Exception as e:
            print(f"⚠️ Failed to update visualization for {object_name}: {e}")
            return False

    def create_object_mesh(self, object_name: str, obj) -> bool:
        """Create mesh representation for simulation object
        
        Args:
            object_name: Name of the simulation object  
            obj: SimulationObject instance
            
        Returns:
            bool: True if mesh creation succeeded
        """
        if not self.available or not self.plotter:
            return False
        
        try:
            mesh = None
            # Create mesh based on object parameters
            if hasattr(obj.parameters, 'geometry_type'):
                geometry_type = obj.parameters.geometry_type
                params = getattr(obj.parameters, 'geometry_params', {})
                
                if geometry_type == "box":
                    size = params.get('size', [0.3, 0.3, 0.3])
                    mesh = self.pv.Cube(x_length=size[0], y_length=size[1], z_length=size[2])
                elif geometry_type == "cylinder":
                    radius = params.get('radius', 0.05)
                    height = params.get('height', 0.2)
                    mesh = self.pv.Cylinder(radius=radius, height=height, direction=(0, 0, 1))
                elif geometry_type == "sphere":
                    radius = params.get('radius', 0.1)
                    mesh = self.pv.Sphere(radius=radius)
            
            if mesh is not None:
                # Apply current transformation
                transform_matrix = obj.pose.to_transformation_matrix()
                mesh.transform(transform_matrix, inplace=True)
                
                # Get color
                color = getattr(obj.parameters, 'color', (0.6, 0.6, 0.6))
                if len(color) > 3:
                    color = color[:3]  # RGB only for PyVista
                
                # Add to scene and store reference
                actor = self.plotter.add_mesh(
                    mesh, 
                    color=color, 
                    opacity=0.8, 
                    name=f"object_{object_name}"
                )
                obj._mesh_actor = actor
                return True
                
        except Exception as e:
            print(f"⚠️ Failed to create mesh for {object_name}: {e}")
            
        return False

    def __del__(self):
        if hasattr(self, '_pyvista_core'):
            self._pyvista_core.cleanup()


class BatchRenderingContext:
    """Context manager for efficient batch rendering in PyVista"""
    
    def __init__(self, visualizer):
        self.visualizer = visualizer
        self.render_needed = False
        
    def __enter__(self):
        # Defer rendering until batch is complete
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        # Perform single render at end of batch for performance, then lightweight event polling
        # Option A implementation:
        #  - Use plotter.render() once (fast path, no full update cycle duplication)
        #  - Immediately process pending UI events (camera, mouse, keyboard) WITHOUT calling plotter.update()
        #    because plotter.update() would invoke an additional render (double cost per frame)
        #  - VTK interactor event processing is significantly cheaper than a full render traversal
        if self.visualizer.available and self.visualizer.plotter:
            try:
                p = self.visualizer.plotter
                p.render()
                # Try to process pending events to keep camera interactive
                # Different PyVista versions expose interactor as .iren; guard defensively.
                iren = getattr(p, 'iren', None)
                if iren is not None:
                    try:
                        # Process a single batch of pending events (non-blocking)
                        iren.ProcessEvents()
                    except Exception:
                        pass
                # Alternative backends (Qt) may expose an application event processor
                app = getattr(p, 'app', None)
                if app is not None:
                    try:
                        app.processEvents()
                    except Exception:
                        pass
            except Exception as e:
                print(f"⚠️ Batch rendering error: {e}")


# Convenience functions for quick setup
def create_interactive_visualizer(window_size: Tuple[int, int] = (1200, 800)) -> PyVistaVisualizer:
    """Create an interactive PyVista visualizer"""
    return PyVistaVisualizer(interactive=True, window_size=window_size)


def setup_basic_scene(visualizer: PyVistaVisualizer) -> bool:
    """Setup a basic scene with ground plane and coordinate axes (now uses core handler)"""
    if not visualizer.available or not hasattr(visualizer, '_pyvista_core'):
        return False
    
    # Use core handler for scene setup
    success = visualizer._pyvista_core.setup_basic_scene(add_ground=True, add_axes=False)
    return success


# Import base class and core handler
from .base_robot_visualizer import BaseRobotVisualizer
from .pyvista_core import PyVistaCoreHandler

class URDFRobotVisualizer(PyVistaVisualizer, BaseRobotVisualizer):
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
        PyVistaVisualizer.__init__(self, interactive, window_size)
        BaseRobotVisualizer.__init__(self)
        
        # Robot management (robots dict inherited from BaseRobotVisualizer)
        # Note: urdf_loaders removed - visual data now stored directly in Robot.links
        self.link_actors: Dict[str, Dict[str, Any]] = {}  # Store link actors per robot
        # Animation controllers removed - unused functionality
                
        # Setup basic scene and controls if available
        if self.available:
            setup_basic_scene(self)
    
    # Connection methods inherited from BaseRobotVisualizer
    
    # load_robot() inherited from BaseRobotVisualizer - no override needed
    
    def _load_robot_specific(self, robot_name: str, robot_instance) -> bool:
        """URDFRobotVisualizer-specific loading implementation"""
        try:
            # Create individual link meshes using robot's link/joint info
            self._create_robot_link_actors_from_robot(robot_name, robot_instance)
            
            # Debug info
            print(f"📚 Robot '{robot_name}' visual data loaded from Link objects")
            print(f"    Links: {len(robot_instance.links)}")
            return True
            
        except Exception as e:
            print(f"❌ URDFRobotVisualizer specific loading failed: {e}")
            return False
    
    def _create_robot_link_actors_from_robot(self, robot_name: str, robot_instance):
        """Create individual link actors using robot instance data - Unified mesh creation"""
        from .robot_mesh_creator import create_robot_mesh_creator
        
        link_actors = {}
        
        print(f"🎨 Creating individual link meshes for '{robot_name}' using shared mesh creator...")
        
        # Initialize shared mesh creator
        mesh_creator = create_robot_mesh_creator(self.pv)
        
        for link_name, link_obj in robot_instance.links.items():
            # Prepare link data for mesh creator
            link_data = {
                'geometry_type': link_obj.geometry_type,
                'geometry_params': link_obj.geometry_params,
                'color': link_obj.color
            }
            
            # Prepare URDF visual origin data from Link object
            urdf_link_data = None
            if hasattr(link_obj, 'visual_pose') and link_obj.visual_pose is not None:
                pos = link_obj.visual_pose.position
                quat = link_obj.visual_pose.rotation.as_quat()
                urdf_link_data = {
                    'has_pose': True,
                    'position': pos,
                    'quaternion': quat
                }
                print(f"    🔧 Will apply visual origin to {link_name}: pos={pos}")
            
            # Create mesh using shared creator
            mesh, color_rgb = mesh_creator.create_robot_link_mesh(link_name, link_data, urdf_link_data)
            
            if mesh is not None:
                # Add to scene using shared creator
                actor = mesh_creator.add_mesh_to_plotter(self.plotter, mesh, robot_name, link_name, color_rgb)
                
                link_actors[link_name] = {
                    'actor': actor,
                    'mesh': mesh.copy(),  # Store original mesh
                    'color': color_rgb,
                    'geometry_type': link_obj.geometry_type
                }
                print(f"  Added {link_name}: {link_obj.geometry_type} with color {color_rgb}")
        
        self.link_actors[robot_name] = link_actors
        print(f"✅ Created {len(link_actors)} individual links for '{robot_name}' using shared mesh creator")
    
    # All mesh creation now uses Robot instance data directly (memo.txt requirement 37)
    # RobotMeshFactory integration complete - old URDF file processing removed
    
    # update_robot_visualization() inherited from BaseRobotVisualizer - no override needed
    # Uses Template Method pattern with _update_visualization_specific()
    
    def _update_visualization_specific(self, robot_name: str, robot_data: Any, link_poses: Dict, force_render: bool) -> bool:
        """URDFRobotVisualizer-specific visualization update implementation"""
        if robot_name not in self.link_actors:
            return False
        
        try:
            link_actors = self.link_actors[robot_name]
            
            for link_name, pose in link_poses.items():
                if link_name in link_actors and pose is not None:
                    # Create transformation matrix
                    transform_matrix = pose.to_transformation_matrix()
                    
                    # Get actor reference
                    actor = link_actors[link_name]['actor']
                    
                    # Update mesh position using transformation matrix directly on actor
                    try:
                        # Convert numpy array to VTK matrix for efficient update
                        vtk_matrix = self.pv.vtk.vtkMatrix4x4()
                        for i in range(4):
                            for j in range(4):
                                vtk_matrix.SetElement(i, j, transform_matrix[i, j])
                        
                        # Set the transformation matrix on the actor
                        actor.SetUserMatrix(vtk_matrix)
                        
                    except Exception as e:
                        # Fallback: copy and transform mesh
                        original_mesh = link_actors[link_name]['mesh'].copy()
                        original_mesh.transform(transform_matrix, inplace=True)
                        
                        # Update the actor using mapper
                        if hasattr(actor, 'GetMapper'):
                            mapper = actor.GetMapper()
                            mapper.SetInputData(original_mesh)
                            mapper.Modified()
            
            return True
            
        except Exception as e:
            print(f"❌ URDFRobotVisualizer specific visualization update failed: {e}")
            return False
    
    # remove_robot() inherited from BaseRobotVisualizer - no override needed
    # Uses Template Method pattern with _remove_robot_specific()
    
    def _remove_robot_specific(self, robot_name: str) -> bool:
        """URDFRobotVisualizer-specific removal implementation"""
        try:
            # Remove from actors
            if robot_name in self.link_actors:
                for link_name, link_info in self.link_actors[robot_name].items():
                    if 'actor' in link_info and link_info['actor'] is not None:
                        try:
                            self.plotter.remove_actor(link_info['actor'])
                        except Exception as e:
                            print(f"⚠️ Failed to remove actor for {robot_name}_{link_name}: {e}")
                del self.link_actors[robot_name]
            
            # Note: urdf_loaders no longer used - visual data is in Robot.links
            
            return True
            
        except Exception as e:
            print(f"❌ URDFRobotVisualizer specific removal failed: {e}")
            return False

# Convenience functions for URDF robot visualization
def create_urdf_robot_visualizer(interactive: bool = True, window_size: Tuple[int, int] = (1200, 800)) -> URDFRobotVisualizer:
    """Create a URDF robot visualizer with basic scene setup"""
    return URDFRobotVisualizer(interactive, window_size)

