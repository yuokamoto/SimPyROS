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

# Import time management
try:
    from core.time_manager import TimeManager, get_global_time_manager
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
            
            # Enable non-blocking mode for performance (critical for real-time simulation)
            if self.interactive and self.available:
                # Non-blocking show for interactive mode
                self._setup_non_blocking_mode()
            
        except Exception as e:
            print(f"Plotter creation error: {e}")
            self.available = False
    
    def _setup_non_blocking_mode(self):
        """Setup non-blocking mode for PyVista to prevent simulation blocking"""
        try:
            # Enable interactive camera controls
            self._setup_camera_controls()
            
            # Show window in non-blocking mode
            self.plotter.show(
                auto_close=False,
                interactive_update=True,
                full_screen=False
            )
            print("⚡ PyVista non-blocking mode enabled for real-time simulation")
            print("🖱️  Interactive camera controls enabled:")
            print("     Left Click+Drag: Rotate")
            print("     Right Click+Drag: Zoom")
            print("     Middle Click+Drag: Pan")
            print("     Mouse Wheel: Zoom In/Out")
        except Exception as e:
            print(f"⚠️ Could not enable non-blocking mode: {e}")
    
    def _setup_camera_controls(self):
        """Setup enhanced camera controls for interactive navigation"""
        if not self.plotter:
            return
            
        try:
            # Enable all interactive modes
            self.plotter.enable_trackball_style()  # Better rotation control
            
            # Set initial camera position for good 3D view
            self.plotter.camera.position = (10, 10, 10)
            self.plotter.camera.focal_point = (0, 0, 0)
            self.plotter.camera.up = (0, 0, 1)  # Z-up orientation
            
            # Enable depth peeling for better transparency
            if hasattr(self.plotter, 'enable_depth_peeling'):
                self.plotter.enable_depth_peeling()
                
            # Set camera controls
            self.plotter.camera.view_angle = 60  # Field of view
            self.plotter.camera.clipping_range = (0.1, 1000)  # Near/far clipping
            
            print("📷 Enhanced camera controls configured")
            
        except Exception as e:
            print(f"⚠️ Camera setup error: {e}")
    
    def batch_mode(self):
        """Context manager for batch rendering to improve performance"""
        return BatchRenderingContext(self)
    
    def __del__(self):
        if self.plotter:
            self.plotter.close()


class BatchRenderingContext:
    """Context manager for efficient batch rendering in PyVista"""
    
    def __init__(self, visualizer):
        self.visualizer = visualizer
        self.render_needed = False
        
    def __enter__(self):
        # Defer rendering until batch is complete
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        # Perform single render at end of batch for performance
        if self.visualizer.available and self.visualizer.plotter:
            try:
                self.visualizer.plotter.render()
                # Update time display after batch completion
                self.visualizer.update_time_display()
            except Exception as e:
                print(f"⚠️ Batch rendering error: {e}")



# RobotMeshFactory has been completely integrated into URDFRobotVisualizer (memo.txt 36, 37)
# All robot visualization now uses Robot instance data directly via load_robot() method


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
            
            plotter.add_mesh(x_axis, color='red', name='coord_axis_x')
            plotter.add_mesh(y_axis, color='green', name='coord_axis_y')
            plotter.add_mesh(z_axis, color='blue', name='coord_axis_z')
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
    # Skip coordinate axes by default - user can enable via toggle
    # success &= SceneBuilder.add_coordinate_axes(visualizer.plotter, visualizer.pv)
    success &= SceneBuilder.set_camera_view(visualizer.plotter)
    
    return success


# Deprecated function removed - use URDFRobotVisualizer.load_robot() instead


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
        
        # Interactive controls state (6.3)
        self.controls_enabled = True
        self.axis_display_visible = False  # Default to disabled coordinate axes
        self.realtime_factor = 1.0
        self.collision_display_visible = False
        self.wireframe_mode = False
        
        # Simulation control state
        self.simulation_paused = False
        self.simulation_running = False
        
        # TimeManager connection for centralized time access
        self._connected_time_manager = None
        self._connected_simulation_manager = None  # Legacy compatibility
        
        # Time display tracking
        self._simulation_start_time = None
        self._time_text_actor = None
        
        # Setup basic scene and controls if available
        if self.available:
            setup_basic_scene(self)
            if interactive:
                self._setup_interactive_controls()
    
    def _setup_interactive_controls(self):
        """Setup interactive UI controls using PyVista widgets"""
        if not self.available or not hasattr(self, 'plotter') or not self.plotter:
            return
        
        try:
            # Add checkbox for axis display toggle (smaller button with text overlay)
            self.plotter.add_checkbox_button_widget(
                callback=self._toggle_axis_display,
                value=self.axis_display_visible,
                position=(10, 10),
                size=30,  # Smaller button
                border_size=1,
                color_on='green',
                color_off='red'
            )
            # Overlay text on button - larger font and better positioning
            self.plotter.add_text("🎯", position=(18, 18), font_size=12, color='white')
            
            # Add slider for real-time factor control
            self.plotter.add_slider_widget(
                callback=self._update_realtime_factor,
                rng=[0.1, 5.0],
                value=self.realtime_factor,
                title="Real-time Factor",
                pointa=(0.1, 0.9),
                pointb=(0.4, 0.9),
                style='modern'
            )
            
            # Add collision toggle button (smaller with overlay text)
            self.plotter.add_checkbox_button_widget(
                callback=self._toggle_collision_display,
                value=self.collision_display_visible,
                position=(10, 50),
                size=30,  # Smaller button
                border_size=1,
                color_on='yellow',
                color_off='gray'
            )
            # Overlay text on button - larger font and better positioning
            self.plotter.add_text("🚧", position=(18, 58), font_size=12, color='white')
            
            # Add wireframe toggle button (smaller with overlay text)
            self.plotter.add_checkbox_button_widget(
                callback=self._toggle_wireframe_mode,
                value=self.wireframe_mode,
                position=(10, 90),
                size=30,  # Smaller button
                border_size=1,
                color_on='cyan',
                color_off='darkgray'
            )
            # Overlay text on button - larger font and better positioning
            self.plotter.add_text("🕸️", position=(18, 98), font_size=12, color='white')
            
            # Add simulation control buttons
            # Play/Pause button (smaller with overlay text)
            self.plotter.add_checkbox_button_widget(
                callback=self._toggle_simulation_pause,
                value=not self.simulation_paused,  # Inverted logic: True = Playing
                position=(10, 130),
                size=30,  # Smaller button
                border_size=1,
                color_on='green',
                color_off='orange'
            )
            # Overlay text on button - larger font and better positioning
            self.plotter.add_text("▶️", position=(18, 138), font_size=12, color='white')
            
            # Reset button (smaller with overlay text)
            try:
                self.plotter.add_checkbox_button_widget(
                    callback=self._reset_simulation,
                    value=False,  # Always false, acts as button
                    position=(50, 130),
                    size=30,  # Smaller button
                    border_size=1,
                    color_on='red',
                    color_off='darkred'
                )
                # Overlay text on button - larger font and better positioning
                self.plotter.add_text("🔄", position=(58, 138), font_size=12, color='white')
            except Exception as e:
                print(f"⚠️ Could not add reset button: {e}")
            
            # Add simulation time display
            self._setup_time_display()
            
            print("✅ Interactive controls setup complete (with simulation controls)")
            
        except Exception as e:
            print(f"⚠️ Failed to setup interactive controls: {e}")
            # Continue without interactive controls
    
    def _toggle_axis_display(self, value):
        """Toggle coordinate axis display"""
        self.axis_display_visible = value
        try:
            if value:
                # Show axis - add coordinate axes if not present
                SceneBuilder.add_coordinate_axes(self.plotter, self.pv)
                print("🎯 Axis display enabled")
            else:
                # Hide axis - remove coordinate axes
                try:
                    # Remove axes actors using correct names
                    self.plotter.remove_actor('coord_axis_x')
                    self.plotter.remove_actor('coord_axis_y')  
                    self.plotter.remove_actor('coord_axis_z')
                    print("🎯 Axis display disabled")
                except Exception as remove_error:
                    print(f"⚠️ Error removing axis actors: {remove_error}")
                    # Try alternative approach - clear all meshes with axis names
                    try:
                        renderer = self.plotter.renderer
                        actors_to_remove = []
                        for actor in renderer.GetActors():
                            if hasattr(actor, 'GetMapper') and actor.GetMapper():
                                input_data = actor.GetMapper().GetInput()
                                if hasattr(input_data, 'GetFieldData'):
                                    # Check if this is an axis actor
                                    actors_to_remove.append(actor)
                        # Remove collected axis actors
                        for actor in actors_to_remove[:3]:  # Remove up to 3 axis actors
                            renderer.RemoveActor(actor)
                        print("🎯 Axis display disabled (fallback method)")
                    except Exception as fallback_error:
                        print(f"⚠️ Fallback axis removal failed: {fallback_error}")
        except Exception as e:
            print(f"⚠️ Error toggling axis display: {e}")
    
    def _update_realtime_factor(self, value):
        """Update real-time factor"""
        old_value = self.realtime_factor
        self.realtime_factor = value
        print(f"⏱️ Visualizer real-time factor slider: {old_value:.2f}x → {value:.2f}x")
        
        # Update connected TimeManager if available
        if self._connected_time_manager:
            print(f"🔗 Updating connected TimeManager...")
            self._connected_time_manager.set_real_time_factor(value)
        # Fallback to connected SimulationManager (legacy)
        elif self._connected_simulation_manager:
            print(f"🔗 Updating connected SimulationManager...")
            self._connected_simulation_manager.set_realtime_factor(value)
        else:
            print(f"⚠️ No TimeManager or SimulationManager connected - change will not affect simulation speed")
    
    def _toggle_collision_display(self, value):
        """Toggle collision geometry display"""
        self.collision_display_visible = value
        try:
            if value:
                # Show collision geometries (would need URDF collision data)
                print("🚧 Collision display enabled")
                # Implementation would add collision meshes
            else:
                print("🚧 Collision display disabled")
                # Implementation would remove collision meshes
        except Exception as e:
            print(f"⚠️ Error toggling collision display: {e}")
    
    def _toggle_wireframe_mode(self, value):
        """Toggle wireframe rendering mode"""
        self.wireframe_mode = value
        try:
            if value:
                print("🕸️ Wireframe mode enabled")
                # Set all robot meshes to wireframe
                for robot_name, link_actors in self.link_actors.items():
                    for link_name, actor_info in link_actors.items():
                        actor = actor_info['actor']
                        if hasattr(actor, 'GetProperty'):
                            actor.GetProperty().SetRepresentationToWireframe()
            else:
                print("🎨 Surface mode enabled")
                # Set all robot meshes to surface
                for robot_name, link_actors in self.link_actors.items():
                    for link_name, actor_info in link_actors.items():
                        actor = actor_info['actor']
                        if hasattr(actor, 'GetProperty'):
                            actor.GetProperty().SetRepresentationToSurface()
        except Exception as e:
            print(f"⚠️ Error toggling wireframe mode: {e}")
    
    def _setup_time_display(self):
        """Setup simulation elapsed time display"""
        try:
            # Add time display in upper right corner
            self._time_text_actor = self.plotter.add_text(
                "Sim Time: 0.0s",
                position=(0.75, 0.95),  # Upper right corner in normalized coordinates
                font_size=12,
                color='white',
                viewport=True  # Use viewport coordinates
            )
            self._simulation_start_time = time.time()
            print("⏰ Time display setup complete")
        except Exception as e:
            print(f"⚠️ Failed to setup time display: {e}")
    
    def update_time_display(self):
        """Update the simulation elapsed time display using centralized time management"""
        if self._time_text_actor:
            try:
                # Priority 1: Get time from connected time manager
                if self._connected_time_manager:
                    stats = self._connected_time_manager.get_timing_stats()
                    time_text = f"Sim: {stats.sim_time:.1f}s | Real: {stats.real_time_elapsed:.1f}s | Speed: {stats.actual_speed}"
                # Priority 2: Get time from connected simulation manager (legacy)
                elif self._connected_simulation_manager:
                    sim_time = self._connected_simulation_manager.get_sim_time()
                    real_time = self._connected_simulation_manager.get_real_time()
                    time_step = self._connected_simulation_manager.get_time_step()
                    time_text = f"Sim: {sim_time:.1f}s | Real: {real_time:.1f}s | Step: {time_step:.3f}s"
                # Priority 3: Global time manager
                elif TIME_MANAGER_SUPPORT:
                    global_time_mgr = get_global_time_manager()
                    if global_time_mgr:
                        stats = global_time_mgr.get_timing_stats()
                        time_text = f"Sim: {stats.sim_time:.1f}s | Real: {stats.real_time_elapsed:.1f}s | Speed: {stats.actual_speed}"
                    else:
                        time_text = "Time: No TimeManager"
                else:
                    # Fallback to real time only
                    if self._simulation_start_time:
                        elapsed_time = time.time() - self._simulation_start_time
                        time_text = f"Real: {elapsed_time:.1f}s"
                    else:
                        time_text = "Time: N/A"
                
                # Update the text actor
                self._time_text_actor.SetInput(time_text)
                
            except Exception as e:
                print(f"⚠️ Error updating time display: {e}")
    
    def _toggle_simulation_pause(self, value):
        """Toggle simulation pause/resume"""
        # Inverted logic: value=True means playing, value=False means paused
        self.simulation_paused = not value
        
        try:
            if self.simulation_paused:
                print("⏸️ Simulation PAUSED")
                if self._connected_simulation_manager:
                    self._connected_simulation_manager.pause_simulation()
            else:
                print("▶️ Simulation RESUMED")
                if self._connected_simulation_manager:
                    self._connected_simulation_manager.resume_simulation()
        except Exception as e:
            print(f"⚠️ Error toggling simulation pause: {e}")
    
    def _reset_simulation(self, value):
        """Reset simulation to initial state"""
        try:
            print("🔄 Simulation RESET")
            if self._connected_simulation_manager:
                self._connected_simulation_manager.reset_simulation()
        except Exception as e:
            print(f"⚠️ Error resetting simulation: {e}")
        
        # Reset the time display when simulation resets
        self._reset_time_display()
    
    def _reset_time_display(self):
        """Reset the simulation time display"""
        try:
            self._simulation_start_time = time.time()
            if self._time_text_actor:
                if self._connected_time_manager:
                    stats = self._connected_time_manager.get_timing_stats()
                    self._time_text_actor.SetInput(f"Sim: 0.0s | Real: 0.0s | Speed: {stats.target_speed}")
                elif self._connected_simulation_manager:
                    time_step = self._connected_simulation_manager.get_time_step()
                    self._time_text_actor.SetInput(f"Sim: 0.0s | Real: 0.0s | Step: {time_step:.3f}s")
                else:
                    self._time_text_actor.SetInput("Sim: 0.0s | Real: 0.0s")
            print("⏰ Time display reset")
        except Exception as e:
            print(f"⚠️ Error resetting time display: {e}")
    
    def connect_time_manager(self, time_manager: TimeManager):
        """Connect to a TimeManager for centralized time access"""
        self._connected_time_manager = time_manager
        # Sync initial realtime factor
        self.realtime_factor = time_manager.get_real_time_factor()
        print(f"🔗 Connected to TimeManager for centralized time access")
    
    def connect_simulation_manager(self, simulation_manager):
        """Connect to a SimulationManager for real-time control synchronization (legacy support)"""
        self._connected_simulation_manager = simulation_manager
        
        # Also connect to its time manager if available
        if hasattr(simulation_manager, 'time_manager') and simulation_manager.time_manager:
            self.connect_time_manager(simulation_manager.time_manager)
        
        # Sync initial realtime factor
        if hasattr(simulation_manager, 'config') and hasattr(simulation_manager.config, 'real_time_factor'):
            self.realtime_factor = simulation_manager.config.real_time_factor
        print(f"🔗 Connected to SimulationManager for real-time factor control")
    
    def disconnect_time_manager(self):
        """Disconnect from TimeManager"""
        self._connected_time_manager = None
        print(f"🔗 Disconnected from TimeManager")
    
    def disconnect_simulation_manager(self):
        """Disconnect from SimulationManager"""
        self._connected_simulation_manager = None
        self.disconnect_time_manager()
        print(f"🔗 Disconnected from SimulationManager")
    
    def _create_mesh_from_urdf(self, urdf_path: str, package_path: Optional[str] = None):
        """
        Create robot mesh from URDF file - Integrated from RobotMeshFactory
        
        Args:
            urdf_path: Path to URDF file
            package_path: Optional path to ROS package directory
            
        Returns:
            Combined PyVista mesh or None
        """
        # Try URDF loader first (supports yourdfpy)
        if URDF_SUPPORT:
            try:
                print("Using URDF loader...")
                from core.urdf_loader import URDFLoader
                loader = URDFLoader(package_path)
                if loader.is_available() and loader.load_urdf(urdf_path):
                    
                    # Create individual meshes with colors
                    meshes = loader.create_pyvista_meshes(self.pv)
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
                        mesh = loader.get_combined_mesh(self.pv)
                        if mesh:
                            loader.print_info()
                            return mesh
                        else:
                            print("URDF loader: mesh creation failed")
                else:
                    print("URDF loader: loading failed")
            except Exception as e:
                print(f"URDF loader error: {e}")
        return None
    
    def load_robot(self, robot_name: str, robot_instance) -> bool:
        """
        Load a robot for visualization and control - Optimized version using robot instance data
        
        Args:
            robot_name: Unique name for this robot instance
            robot_instance: Robot object from robot.py (already loaded URDF)
            
        Returns:
            bool: Success status
        """
        if not self.available:
            print("❌ PyVista not available")
            return False
        
        print(f"🤖 Loading robot '{robot_name}' using existing robot data")
        
        try:
            # Check if robot has URDF data loaded
            if not hasattr(robot_instance, 'urdf_loader') or robot_instance.urdf_loader is None:
                print(f"❌ Robot instance has no URDF data loaded")
                return False
            
            # Store robot (reuse existing URDF loader from robot instance)
            self.robots[robot_name] = robot_instance
            self.urdf_loaders[robot_name] = robot_instance.urdf_loader
            
            # Create individual link meshes using robot's link/joint info
            self._create_robot_link_actors_from_robot(robot_name, robot_instance)
            
            # Create animation controller for this robot
            animation_controller = AnimationController(self.plotter, self.pv)
            self.animation_controllers[robot_name] = animation_controller
            
            print(f"✅ Robot '{robot_name}' loaded successfully (reused URDF data)")
            robot_instance.urdf_loader.print_info()
            return True
            
        except Exception as e:
            print(f"❌ Failed to load robot '{robot_name}': {e}")
            return False
    
    def _create_robot_link_actors_from_robot(self, robot_name: str, robot_instance):
        """Create individual link actors using robot instance data - Optimized approach"""
        link_actors = {}
        
        print(f"🎨 Creating individual link meshes for '{robot_name}' using robot data...")
        
        for link_name, link_obj in robot_instance.links.items():
            # Create mesh based on geometry type
            mesh = None
            
            if link_obj.geometry_type == "box":
                size = link_obj.geometry_params.get('size', [0.3, 0.3, 0.1])
                mesh = self.pv.Cube(x_length=size[0], y_length=size[1], z_length=size[2])
                
            elif link_obj.geometry_type == "cylinder":
                radius = link_obj.geometry_params.get('radius', 0.05)
                length = link_obj.geometry_params.get('length', 0.35)
                mesh = self.pv.Cylinder(radius=radius, height=length, direction=(0, 0, 1))
                
            elif link_obj.geometry_type == "sphere":
                radius = link_obj.geometry_params.get('radius', 0.08)
                mesh = self.pv.Sphere(radius=radius)
            
            if mesh is not None:
                # Apply visual origin transformation from URDF data
                urdf_loader = robot_instance.urdf_loader
                if urdf_loader and link_name in urdf_loader.links:
                    urdf_link_info = urdf_loader.links[link_name]
                    if hasattr(urdf_link_info, 'pose') and urdf_link_info.pose is not None:
                        transform_matrix = urdf_link_info.pose.to_transformation_matrix()
                        mesh.transform(transform_matrix, inplace=True)
                        pos = urdf_link_info.pose.position
                        print(f"    🔧 Applied visual origin to {link_name}: pos={pos}")
                
                # Add to scene with link color
                color = link_obj.color[:3] if len(link_obj.color) >= 3 else (0.6, 0.6, 0.6)
                actor = self.plotter.add_mesh(
                    mesh, 
                    color=color, 
                    opacity=1.0, 
                    name=f"{robot_name}_{link_name}"
                )
                link_actors[link_name] = {
                    'actor': actor,
                    'mesh': mesh.copy(),  # Store original mesh
                    'color': color,
                    'geometry_type': link_obj.geometry_type
                }
                print(f"  Added {link_name}: {link_obj.geometry_type} with color {color}")
        
        self.link_actors[robot_name] = link_actors
        print(f"✅ Created {len(link_actors)} individual links for '{robot_name}' using robot data")
    
    # _create_robot_link_actors has been removed - use _create_robot_link_actors_from_robot instead
    # This ensures all mesh creation uses Robot instance data directly (memo.txt requirement 37)
    
    def update_robot_visualization(self, robot_name: str, force_render: bool = True):
        """Update robot visualization based on current joint positions
        
        Args:
            robot_name: Name of robot to update
            force_render: Whether to force rendering updates (for batch mode optimization)
        """
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
            
            # Update time display only when force_render is True (performance optimization)
            if force_render:
                self.update_time_display()
            
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

