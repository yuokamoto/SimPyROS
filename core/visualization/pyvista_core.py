#!/usr/bin/env python3
"""
PyVista Core Handler

Shared PyVista functionality for standard and process-separated implementations.
Uses composition pattern to avoid complex inheritance hierarchies.
"""

import os
from typing import Tuple, Optional, Any
import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))


class PyVistaCoreHandler:
    """
    Core PyVista functionality handler for composition pattern
    
    Provides shared PyVista initialization, plotter creation, and scene setup
    functionality without complex inheritance. Used by both PyVistaVisualizer
    and ProcessURDFVisualizer through composition.
    """
    
    def __init__(self):
        """Initialize PyVista core handler"""
        self.pv = None  # PyVista module reference
        self.vtk = None  # VTK module reference
        self.plotter = None
        self.available = False
        self.display_available = False
    
    def initialize_pyvista(self, interactive: bool = True, force_off_screen: bool = False) -> bool:
        """
        Initialize PyVista with proper configuration - Unified implementation
        
        Args:
            interactive: Enable interactive mode if display available
            force_off_screen: Force off-screen mode regardless of display
            
        Returns:
            bool: Success status
        """
        try:
            # Configure PyVista environment
            if not interactive or force_off_screen:
                os.environ['PYVISTA_OFF_SCREEN'] = 'true'
                os.environ['PYVISTA_USE_PANEL'] = 'false'
                os.environ['VTK_SILENCE_GET_VOID_POINTER_WARNINGS'] = '1'
            else:
                os.environ['PYVISTA_OFF_SCREEN'] = 'false'
                os.environ['PYVISTA_USE_PANEL'] = 'false'
            
            # Check if display is available
            self.display_available = bool(os.environ.get('DISPLAY', ''))
            
            # Import PyVista and VTK
            import pyvista as pv
            import vtk
            
            self.pv = pv
            self.vtk = vtk
            
            # Configure VTK for headless operation if needed
            if not self.display_available or not interactive or force_off_screen:
                try:
                    vtk.vtkOpenGLRenderWindow.SetGlobalMaximumNumberOfMultiSamples(0)
                    pv.start_xvfb()  # Start virtual framebuffer if available
                except Exception:
                    pass  # Continue without Xvfb
                    
                # Force off-screen rendering
                pv.OFF_SCREEN = True
                pv.set_plot_theme('document')
            else:
                pv.OFF_SCREEN = False
                pv.set_plot_theme('document')
            
            self.available = True
            mode = "Interactive" if interactive and self.display_available and not force_off_screen else "Headless"
            print(f"PyVista initialized ({mode} mode)")
            
            return True
            
        except ImportError as e:
            self.available = False
            print(f"PyVista not available: {e}")
            return False
        except Exception as e:
            self.available = False
            print(f"PyVista initialization error: {e}")
            return False
    
    def create_plotter(self, window_size: Tuple[int, int] = (1200, 800), 
                      background: str = 'lightblue') -> bool:
        """
        Create PyVista plotter - Unified implementation
        
        Args:
            window_size: Window dimensions (width, height)
            background: Background color
            
        Returns:
            bool: Success status
        """
        if not self.available or not self.pv:
            return False
            
        try:
            off_screen = not self.display_available or self.pv.OFF_SCREEN
            self.plotter = self.pv.Plotter(
                off_screen=off_screen, 
                window_size=window_size
            )
            self.plotter.set_background(background)
            
            return True
            
        except Exception as e:
            print(f"Plotter creation error: {e}")
            self.available = False
            return False
    
    def setup_basic_scene(self, add_ground: bool = True, add_axes: bool = False, 
                         ground_size: float = 10.0, axes_length: float = 1.0) -> bool:
        """
        Setup basic scene elements - Unified implementation
        
        Args:
            add_ground: Add ground plane
            add_axes: Add coordinate axes
            ground_size: Ground plane size
            axes_length: Coordinate axes length
            
        Returns:
            bool: Success status
        """
        if not self.available or not self.plotter or not self.pv:
            return False
            
        success = True
        
        # Add ground plane
        if add_ground:
            try:
                ground = self.pv.Plane(
                    center=(0, 0, -0.2), 
                    direction=[0, 0, 1], 
                    i_size=ground_size, 
                    j_size=ground_size, 
                    i_resolution=20, 
                    j_resolution=20
                )
                self.plotter.add_mesh(ground, color='lightgray', opacity=0.6)
            except Exception as e:
                print(f"Error adding ground plane: {e}")
                success = False
        
        # Add coordinate axes
        if add_axes:
            try:
                x_axis = self.pv.Arrow(start=(0, 0, 0), direction=[1, 0, 0], scale=axes_length)
                y_axis = self.pv.Arrow(start=(0, 0, 0), direction=[0, 1, 0], scale=axes_length)
                z_axis = self.pv.Arrow(start=(0, 0, 0), direction=[0, 0, 1], scale=axes_length)
                
                self.plotter.add_mesh(x_axis, color='red', name='coord_axis_x')
                self.plotter.add_mesh(y_axis, color='green', name='coord_axis_y')
                self.plotter.add_mesh(z_axis, color='blue', name='coord_axis_z')
            except Exception as e:
                print(f"Error adding coordinate axes: {e}")
                success = False
        
        return success
    
    def setup_camera_controls(self) -> bool:
        """Setup enhanced camera controls for interactive navigation - Unified implementation"""
        if not self.plotter:
            return False
            
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
            
            print("⚡ PyVista interactive mode prepared for real-time simulation")
            print("📷 Enhanced camera controls configured")
            print("🖱️  Interactive camera controls enabled:")
            print("     Left Click+Drag: Rotate")
            print("     Right Click+Drag: Zoom")
            print("     Middle Click+Drag: Pan")
            print("     Mouse Wheel: Zoom In/Out")
            
            return True
            
        except Exception as e:
            print(f"⚠️ Camera setup error: {e}")
            return False
    
    def cleanup(self):
        """Cleanup PyVista resources"""
        if self.plotter:
            try:
                self.plotter.close()
            except Exception:
                pass  # Ignore cleanup errors
        self.plotter = None
        self.pv = None
        self.vtk = None