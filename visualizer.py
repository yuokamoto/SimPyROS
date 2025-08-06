import matplotlib
# Set non-interactive backend if no display available
import os
if 'DISPLAY' not in os.environ:
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from typing import List, Dict, Optional, Tuple, Callable
import threading
import time
import signal
import sys
from simulation_object import SimulationObject, Pose
import math


class Object3DVisualizer:
    """3D visualization for simulation objects with real-time animation"""
    
    def __init__(self, figure_size=(12, 9), grid_size=10.0):
        # Create 3D plot
        self.fig = plt.figure(figsize=figure_size)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Initialize plot settings
        self.grid_size = grid_size
        self._setup_plot()
        
        # Object tracking
        self.objects: Dict[str, SimulationObject] = {}
        self.object_colors: Dict[str, str] = {}
        self.object_markers: Dict[str, str] = {}
        self.object_sizes: Dict[str, int] = {}
        
        # Trajectory tracking
        self.trajectories: Dict[str, List[Tuple[float, float, float]]] = {}
        self.max_trajectory_length = 1000
        
        # Animation
        self.animation_running = False
        self.update_interval = 50  # milliseconds
        self.ani = None
        
        # Connection visualization
        self.show_connections = True
        
        # View angle tracking
        self._current_elev = 20
        self._current_azim = 45
        
        # Shutdown handling
        self._shutdown_requested = False
        self._setup_signal_handlers()
        
        # Setup mouse interaction handlers
        self._setup_mouse_handlers()
        
    def _setup_plot(self):
        """Setup 3D plot appearance"""
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('SimPyROS 3D Simulation Visualization')
        
        # Set equal aspect ratio and limits
        self.ax.set_xlim(-self.grid_size, self.grid_size)
        self.ax.set_ylim(-self.grid_size, self.grid_size)
        self.ax.set_zlim(0, self.grid_size)
        
        # Grid
        self.ax.grid(True, alpha=0.3)
        
        # Use current view or default viewing angle
        if hasattr(self, '_current_elev') and hasattr(self, '_current_azim'):
            self.ax.view_init(elev=self._current_elev, azim=self._current_azim)
        else:
            self.ax.view_init(elev=20, azim=45)
            self._current_elev = 20
            self._current_azim = 45
        
    def _setup_signal_handlers(self):
        """Setup signal handlers for graceful shutdown"""
        def signal_handler(signum, frame):
            print("\nShutdown requested (Ctrl+C). Closing visualization...")
            self._shutdown_requested = True
            self.cleanup()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        # Also handle SIGTERM on Unix systems
        if hasattr(signal, 'SIGTERM'):
            signal.signal(signal.SIGTERM, signal_handler)
    
    def _setup_mouse_handlers(self):
        """Setup mouse interaction handlers to preserve view changes"""
        def on_mouse_release(event):
            """Update stored view angles when mouse is released"""
            if event.inaxes == self.ax:
                self._current_elev = self.ax.elev
                self._current_azim = self.ax.azim
        
        # Connect mouse release event
        self.fig.canvas.mpl_connect('button_release_event', on_mouse_release)
            
    def cleanup(self):
        """Clean up resources"""
        try:
            if self.animation_running and self.ani:
                self.ani.pause()
                self.animation_running = False
            
            # Close matplotlib figure
            if self.fig:
                plt.close(self.fig)
                
        except Exception as e:
            print(f"Warning during cleanup: {e}")
        
    def add_object(self, obj: SimulationObject, color='blue', marker='o', size=100, show_trajectory=True):
        """Add a simulation object to visualization"""
        name = obj.parameters.name
        self.objects[name] = obj
        self.object_colors[name] = color
        self.object_markers[name] = marker
        self.object_sizes[name] = size
        
        if show_trajectory:
            self.trajectories[name] = []
            
    def remove_object(self, name: str):
        """Remove an object from visualization"""
        if name in self.objects:
            del self.objects[name]
            del self.object_colors[name] 
            del self.object_markers[name]
            del self.object_sizes[name]
            if name in self.trajectories:
                del self.trajectories[name]
                
    def _draw_coordinate_frame(self, pose: Pose, scale=0.5, alpha=0.8):
        """Draw coordinate frame (X-red, Y-green, Z-blue) at object pose"""
        # Get rotation matrix from pose
        rotation_matrix = pose.rotation.as_matrix()
        
        # Origin
        origin = pose.position
        
        # Coordinate axes
        x_axis = origin + rotation_matrix[:, 0] * scale  # Red - X axis
        y_axis = origin + rotation_matrix[:, 1] * scale  # Green - Y axis  
        z_axis = origin + rotation_matrix[:, 2] * scale  # Blue - Z axis
        
        # Draw axes
        self.ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], 
                     'r-', linewidth=2, alpha=alpha)
        self.ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], 
                     'g-', linewidth=2, alpha=alpha)
        self.ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], 
                     'b-', linewidth=2, alpha=alpha)
                     
    def _draw_connections(self):
        """Draw connections between objects"""
        if not self.show_connections:
            return
            
        for name, obj in self.objects.items():
            for connected_obj in obj.get_connected_objects():
                if connected_obj.parameters.name in self.objects:  # Only draw if both objects are visualized
                    # Draw line between connected objects
                    pos1 = obj.pose.position
                    pos2 = connected_obj.pose.position
                    
                    self.ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]],
                                'k--', linewidth=1, alpha=0.5)
                                
    def _update_trajectories(self):
        """Update trajectory data for all objects"""
        for name, obj in self.objects.items():
            if name in self.trajectories:
                pos = obj.pose.position
                self.trajectories[name].append((pos[0], pos[1], pos[2]))
                
                # Limit trajectory length
                if len(self.trajectories[name]) > self.max_trajectory_length:
                    self.trajectories[name] = self.trajectories[name][-self.max_trajectory_length:]
                    
    def _draw_trajectories(self):
        """Draw trajectory paths"""
        for name, trajectory in self.trajectories.items():
            if len(trajectory) > 1:
                trajectory = np.array(trajectory)
                self.ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
                            color=self.object_colors[name], alpha=0.3, linewidth=1)
                            
    def _update_plot(self, frame):
        """Update plot for animation"""
        # Check if shutdown was requested
        if self._shutdown_requested:
            return
            
        try:
            # Save current view before clearing
            self._current_elev = self.ax.elev
            self._current_azim = self.ax.azim
            
            self.ax.clear()
            self._setup_plot()
            
            # Update trajectories
            self._update_trajectories()
            
            # Draw trajectories
            self._draw_trajectories()
            
            # Draw objects
            for name, obj in self.objects.items():
                pose = obj.pose
                pos = pose.position
                
                # Draw object as point
                self.ax.scatter(pos[0], pos[1], pos[2], 
                              c=self.object_colors[name], 
                              marker=self.object_markers[name],
                              s=self.object_sizes[name],
                              alpha=0.8)
                
                # Draw coordinate frame
                self._draw_coordinate_frame(pose, scale=0.3)
                
                # Add label
                self.ax.text(pos[0], pos[1], pos[2] + 0.2, name, fontsize=8)
                
            # Draw connections
            self._draw_connections()
            
            # Update plot limits if objects are outside
            if self.objects:
                positions = np.array([obj.pose.position for obj in self.objects.values()])
                if positions.size > 0:
                    margin = 1.0
                    x_min, x_max = positions[:, 0].min() - margin, positions[:, 0].max() + margin
                    y_min, y_max = positions[:, 1].min() - margin, positions[:, 1].max() + margin
                    z_min, z_max = positions[:, 2].min() - margin, positions[:, 2].max() + margin
                    
                    # Expand limits if needed
                    current_xlim = self.ax.get_xlim()
                    current_ylim = self.ax.get_ylim() 
                    current_zlim = self.ax.get_zlim()
                    
                    self.ax.set_xlim(min(current_xlim[0], x_min), max(current_xlim[1], x_max))
                    self.ax.set_ylim(min(current_ylim[0], y_min), max(current_ylim[1], y_max))
                    self.ax.set_zlim(min(current_zlim[0], z_min), max(current_zlim[1], z_max))
                    
        except Exception as e:
            if not self._shutdown_requested:
                print(f"Error in plot update: {e}")
    
    def start_animation(self):
        """Start real-time animation"""
        if not self.animation_running:
            self.animation_running = True
            self.ani = animation.FuncAnimation(
                self.fig, self._update_plot, interval=self.update_interval,
                blit=False, cache_frame_data=False
            )
            
    def stop_animation(self):
        """Stop animation"""
        if self.animation_running and self.ani:
            self.ani.pause()
            self.animation_running = False
            
    def show(self, block=True):
        """Show the visualization window"""
        try:
            if 'DISPLAY' not in os.environ:
                print("No display detected. Saving visualization as image instead.")
                os.makedirs("output", exist_ok=True)
                self.save_frame("output/visualization_output.png")
                return
                
            print("Visualization window opened. Press Ctrl+C to close.")
            plt.show(block=block)
        except KeyboardInterrupt:
            print("\nVisualization interrupted by user")
            self.cleanup()
        except Exception as e:
            print(f"Error in visualization: {e}")
            self.cleanup()
        
    def save_frame(self, filename: str, dpi=300):
        """Save current frame as image"""
        self.fig.savefig(filename, dpi=dpi, bbox_inches='tight')
        
    def set_view(self, elev=20, azim=45):
        """Set viewing angle"""
        self.ax.view_init(elev=elev, azim=azim)
        self._current_elev = elev
        self._current_azim = azim
        
    def toggle_connections(self):
        """Toggle connection visualization"""
        self.show_connections = not self.show_connections
        
    def clear_trajectories(self):
        """Clear all trajectory data"""
        for name in self.trajectories:
            self.trajectories[name] = []
            
    def set_trajectory_length(self, length: int):
        """Set maximum trajectory length"""
        self.max_trajectory_length = length
        
    def get_statistics(self) -> Dict:
        """Get visualization statistics"""
        return {
            'num_objects': len(self.objects),
            'num_trajectories': len(self.trajectories),
            'total_trajectory_points': sum(len(traj) for traj in self.trajectories.values()),
            'animation_running': self.animation_running
        }


class MultiViewVisualizer:
    """Multiple view visualization (Top, Side, 3D)"""
    
    def __init__(self, figure_size=(15, 10)):
        self.fig = plt.figure(figsize=figure_size)
        
        # Create subplots
        self.ax_3d = self.fig.add_subplot(221, projection='3d')  # 3D view
        self.ax_xy = self.fig.add_subplot(222)  # Top view (XY)
        self.ax_xz = self.fig.add_subplot(223)  # Side view (XZ)
        self.ax_yz = self.fig.add_subplot(224)  # Side view (YZ)
        
        self.visualizer_3d = Object3DVisualizer()
        self.visualizer_3d.ax = self.ax_3d
        self.visualizer_3d._setup_plot()
        
        self._setup_2d_plots()
        
    def _setup_2d_plots(self):
        """Setup 2D plot views"""
        # Top view (XY)
        self.ax_xy.set_xlabel('X (m)')
        self.ax_xy.set_ylabel('Y (m)')
        self.ax_xy.set_title('Top View (XY)')
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')
        
        # Side view (XZ)
        self.ax_xz.set_xlabel('X (m)')
        self.ax_xz.set_ylabel('Z (m)')
        self.ax_xz.set_title('Side View (XZ)')
        self.ax_xz.grid(True, alpha=0.3)
        self.ax_xz.set_aspect('equal')
        
        # Side view (YZ)
        self.ax_yz.set_xlabel('Y (m)')
        self.ax_yz.set_ylabel('Z (m)')
        self.ax_yz.set_title('Side View (YZ)')
        self.ax_yz.grid(True, alpha=0.3)
        self.ax_yz.set_aspect('equal')
        
    def add_object(self, obj: SimulationObject, **kwargs):
        """Add object to all views"""
        self.visualizer_3d.add_object(obj, **kwargs)
        
    def start_animation(self):
        """Start animation for all views"""
        self.visualizer_3d.start_animation()
        
    def show(self, block=True):
        """Show multi-view visualization"""
        plt.tight_layout()
        plt.show(block=block)