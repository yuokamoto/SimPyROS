#!/usr/bin/env python3
"""
Fixed Real-time Demo - Robust version with error handling
Handles both PyVista rendering and fallback modes
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np
import time
import math
import threading
import json
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose
import simpy

# Try to import PyVista with proper error handling
PYVISTA_AVAILABLE = False
try:
    # Set environment variables before importing
    os.environ['PYVISTA_OFF_SCREEN'] = 'true'
    os.environ['PYVISTA_USE_PANEL'] = 'false'
    
    import pyvista as pv
    pv.OFF_SCREEN = True
    pv.set_plot_theme('document')
    PYVISTA_AVAILABLE = True
    print("PyVista available - enabling advanced visualization")
except Exception as e:
    print(f"PyVista not available ({e}) - using data-only mode")


class RobustRealtimeVisualizer:
    """Robust real-time visualizer with multiple rendering backends"""
    
    def __init__(self, use_pyvista=True, fps_target=30):
        self.fps_target = fps_target
        self.frame_interval = 1.0 / fps_target
        
        # Rendering backend selection
        self.use_pyvista = use_pyvista and PYVISTA_AVAILABLE
        self.plotter = None
        
        if self.use_pyvista:
            try:
                self.plotter = pv.Plotter(window_size=(1200, 900), off_screen=True)
                self.plotter.set_background('white')
                self._setup_pyvista_scene()
                print("PyVista backend initialized successfully")
            except Exception as e:
                print(f"PyVista initialization failed: {e}")
                print("Falling back to data-only mode")
                self.use_pyvista = False
                self.plotter = None
        
        # State management
        self.objects = {}
        self.trajectories = {}
        self.running = False
        self.paused = False
        self.frame_count = 0
        self.screenshot_count = 0
        
        print(f"Visualizer initialized - Backend: {'PyVista' if self.use_pyvista else 'Data-only'}")
        
    def _setup_pyvista_scene(self):
        """Setup PyVista 3D scene"""
        if not self.plotter:
            return
            
        try:
            # Ground plane
            ground = pv.Plane(center=[0, 0, -0.1], direction=[0, 0, 1], 
                             i_size=10, j_size=10, i_resolution=20, j_resolution=20)
            self.plotter.add_mesh(ground, color='lightgray', opacity=0.3)
            
            # Coordinate axes
            axes_length = 1.5
            x_axis = pv.Arrow(start=[0, 0, 0], direction=[1, 0, 0], scale=axes_length)
            y_axis = pv.Arrow(start=[0, 0, 0], direction=[0, 1, 0], scale=axes_length)
            z_axis = pv.Arrow(start=[0, 0, 0], direction=[0, 0, 1], scale=axes_length)
            
            self.plotter.add_mesh(x_axis, color='red', opacity=0.8)
            self.plotter.add_mesh(y_axis, color='green', opacity=0.8)
            self.plotter.add_mesh(z_axis, color='blue', opacity=0.8)
            
            # Camera position
            self.plotter.camera_position = [(8, 8, 6), (0, 0, 1), (0, 0, 1)]
            
        except Exception as e:
            print(f"PyVista scene setup error: {e}")
            
    def _create_robot_mesh(self, robot_type):
        """Create robot mesh for PyVista rendering"""
        if not PYVISTA_AVAILABLE:
            return None
            
        try:
            if robot_type == 'racer':
                # Racing car
                body = pv.Box(bounds=[-0.4, 0.4, -0.15, 0.15, 0, 0.08])
                # Wheels
                wheel_positions = [[-0.3, -0.18, -0.02], [-0.3, 0.18, -0.02], 
                                 [0.3, -0.18, -0.02], [0.3, 0.18, -0.02]]
                wheels = []
                for pos in wheel_positions:
                    wheel = pv.Cylinder(center=pos, direction=[0, 1, 0], 
                                       radius=0.06, height=0.03, resolution=8)
                    wheels.append(wheel)
                
                result = body
                for wheel in wheels:
                    result = result + wheel
                return result
                
            elif robot_type == 'walker':
                # Bipedal walker
                torso = pv.Box(bounds=[-0.1, 0.1, -0.05, 0.05, 0, 0.25])
                head = pv.Sphere(center=[0, 0, 0.35], radius=0.08, phi_resolution=12, theta_resolution=12)
                
                # Legs
                left_leg = pv.Cylinder(center=[-0.05, 0, -0.15], direction=[0, 0, 1], 
                                      radius=0.025, height=0.3, resolution=8)
                right_leg = pv.Cylinder(center=[0.05, 0, -0.15], direction=[0, 0, 1], 
                                       radius=0.025, height=0.3, resolution=8)
                
                return torso + head + left_leg + right_leg
                
            elif robot_type == 'flyer':
                # Quadcopter
                center = pv.Sphere(center=[0, 0, 0], radius=0.05, phi_resolution=10, theta_resolution=10)
                
                # Propeller arms and props
                arms = []
                for angle in [0, 90, 180, 270]:
                    rad = math.radians(angle)
                    arm_end = [0.2 * math.cos(rad), 0.2 * math.sin(rad), 0]
                    
                    arm = pv.Cylinder(center=[arm_end[0]/2, arm_end[1]/2, 0], 
                                     direction=arm_end, radius=0.01, height=0.2, resolution=6)
                    prop = pv.Cylinder(center=arm_end, direction=[0, 0, 1], 
                                      radius=0.05, height=0.005, resolution=8)
                    arms.append(arm + prop)
                
                result = center
                for arm in arms:
                    result = result + arm
                return result
                
            else:
                return pv.Sphere(radius=0.2, phi_resolution=12, theta_resolution=12)
                
        except Exception as e:
            print(f"Mesh creation error: {e}")
            return pv.Sphere(radius=0.1)  # Fallback
            
    def add_robot(self, sim_obj, robot_type, color):
        """Add robot to visualization"""
        name = sim_obj.parameters.name
        self.objects[name] = {
            'sim_obj': sim_obj,
            'type': robot_type,
            'color': color,
            'mesh_actor': None
        }
        self.trajectories[name] = []
        
        # Create PyVista mesh if available
        if self.use_pyvista and self.plotter:
            try:
                mesh = self._create_robot_mesh(robot_type)
                if mesh:
                    actor = self.plotter.add_mesh(mesh, color=color, opacity=0.9, name=name)
                    self.objects[name]['mesh_actor'] = actor
                    
            except Exception as e:
                print(f"PyVista mesh addition error for {name}: {e}")
                
        print(f"Added {robot_type} robot: {name}")
        
    def update_robot(self, name, pose):
        """Update robot pose"""
        if name not in self.objects:
            return
            
        # Update simulation object
        self.objects[name]['sim_obj'].pose = pose
        
        # Add to trajectory
        pos = pose.position
        self.trajectories[name].append({
            'time': time.time(),
            'position': [pos[0], pos[1], pos[2]],
            'rotation': [pose.roll, pose.pitch, pose.yaw]
        })
        
        # Limit trajectory length
        if len(self.trajectories[name]) > 300:
            self.trajectories[name].pop(0)
            
        # Update PyVista mesh if available
        if self.use_pyvista and self.plotter:
            try:
                robot_data = self.objects[name]
                mesh = self._create_robot_mesh(robot_data['type'])
                
                if mesh:
                    # Transform mesh
                    transform_matrix = pose.to_transformation_matrix()
                    mesh.transform(transform_matrix, inplace=True)
                    
                    # Remove old actor
                    if robot_data['mesh_actor']:
                        self.plotter.remove_actor(robot_data['mesh_actor'])
                    
                    # Add new actor
                    actor = self.plotter.add_mesh(mesh, color=robot_data['color'], 
                                                opacity=0.9, name=name)
                    self.objects[name]['mesh_actor'] = actor
                    
            except Exception as e:
                # Silently continue on PyVista errors
                pass
                
    def save_screenshot(self, suffix=""):
        """Save PyVista screenshot if available"""
        if not self.use_pyvista or not self.plotter:
            return None
            
        try:
            os.makedirs("output", exist_ok=True)
            filename = f"output/realtime_render_{self.screenshot_count:04d}{suffix}.png"
            self.plotter.screenshot(filename)
            self.screenshot_count += 1
            return filename
        except Exception as e:
            print(f"Screenshot error: {e}")
            return None
            
    def save_data_snapshot(self, frame_num):
        """Save current state as JSON data"""
        os.makedirs("output", exist_ok=True)
        
        snapshot = {
            'frame': frame_num,
            'timestamp': time.time(),
            'backend': 'PyVista' if self.use_pyvista else 'Data-only',
            'robots': {}
        }
        
        for name, robot_data in self.objects.items():
            pose = robot_data['sim_obj'].pose
            snapshot['robots'][name] = {
                'type': robot_data['type'],
                'color': robot_data['color'],
                'position': [pose.x, pose.y, pose.z],
                'rotation': [pose.roll, pose.pitch, pose.yaw],
                'trajectory_length': len(self.trajectories[name])
            }
            
        filename = f"output/fixed_realtime_data_{frame_num:06d}.json"
        with open(filename, 'w') as f:
            json.dump(snapshot, f, indent=2)
            
        return filename
        
    def print_status(self, elapsed_time):
        """Print current status"""
        print(f"\\n=== t={elapsed_time:.1f}s Status ===")
        for name, robot_data in self.objects.items():
            pose = robot_data['sim_obj'].pose
            traj_len = len(self.trajectories[name])
            print(f"{name:10s}: pos=({pose.x:6.2f}, {pose.y:6.2f}, {pose.z:6.2f}) "
                  f"yaw={math.degrees(pose.yaw):6.1f}° trail={traj_len}")
                  
        fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        backend = "PyVista" if self.use_pyvista else "Data-only"
        print(f"Backend: {backend}, FPS: {fps:.1f}, Frames: {self.frame_count}")
        
    def run_demo(self, duration=20):
        """Run complete demonstration"""
        print(f"Starting {duration}s robust real-time demo...")
        
        # Create simulation environment
        env = simpy.Environment()
        
        # Create robots with different characteristics
        robots_config = [
            ("speedy", "racer", "red"),
            ("walker", "walker", "green"), 
            ("drone", "flyer", "blue")
        ]
        
        for name, robot_type, color in robots_config:
            sim_obj = SimulationObject(env, ObjectParameters(
                name=name, object_type=ObjectType.DYNAMIC,
                initial_pose=Pose(x=0, y=0, z=0.5), update_interval=0.05
            ))
            self.add_robot(sim_obj, robot_type, color)
            
        # Motion controller with realistic patterns
        def motion_controller():
            start_time = time.time()
            
            while self.running:
                if self.paused:
                    time.sleep(0.1)
                    continue
                    
                current_time = time.time() - start_time
                
                if current_time >= duration:
                    break
                    
                try:
                    # Speedy: Race track (oval)
                    track_t = current_time * 0.5  # Speed factor
                    track_angle = track_t * 2 * math.pi / 8  # 8 second lap
                    speedy_x = 3 * math.cos(track_angle)
                    speedy_y = 1.5 * math.sin(track_angle)
                    speedy_z = 0.1 + 0.03 * abs(math.sin(track_angle * 4))  # Road bumps
                    speedy_yaw = track_angle + math.pi/2  # Tangent to track
                    self.update_robot("speedy", Pose(x=speedy_x, y=speedy_y, z=speedy_z, yaw=speedy_yaw))
                    
                    # Walker: Figure-8 walking pattern
                    walk_t = current_time * 0.3
                    walker_x = 1.8 * math.sin(walk_t)
                    walker_y = 0.9 * math.sin(2 * walk_t)
                    walker_z = 0.3 + 0.08 * abs(math.sin(walk_t * 6))  # Walking bounce
                    walker_pitch = 0.15 * math.sin(walk_t * 4)  # Walking lean
                    self.update_robot("walker", Pose(x=walker_x, y=walker_y, z=walker_z, pitch=walker_pitch))
                    
                    # Drone: 3D patrol with altitude changes
                    patrol_t = current_time * 0.25
                    drone_x = 2 * math.cos(patrol_t)
                    drone_y = 2 * math.sin(patrol_t * 0.7)
                    drone_z = 2 + 0.8 * math.sin(patrol_t * 0.4)  # Altitude patrol
                    drone_roll = 0.3 * math.sin(patrol_t * 2)  # Banking in turns
                    drone_yaw = patrol_t * 0.5
                    self.update_robot("drone", Pose(x=drone_x, y=drone_y, z=drone_z, roll=drone_roll, yaw=drone_yaw))
                    
                    time.sleep(0.033)  # ~30 Hz update
                    
                except Exception as e:
                    print(f"Motion controller error: {e}")
                    time.sleep(0.1)
                    
        # Rendering and data export loop
        def render_export_loop():
            last_screenshot = 0
            last_data_export = 0 
            last_status = 0
            
            screenshot_interval = self.fps_target * 3  # Every 3 seconds
            data_export_interval = self.fps_target * 2  # Every 2 seconds  
            status_interval = self.fps_target * 5  # Every 5 seconds
            
            start_time = time.time()
            
            while self.running:
                if self.paused:
                    time.sleep(0.1)
                    continue
                    
                elapsed = time.time() - start_time
                
                # Save screenshot
                if (self.frame_count - last_screenshot >= screenshot_interval and 
                    self.use_pyvista):
                    screenshot_file = self.save_screenshot()
                    if screenshot_file:
                        print(f"t={elapsed:.1f}s: Screenshot saved - {screenshot_file}")
                    last_screenshot = self.frame_count
                    
                # Export data
                if self.frame_count - last_data_export >= data_export_interval:
                    data_file = self.save_data_snapshot(self.frame_count)
                    print(f"t={elapsed:.1f}s: Data saved - {data_file}")
                    last_data_export = self.frame_count
                    
                # Status update
                if self.frame_count - last_status >= status_interval:
                    self.print_status(elapsed)
                    last_status = self.frame_count
                    
                self.frame_count += 1
                time.sleep(self.frame_interval)
                
        # Run demo
        self.running = True
        
        motion_thread = threading.Thread(target=motion_controller)
        motion_thread.daemon = True
        motion_thread.start()
        
        render_thread = threading.Thread(target=render_export_loop)
        render_thread.daemon = True
        render_thread.start()
        
        # Save initial screenshot
        if self.use_pyvista:
            initial_screenshot = self.save_screenshot("_initial")
            if initial_screenshot:
                print(f"Initial screenshot: {initial_screenshot}")
        
        # Main demo loop
        demo_start = time.time()
        try:
            while time.time() - demo_start < duration:
                time.sleep(1)
                elapsed = time.time() - demo_start
                
                if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                    progress = (elapsed / duration) * 100
                    print(f"Demo: {progress:.0f}% complete ({elapsed:.0f}s/{duration}s)")
                    
        except KeyboardInterrupt:
            print("\\nDemo interrupted by user")
            
        # Cleanup
        self.running = False
        time.sleep(1)  # Let threads finish
        
        # Final status
        final_elapsed = time.time() - demo_start
        self.print_status(final_elapsed)
        
        # Save final screenshot
        if self.use_pyvista:
            final_screenshot = self.save_screenshot("_final")
            if final_screenshot:
                print(f"Final screenshot: {final_screenshot}")
        
        # Export complete trajectories
        os.makedirs("output", exist_ok=True)
        trajectory_file = "output/complete_trajectories.json"
        trajectory_data = {
            'demo_duration': final_elapsed,
            'total_frames': self.frame_count,
            'backend': 'PyVista' if self.use_pyvista else 'Data-only',
            'fps': self.frame_count / final_elapsed,
            'trajectories': self.trajectories
        }
        
        with open(trajectory_file, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
            
        print(f"\\n🎉 Demo completed successfully!")
        print(f"Duration: {final_elapsed:.1f}s")
        print(f"Frames: {self.frame_count}")
        print(f"Avg FPS: {self.frame_count / final_elapsed:.1f}")
        print(f"Backend: {'PyVista' if self.use_pyvista else 'Data-only'}")
        print(f"Screenshots: {self.screenshot_count}")
        print("\\nGenerated files in output/:")
        print("  - fixed_realtime_data_*.json")
        print("  - complete_trajectories.json")
        if self.use_pyvista:
            print("  - realtime_render_*.png")
            
        return self.frame_count
        

def main():
    """Main demo with error handling"""
    print("Robust Real-time Robot Simulation Demo")
    print("Handles PyVista and fallback modes gracefully")
    print("=" * 60)
    
    # Try PyVista first, fallback to data-only
    try:
        viz = RobustRealtimeVisualizer(use_pyvista=True, fps_target=25)
        frames = viz.run_demo(duration=15)  # 15 second demo
        
    except Exception as e:
        print(f"PyVista mode failed: {e}")
        print("Retrying with data-only mode...")
        
        viz = RobustRealtimeVisualizer(use_pyvista=False, fps_target=30)
        frames = viz.run_demo(duration=15)
        
    print(f"\\n✅ Demo successful! Processed {frames} frames")
    

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\\nDemo interrupted by user")
    except Exception as e:
        print(f"\\nDemo error: {e}")
        import traceback
        traceback.print_exc()