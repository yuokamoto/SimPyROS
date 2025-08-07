#!/usr/bin/env python3
"""
Simple Real-time Demo - Works in all environments
Demonstrates real-time robot simulation with periodic data export
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import time
import math
import threading
import json
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose
import simpy


class SimpleRealtimeVisualizer:
    """Lightweight real-time visualizer that works everywhere"""
    
    def __init__(self, fps_target=30):
        self.fps_target = fps_target
        self.frame_interval = 1.0 / fps_target
        
        # State
        self.objects = {}
        self.trajectories = {}
        self.running = False
        self.frame_count = 0
        
        print(f"Simple real-time visualizer initialized (Target: {fps_target} FPS)")
        
    def add_robot(self, sim_obj, robot_type='simple', color='blue'):
        """Add robot to tracking"""
        name = sim_obj.parameters.name
        self.objects[name] = {
            'sim_obj': sim_obj,
            'type': robot_type,
            'color': color
        }
        self.trajectories[name] = []
        print(f"Added {robot_type} robot: {name}")
        
    def update_robot(self, name, pose):
        """Update robot state"""
        if name in self.objects:
            # Update the robot's actual pose
            self.objects[name]['sim_obj'].pose = pose
            
        if name in self.trajectories:
            pos = pose.position
            self.trajectories[name].append({
                'time': time.time(),
                'position': [pos[0], pos[1], pos[2]],
                'rotation': [pose.roll, pose.pitch, pose.yaw]
            })
            
            # Limit trajectory length
            if len(self.trajectories[name]) > 500:
                self.trajectories[name].pop(0)
                
    def save_data_snapshot(self, frame_num):
        """Save current state as data"""
        os.makedirs("output", exist_ok=True)
        
        snapshot = {
            'frame': frame_num,
            'timestamp': time.time(),
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
            
        filename = f"output/realtime_data_{frame_num:06d}.json"
        with open(filename, 'w') as f:
            json.dump(snapshot, f, indent=2)
            
        return filename
        
    def export_full_trajectories(self):
        """Export complete trajectory data"""
        os.makedirs("output", exist_ok=True)
        
        trajectory_data = {
            'export_time': time.time(),
            'total_frames': self.frame_count,
            'fps_target': self.fps_target,
            'trajectories': self.trajectories
        }
        
        filename = "output/realtime_trajectories.json"
        with open(filename, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
            
        print(f"Full trajectories exported: {filename}")
        
    def print_status(self, elapsed_time):
        """Print current status"""
        print(f"\\n=== t={elapsed_time:.1f}s Status ===")
        for name, robot_data in self.objects.items():
            pose = robot_data['sim_obj'].pose
            traj_len = len(self.trajectories[name])
            print(f"{name:12s}: pos=({pose.x:6.2f}, {pose.y:6.2f}, {pose.z:6.2f}) "
                  f"rot=({math.degrees(pose.yaw):6.1f}°) trail={traj_len}")
                  
        fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        print(f"Performance: {fps:.1f} FPS, {self.frame_count} frames")
        
    def run_demo(self, duration=30):
        """Run real-time demonstration"""
        print(f"Starting {duration}s real-time demo...")
        
        # Create simulation
        env = simpy.Environment()
        
        # Create diverse robots with different motion patterns
        robots_config = [
            ("racer", "racing", "red", "oval_track"),
            ("explorer", "walking", "green", "random_walk"),
            ("guardian", "flying", "blue", "patrol_pattern")
        ]
        
        for name, robot_type, color, motion in robots_config:
            sim_obj = SimulationObject(env, ObjectParameters(
                name=name, object_type=ObjectType.DYNAMIC,
                initial_pose=Pose(x=0, y=0, z=0.5), update_interval=0.05
            ))
            self.add_robot(sim_obj, robot_type, color)
            
        # Motion patterns
        def motion_controller():
            start_time = time.time()
            
            while self.running:
                current_time = time.time() - start_time
                
                if current_time >= duration:
                    break
                    
                try:
                    # Racer: Oval track
                    track_progress = (current_time * 0.4) % (2 * math.pi)
                    racer_x = 3 * math.cos(track_progress)
                    racer_y = 1.5 * math.sin(track_progress)
                    racer_z = 0.1 + 0.05 * abs(math.sin(track_progress * 4))
                    racer_yaw = track_progress + math.pi/2
                    racer_pose = Pose(x=racer_x, y=racer_y, z=racer_z, yaw=racer_yaw)
                    self.update_robot("racer", racer_pose)
                    
                    # Explorer: Random walk with boundaries
                    walk_t = current_time * 0.3
                    explorer_x = 2 * math.sin(walk_t) + 0.5 * math.sin(walk_t * 3)
                    explorer_y = 2 * math.cos(walk_t * 0.7) + 0.3 * math.cos(walk_t * 5)
                    explorer_z = 0.3 + 0.1 * abs(math.sin(walk_t * 2))  # Walking bounce
                    explorer_pitch = 0.1 * math.sin(walk_t * 4)  # Walking motion
                    explorer_pose = Pose(x=explorer_x, y=explorer_y, z=explorer_z, pitch=explorer_pitch)
                    self.update_robot("explorer", explorer_pose)
                    
                    # Guardian: Patrol pattern with altitude changes
                    patrol_t = current_time * 0.2
                    guardian_x = 1.5 * math.sin(patrol_t * 2)
                    guardian_y = 1.5 * math.cos(patrol_t)
                    guardian_z = 2 + 0.5 * math.sin(patrol_t * 0.5)  # Altitude variation
                    guardian_roll = 0.2 * math.sin(patrol_t * 3)  # Banking
                    guardian_pose = Pose(x=guardian_x, y=guardian_y, z=guardian_z, roll=guardian_roll)
                    self.update_robot("guardian", guardian_pose)
                    
                    time.sleep(0.03)  # ~33 Hz
                    
                except Exception as e:
                    print(f"Motion controller error: {e}")
                    time.sleep(0.1)
                    
            print("Motion controller finished")
            
        # Data export loop
        def data_export_loop():
            last_export = 0
            last_status = 0
            export_interval = self.fps_target * 2  # Every 2 seconds
            status_interval = self.fps_target * 5  # Every 5 seconds
            
            start_time = time.time()
            
            while self.running:
                current_time = time.time()
                elapsed = current_time - start_time
                
                # Export data snapshots
                if self.frame_count - last_export >= export_interval:
                    filename = self.save_data_snapshot(self.frame_count)
                    print(f"t={elapsed:.1f}s: Data snapshot saved - {filename}")
                    last_export = self.frame_count
                    
                # Print status
                if self.frame_count - last_status >= status_interval:
                    self.print_status(elapsed)
                    last_status = self.frame_count
                    
                self.frame_count += 1
                time.sleep(self.frame_interval)
                
            print("Data export loop finished")
            
        # Start demo
        self.running = True
        
        # Start threads
        motion_thread = threading.Thread(target=motion_controller)
        motion_thread.daemon = True
        motion_thread.start()
        
        data_thread = threading.Thread(target=data_export_loop)
        data_thread.daemon = True
        data_thread.start()
        
        print("Real-time demo running...")
        print("- 3 robots with different motion patterns")
        print("- Periodic data export to output/")
        print("- Press Ctrl+C to stop early")
        
        # Main demo loop
        start_demo_time = time.time()
        try:
            while time.time() - start_demo_time < duration:
                time.sleep(1)
                elapsed = time.time() - start_demo_time
                
                # Progress updates
                if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                    remaining = duration - elapsed
                    progress = (elapsed / duration) * 100
                    print(f"Demo progress: {progress:.0f}% ({elapsed:.0f}s/{duration}s)")
                    
        except KeyboardInterrupt:
            print("\\nDemo interrupted by user")
            
        # Cleanup
        self.running = False
        
        # Wait for threads
        time.sleep(1)
        
        # Final status and export
        final_elapsed = time.time() - start_demo_time
        self.print_status(final_elapsed)
        self.export_full_trajectories()
        
        # Summary
        print(f"\\n🎉 Demo completed!")
        print(f"Duration: {final_elapsed:.1f}s")
        print(f"Frames processed: {self.frame_count}")
        print(f"Average FPS: {self.frame_count / final_elapsed:.1f}")
        print("\\n📁 Generated files:")
        print("  - output/realtime_data_*.json (periodic snapshots)")
        print("  - output/realtime_trajectories.json (full trajectory data)")
        
        return self.frame_count


def quick_demo():
    """Quick 10-second demo"""
    print("=== Quick Demo (10s) ===")
    viz = SimpleRealtimeVisualizer(fps_target=20)
    return viz.run_demo(duration=10)
    

def full_demo():
    """Full 30-second demo"""
    print("=== Full Demo (30s) ===")
    viz = SimpleRealtimeVisualizer(fps_target=30)
    return viz.run_demo(duration=30)


def main():
    """Main function with demo selection"""
    print("Simple Real-time Robot Simulation Demo")
    print("Works in any environment - no graphics dependencies!")
    print("=" * 60)
    
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "full":
        frames = full_demo()
    else:
        frames = quick_demo()
        
    print(f"\\n✅ Success! Processed {frames} frames in real-time")
    print("Check output/ folder for generated data files")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\\nDemo interrupted by user")
    except Exception as e:
        print(f"\\nDemo error: {e}")
        import traceback
        traceback.print_exc()