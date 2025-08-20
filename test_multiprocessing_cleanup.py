#!/usr/bin/env python3
"""
Multiprocessing cleanup test script

Tests that resource_tracker processes are properly cleaned up
"""

import sys
import os
import time
import subprocess

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig


def get_resource_tracker_processes():
    """Get all resource_tracker processes using ps command"""
    try:
        # Use ps to find resource_tracker processes
        result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
        trackers = []
        for line in result.stdout.split('\n'):
            if 'resource_tracker' in line and 'python' in line:
                parts = line.split()
                if len(parts) > 1:
                    try:
                        pid = int(parts[1])
                        trackers.append(pid)
                    except ValueError:
                        continue
        return trackers
    except Exception:
        return []


def main():
    print("🧪 Testing multiprocessing cleanup...")
    
    # Check initial resource tracker processes
    initial_trackers = get_resource_tracker_processes()
    print(f"📊 Initial resource_tracker processes: {len(initial_trackers)} - {initial_trackers}")
    
    # Create simulation with process-separated visualization
    config = SimulationConfig(
        visualization=True,
        visualization_backend='process_separated_pyvista',
        real_time_factor=2.0,
        enable_monitor=False  # Disable monitor to avoid X11 issues
    )
    
    print("🚀 Creating SimulationManager...")
    sim = SimulationManager(config)
    
    try:
        print("🤖 Adding robot...")
        robot = sim.add_robot_from_urdf(
            name="test_robot",
            urdf_path="examples/robots/articulated_arm_robot.urdf"
        )
        
        def simple_control(dt):
            pass  # No control needed for this test
        
        sim.set_robot_control_callback("test_robot", simple_control)
        
        # Check resource tracker processes after initialization
        mid_trackers = get_resource_tracker_processes()
        print(f"📊 Resource_tracker processes after init: {len(mid_trackers)} - {mid_trackers}")
        
        print("⏱️ Running simulation for 3 seconds...")
        sim.run(duration=3.0, auto_close=True)
        
    except Exception as e:
        print(f"❌ Error during simulation: {e}")
        
    finally:
        print("🛑 Shutting down simulation...")
        sim.shutdown()
        
        # Wait a moment for cleanup
        time.sleep(2.0)
        
        # Check final resource tracker processes
        final_trackers = get_resource_tracker_processes()
        print(f"📊 Final resource_tracker processes: {len(final_trackers)} - {final_trackers}")
        
        # Compare results
        if len(final_trackers) <= len(initial_trackers):
            print("✅ SUCCESS: No orphaned resource_tracker processes detected")
        else:
            orphaned = set(final_trackers) - set(initial_trackers)
            print(f"⚠️ WARNING: Orphaned resource_tracker processes detected: {orphaned}")
            
            # Force cleanup attempt
            print("🧹 Attempting force cleanup...")
            from core.multiprocessing_cleanup import force_cleanup_resource_tracker
            force_cleanup_resource_tracker()
            
            time.sleep(1.0)
            post_cleanup_trackers = get_resource_tracker_processes()
            print(f"📊 Post-cleanup resource_tracker processes: {len(post_cleanup_trackers)} - {post_cleanup_trackers}")


if __name__ == "__main__":
    main()