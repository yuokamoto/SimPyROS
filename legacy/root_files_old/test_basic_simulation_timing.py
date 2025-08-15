#!/usr/bin/env python3
"""
Test basic_simulation timing accuracy in headless mode
"""

import sys
import os
import time
import math

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose

def test_basic_simulation_timing():
    """Test basic simulation timing accuracy"""
    
    print("🧪 Basic Simulation Timing Test")
    print("Testing if basic simulation achieves real-time factor 1.0")
    print("=" * 60)
    
    # Create configuration matching basic_simulation.py (but headless for testing)
    config = SimulationConfig(
        real_time_factor=1.0,  # Ensure 1:1 real-time synchronization
        visualization=False  # Headless for accurate timing measurement
    )
    
    sim = SimulationManager(config)
    
    try:
        # Add robot like basic_simulation.py
        robot = sim.add_robot_from_urdf(
            name="my_robot",
            urdf_path="examples/robots/articulated_arm_robot.urdf",
            unified_process=True
        )
        
        frame_count = 0
        
        def my_control(dt: float):
            """Simple sinusoidal joint motion (matching basic_simulation.py)"""
            nonlocal frame_count
            frame_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
            joint_names = [name for name in robot.get_joint_names() 
                          if robot.joints[name].joint_type.value != 'fixed']
            
            for i, joint_name in enumerate(joint_names):
                position = 0.5 * math.sin(t + i * math.pi / 3)
                sim.set_robot_joint_position("my_robot", joint_name, position)
        
        sim.set_robot_control_callback("my_robot", my_control, frequency=20.0)
        
        # Record timing
        wall_start_time = time.time()
        duration = 3.0
        
        print(f"🚀 Running simulation for {duration}s...")
        print(f"Expected wall time: {duration}s (real-time factor 1.0)")
        
        sim.run(duration=duration)
        
        # Calculate results
        wall_elapsed_time = time.time() - wall_start_time
        sim_time = sim.get_sim_time()
        actual_factor = sim_time / wall_elapsed_time if wall_elapsed_time > 0 else 0
        timing_error = abs(wall_elapsed_time - duration) / duration * 100
        
        print(f"\n📊 Timing Results:")
        print(f"   Simulation time: {sim_time:.3f}s")
        print(f"   Wall clock time: {wall_elapsed_time:.3f}s")
        print(f"   Target wall time: {duration:.3f}s")
        print(f"   Actual real-time factor: {actual_factor:.3f}x")
        print(f"   Target real-time factor: 1.000x")
        print(f"   Timing error: {timing_error:.1f}%")
        print(f"   Control callbacks: {frame_count}")
        
        # Assessment
        if timing_error < 5.0:
            print(f"   ✅ GOOD: Timing accuracy within 5%")
        elif timing_error < 10.0:
            print(f"   ⚠️ FAIR: Timing accuracy within 10%")
        else:
            print(f"   ❌ POOR: Timing accuracy >10%")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            sim.shutdown()
        except:
            pass

if __name__ == "__main__":
    test_basic_simulation_timing()