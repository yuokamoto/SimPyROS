#!/usr/bin/env python3
"""
Simple robot movement test to debug the callback issue
"""

import sys
import os
import math
import time

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig


def test_simple_robot():
    """Minimal test of robot movement"""
    print("🔍 Testing simple robot movement...")
    
    # Create simple config
    config = SimulationConfig(
        real_time_factor=1.0,
        visualization=True,
        enable_frequency_grouping=False,  # Disable complex features
        update_rate=50.0
    )
    
    sim = SimulationManager(config)
    
    try:
        # Add robot
        print("📦 Adding robot...")
        robot = sim.add_robot_from_urdf(
            name="test_robot",
            urdf_path="examples/robots/articulated_arm_robot.urdf",
            joint_update_rate=50.0,  # Match callback frequency
            unified_process=False  # Use old approach
        )
        
        print(f"✅ Robot added successfully")
        print(f"📋 Joint names: {robot.get_joint_names()}")
        
        # Simple callback counter
        callback_count = 0
        
        def simple_control(dt: float):
            nonlocal callback_count
            callback_count += 1
            
            # Print every callback for first 20, then every 50
            if callback_count <= 20 or callback_count % 50 == 0:
                t = sim.get_sim_time()
                print(f"🔄 Callback #{callback_count}: t={t:.2f}s, dt={dt:.4f}s")
                
                # Set simple joint positions
                joint_names = robot.get_joint_names()
                for i, joint_name in enumerate(joint_names[:2]):  # Only first 2 joints
                    position = 0.5 * math.sin(t + i)
                    print(f"   Setting {joint_name} = {position:.3f}")
                    sim.set_robot_joint_position("test_robot", joint_name, position)
        
        print("🎮 Setting control callback...")
        sim.set_robot_control_callback("test_robot", simple_control, frequency=50.0)
        
        print("🚀 Starting 3-second test...")
        sim.run(duration=3.0, auto_close=True)
        
        print(f"✅ Test completed. Total callbacks: {callback_count}")
        
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
    test_simple_robot()