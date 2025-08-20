#!/usr/bin/env python3
"""
Test script for thread-safe monitor implementation
"""

import sys
import os
import time

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig


def test_safe_monitor():
    print("🧪 Testing thread-safe monitor implementation...")
    
    # Create simulation with process-separated monitor
    config = SimulationConfig(
        visualization=True,
        visualization_backend='process_separated_pyvista',
        real_time_factor=2.0,
        enable_monitor=True  # Enable monitor
    )
    
    print("🚀 Creating SimulationManager with monitor...")
    sim = SimulationManager(config)
    
    try:
        print("🤖 Adding robot...")
        robot = sim.add_robot_from_urdf(
            name="test_robot",
            urdf_path="examples/robots/articulated_arm_robot.urdf"
        )
        
        def simple_control(dt):
            pass  # No control needed
        
        sim.set_robot_control_callback("test_robot", simple_control)
        
        print("⏱️ Running simulation for 5 seconds...")
        sim.run(duration=5.0, auto_close=True)
        
    except Exception as e:
        print(f"❌ Error during simulation: {e}")
        
    finally:
        print("🛑 Shutting down simulation...")
        sim.shutdown()
        print("✅ Shutdown complete")
        
        # Additional wait for cleanup
        time.sleep(1.0)


if __name__ == "__main__":
    test_safe_monitor()