#!/usr/bin/env python3
"""
Test script for SimulationMonitor functionality
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
import time

def test_monitor_with_headless():
    """Test monitor functionality with headless simulation"""
    
    config = SimulationConfig(
        visualization=False,  # Headless mode
        enable_monitor=True,  # Enable monitor anyway
        monitor_enable_controls=True,  # Enable controls for testing
        real_time_factor=2.0,
        update_rate=50.0
    )
    
    print("🧪 Testing simulation monitor with headless mode")
    print("📊 Monitor window should appear even without visualization")
    print("🔄 Running 10 second simulation with monitor")
    
    # Create simulation manager
    sim = SimulationManager(config)
    
    # Add a robot for demonstration
    robot = sim.add_robot_from_urdf("test_robot", "examples/robots/articulated_arm_robot.urdf")
    
    if robot:
        # Set a simple control callback
        def simple_control(dt, sim_time):
            # Simple joint oscillation
            joint_positions = {
                'base_to_shoulder': 0.5 * sin(sim_time),
                'shoulder_to_elbow': 0.3 * sin(sim_time * 1.5),
                'elbow_to_wrist': 0.2 * sin(sim_time * 2.0),
                'wrist_to_end': 0.1 * sin(sim_time * 3.0)
            }
            robot.set_joint_positions(joint_positions)
        
        from math import sin
        sim.set_robot_control_callback("test_robot", simple_control, frequency=20.0)
    
    # Run simulation
    try:
        sim.run(duration=10.0)
    except KeyboardInterrupt:
        print("🛑 User interrupted simulation")
    
    print("✅ Monitor test completed")

if __name__ == "__main__":
    test_monitor_with_headless()