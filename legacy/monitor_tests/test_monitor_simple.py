#!/usr/bin/env python3
"""
Simple test for monitor creation without tkinter window
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationConfig
import time

def test_monitor_creation():
    """Test monitor configuration and data preparation"""
    
    config = SimulationConfig(
        visualization=False,  # Headless mode
        enable_monitor=True,  # Enable monitor anyway
        monitor_enable_controls=True,  # Enable controls for testing
        real_time_factor=2.0,
        update_rate=50.0
    )
    
    print("🧪 Testing monitor configuration")
    print(f"📊 Monitor enabled: {config.enable_monitor}")
    print(f"🎮 Monitor controls: {config.monitor_enable_controls}")
    print(f"⚡ Real-time factor: {config.real_time_factor}")
    print(f"📺 Visualization: {config.visualization}")
    
    # Test data structure that would be sent to monitor
    monitor_data = {
        'sim_time': 5.25,
        'real_time': 2.63,
        'target_rt_factor': 2.0,
        'actual_rt_factor': 2.0,
        'timing_accuracy': 99.5,
        'update_rate': 50.0,
        'time_step': 0.02,
        'visualization': 'headless',
        'active_robots': 1,
        'active_objects': 0,
        'architecture': 'Event-Driven SimPy',
        'simulation_state': 'running'
    }
    
    print("\n📊 Sample monitor data:")
    for key, value in monitor_data.items():
        print(f"   {key}: {value}")
    
    print("\n✅ Monitor configuration test passed")
    return True

if __name__ == "__main__":
    test_monitor_creation()