#!/usr/bin/env python3
"""
Minimal test for X11 RENDER error in monitor window
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import time
from core.simulation_monitor import SimulationMonitor

def test_monitor_x11():
    """Test if the simplified monitor still causes X11 errors"""
    
    print("🧪 Testing simplified monitor for X11 RENDER errors...")
    print("📊 Creating monitor with basic tk widgets (no ttk)")
    
    monitor = SimulationMonitor("X11 Test Monitor")
    
    try:
        # Start the monitor
        monitor.start(enable_controls=True)
        
        # Give it time to create the window
        time.sleep(2.0)
        
        # Update with some test data
        test_data = {
            'sim_time': 1.5,
            'real_time': 0.75,
            'target_rt_factor': 2.0,
            'actual_rt_factor': 2.0,
            'timing_accuracy': 99.8,
            'update_rate': 30.0,
            'time_step': 0.033,
            'visualization': 'test',
            'active_robots': 1,
            'active_objects': 0,
            'architecture': 'Test Mode',
            'simulation_state': 'running'
        }
        
        print("📊 Updating monitor with test data...")
        monitor.update_data(test_data)
        
        # Let it run for a few seconds
        time.sleep(3.0)
        
        print("✅ Monitor window test completed without X11 errors")
        
    except Exception as e:
        print(f"❌ Monitor test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("🛑 Stopping monitor...")
        monitor.stop()
        time.sleep(1.0)

if __name__ == "__main__":
    test_monitor_x11()