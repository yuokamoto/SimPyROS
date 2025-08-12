#!/usr/bin/env python3
"""
Test UI improvements: button labels and real-time factor sync
"""
import time
import math
from core.simulation_manager import SimulationManager

def main():
    print("🎨 Testing UI Improvements")
    print("=" * 40)
    
    # Create simulation
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf('test_robot', 'examples/robots/articulated_arm_robot.urdf')

    def test_control(dt):
        t = time.time()
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        for i, joint_name in enumerate(joint_names):
            position = 0.6 * math.sin(t * 0.8 + i * math.pi / 4)
            sim.set_robot_joint_position('test_robot', joint_name, position)

    sim.set_robot_control_callback('test_robot', test_control, frequency=30.0)
    
    print("🎮 UI Improvements to Test:")
    print("✅ Button Labels:")
    print("  - 🎯 Axes: Toggle coordinate axes")
    print("  - 🚧 Collision: Toggle collision geometry")  
    print("  - 🕸️ Wire: Toggle wireframe mode")
    print("  - ▶️ Play/Pause: Pause/resume simulation")
    print("  - 🔄 Reset: Reset robot position")
    print("")
    print("✅ Real-time Factor:")
    print("  - Slider should show live updates in console")
    print("  - Changes should immediately affect robot motion speed")
    print("  - Try values from 0.1x (slow) to 5.0x (fast)")
    print("")
    print("🚀 Starting enhanced UI test...")
    print("Close window to exit.")
    
    # Run simulation
    sim.run(duration=60.0)  # 1 minute test
    print("✅ UI improvements test completed")

if __name__ == "__main__":
    main()