#!/usr/bin/env python3
"""
Test script for memo.txt item 9 improvements:
1. Mesh transparency fix (opacity=1.0)
2. Simulation elapsed time display
3. Smaller buttons with overlaid text
"""
import time
import math
from core.simulation_manager import SimulationManager

def main():
    print("🎯 Testing Memo.txt Item 9 Improvements")
    print("=" * 45)
    print("Expected improvements:")
    print("✅ 1. Robot meshes should be opaque (not semi-transparent)")
    print("✅ 2. Simulation elapsed time should display in upper right")
    print("✅ 3. UI buttons should be smaller with emoji text overlaid")
    print()
    
    # Create simulation
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf(
        'test_robot', 
        'examples/robots/articulated_arm_robot.urdf'
    )

    def test_control(dt):
        """Simple joint motion for testing"""
        t = time.time()
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        for i, joint_name in enumerate(joint_names):
            position = 0.4 * math.sin(t * 0.5 + i * math.pi / 4)
            sim.set_robot_joint_position('test_robot', joint_name, position)

    sim.set_robot_control_callback('test_robot', test_control, frequency=20.0)
    
    print("🚀 Starting test simulation...")
    print("🎮 Test the following UI improvements:")
    print("  🎯 Axis toggle button (smaller with emoji)")
    print("  🚧 Collision toggle button (smaller with emoji)")
    print("  🕸️ Wireframe toggle button (smaller with emoji)")
    print("  ▶️ Play/Pause button (smaller with emoji)")
    print("  🔄 Reset button (smaller with emoji)")
    print("  ⏰ Time display in upper right corner")
    print("  🎨 Robot should be opaque, not transparent")
    print()
    print("Close window or press Ctrl+C when finished testing.")
    
    # Run for 30 seconds to test the improvements
    sim.run(duration=30.0)
    print("✅ Test completed")

if __name__ == "__main__":
    main()