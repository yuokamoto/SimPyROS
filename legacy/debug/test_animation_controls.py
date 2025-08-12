#!/usr/bin/env python3
"""
Test animation and simulation controls
"""
import time
import math
from core.simulation_manager import SimulationManager

def main():
    print("🎬 Testing Animation and Simulation Controls")
    
    # Create simulation with visualization
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf('test_robot', 'examples/robots/articulated_arm_robot.urdf')

    control_count = 0
    def animated_control(dt):
        nonlocal control_count
        control_count += 1
        t = time.time()
        
        # Get movable joints
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        print(f"Animation frame #{control_count} at t={t:.2f}")
        
        # Sinusoidal motion for each joint
        for i, joint_name in enumerate(joint_names):
            position = 0.8 * math.sin(t * 0.5 + i * math.pi / 2)  # Slower, larger motion
            sim.set_robot_joint_position('test_robot', joint_name, position)

    # Set control callback for smooth animation
    sim.set_robot_control_callback('test_robot', animated_control, frequency=30.0)
    
    print("🚀 Starting animated simulation...")
    print("UI Controls available:")
    print("  📊 Real-time Factor Slider: Adjust simulation speed")
    print("  🎯 Axis Toggle: Show/hide coordinate axes") 
    print("  🚧 Collision Toggle: Show/hide collision geometry")
    print("  🕸️ Wireframe Toggle: Switch between surface/wireframe rendering")
    print("  ▶️ Play/Pause Button: Pause/resume simulation")
    print("  🔄 Reset Button: Reset robot to initial position")
    print("Close the window to exit.")
    
    # Run with duration for testing
    sim.run(duration=30.0)  # 30 seconds
    
    print(f"✅ Animation test completed. Total frames: {control_count}")

if __name__ == "__main__":
    main()