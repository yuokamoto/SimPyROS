#!/usr/bin/env python3
"""
Test pause functionality with mobile robot motion
"""
import time
import math
from core.simulation_manager import SimulationManager
from core.simulation_object import Velocity

def main():
    print("🚗 Testing pause functionality with mobile robot")
    
    # Create simulation 
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf('mobile_robot', 'examples/robots/mobile_robot.urdf')

    def mobile_control(dt):
        t = time.time()
        
        # Move robot in circular motion
        linear_velocity = (0.5 * math.cos(t), 0.5 * math.sin(t), 0.0)
        angular_velocity = (0.0, 0.0, 0.3)
        
        velocity = Velocity(
            linear_x=linear_velocity[0], linear_y=linear_velocity[1], linear_z=linear_velocity[2],
            angular_x=angular_velocity[0], angular_y=angular_velocity[1], angular_z=angular_velocity[2]
        )
        robot.set_velocity(velocity)
        
        # Also control joints if any
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        for i, joint_name in enumerate(joint_names):
            position = 0.3 * math.sin(t + i)
            sim.set_robot_joint_position('mobile_robot', joint_name, position)

    # Set control callback
    sim.set_robot_control_callback('mobile_robot', mobile_control, frequency=20.0)
    
    print("🚀 Starting mobile robot simulation...")
    print("UI Controls:")
    print("  ▶️ Play/Pause Button: Test pausing mobile robot motion")
    print("  🎯 Axis Toggle: Test coordinate axis display on/off")
    print("  📊 Real-time Factor: Adjust simulation speed")
    print("  🔄 Reset Button: Reset robot position")
    print("Close window to exit.")
    
    # Run simulation
    sim.run(duration=20.0)
    print("✅ Mobile robot pause test completed")

if __name__ == "__main__":
    main()