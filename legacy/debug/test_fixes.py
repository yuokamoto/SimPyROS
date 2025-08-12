#!/usr/bin/env python3
from core.simulation_manager import SimulationManager
import math
import time

def main():
    print("🧪 Testing fixes for animation and visual origin")
    
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf('test', 'examples/robots/articulated_arm_robot.urdf')

    def test_control(dt):
        t = time.time()
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        print(f"Control update: {len(joint_names)} joints")
        for i, joint_name in enumerate(joint_names):
            pos = 0.5 * math.sin(t + i)
            sim.set_robot_joint_position('test', joint_name, pos)

    sim.set_robot_control_callback('test', test_control, frequency=10.0)
    print('🚀 Starting test with 3s duration...')
    print('Window should stay open after 3 seconds for manual inspection.')
    sim.run(duration=3.0)  # Should stay open after 3s
    print('✅ Test completed')

if __name__ == "__main__":
    main()