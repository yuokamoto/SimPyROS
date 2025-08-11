#!/usr/bin/env python3
import time
import math
from core.simulation_manager import SimulationManager

def main():
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf('test_robot', 'examples/robots/articulated_arm_robot.urdf')

    def test_control(dt):
        t = time.time()
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        print(f"Control update at t={t:.2f}, dt={dt:.4f}")
        for i, joint_name in enumerate(joint_names):
            position = 0.5 * math.sin(t + i * math.pi / 3)
            sim.set_robot_joint_position('test_robot', joint_name, position)
            print(f'  Setting {joint_name} to {position:.3f}')

    sim.set_robot_control_callback('test_robot', test_control, frequency=2.0)
    sim.run(duration=3.0)

if __name__ == "__main__":
    main()