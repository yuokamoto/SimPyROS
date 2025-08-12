#!/usr/bin/env python3
import time
import math
from core.simulation_manager import SimulationManager, SimulationConfig

def main():
    # Headless mode for debugging
    config = SimulationConfig(visualization=False, update_rate=10.0)
    sim = SimulationManager(config)
    robot = sim.add_robot_from_urdf('test_robot', 'examples/robots/articulated_arm_robot.urdf')

    control_count = 0
    def test_control(dt):
        nonlocal control_count
        control_count += 1
        t = time.time()
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        print(f"Control update #{control_count} at t={t:.2f}, dt={dt:.4f}")
        for i, joint_name in enumerate(joint_names):
            position = 0.5 * math.sin(t + i * math.pi / 3)
            sim.set_robot_joint_position('test_robot', joint_name, position)
            current_pos = robot.get_joint_positions().get(joint_name, 0)
            print(f'  {joint_name}: target={position:.3f}, current={current_pos:.3f}')

    sim.set_robot_control_callback('test_robot', test_control, frequency=2.0)
    print("Starting simulation...")
    sim.run(duration=3.0)
    print(f"Simulation finished. Total control calls: {control_count}")

if __name__ == "__main__":
    main()