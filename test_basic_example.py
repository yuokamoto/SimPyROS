#!/usr/bin/env python3
"""Test basic_simulation.py first example"""
import sys, os, math, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from core.simulation_manager import SimulationManager

def main():
    print('🎯 Testing basic_simulation.py first example')
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf(
        name='my_robot',
        urdf_path='examples/robots/articulated_arm_robot.urdf'
    )

    def my_control(dt):
        t = time.time()
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        for i, joint_name in enumerate(joint_names):
            position = 0.5 * math.sin(t + i * math.pi / 3)
            sim.set_robot_joint_position('my_robot', joint_name, position)

    sim.set_robot_control_callback('my_robot', my_control, frequency=20.0)
    print("⏰ Running for 2s then visualization continues...")
    sim.run(duration=2.0)
    print('✅ basic_simulation.py example working!')

if __name__ == "__main__":
    main()