#!/usr/bin/env python3

"""
Debug test for robot movement issue
"""

import sys
import os
import time
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from simulation_object import *
from pyvista_visualizer import PyVistaVisualizer, AnimationController
import math

def debug_movement_test():
    print("🔧 Debug Movement Test")
    print("=" * 50)
    
    # Create simple robot
    env = simpy.Environment()
    
    robot_params = ObjectParameters(
        name="debug_robot",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose.from_position_rotation(np.array([0.0, 0.0, 0.1]), Rotation.identity())
    )
    robot = SimulationObject(env, robot_params)
    
    # Setup visualization
    visualizer = PyVistaVisualizer()
    animation_controller = AnimationController(visualizer.plotter, visualizer.pv)
    
    # Create simple box mesh for robot
    robot_mesh = visualizer.pv.Cube()
    animation_controller.add_robot("debug_robot", robot_mesh, color='blue')
    
    print("✅ Simple robot mesh added")
    
    # Manual position updates with debugging
    positions = [
        [0.0, 0.0, 0.1],
        [1.0, 0.0, 0.1], 
        [1.0, 1.0, 0.1],
        [0.0, 1.0, 0.1],
        [0.0, 0.0, 0.1]
    ]
    
    print("🔄 Testing position updates...")
    
    for i, pos in enumerate(positions):
        print(f"Step {i}: Moving to {pos}")
        
        # Update robot position
        robot.pose.position = np.array(pos)
        
        # Update visualization
        success = animation_controller.update_robot_pose_efficient("debug_robot", robot.pose)
        print(f"  Update result: {success}")
        
        # Force render update
        if hasattr(animation_controller.plotter, 'render'):
            animation_controller.plotter.render()
        
        time.sleep(1)
    
    print("✅ Movement test completed")
    
    # Check actor state
    if "debug_robot" in animation_controller.actors:
        actor = animation_controller.actors["debug_robot"]
        print(f"Actor type: {type(actor)}")
        if hasattr(actor, 'GetMatrix'):
            matrix = actor.GetMatrix()
            print(f"Actor matrix elements: {[matrix.GetElement(i,j) for i in range(4) for j in range(4)]}")

if __name__ == "__main__":
    debug_movement_test()