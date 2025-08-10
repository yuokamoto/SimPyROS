#!/usr/bin/env python3

"""
Debug test specifically for URDF robot movement
"""

import sys
import os
import time
import math
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from simulation_object import *
from pyvista_visualizer import PyVistaVisualizer, AnimationController, RobotMeshFactory

def debug_urdf_movement():
    print("🔧 Debug URDF Robot Movement Test")
    print("=" * 50)
    
    # Create environment
    env = simpy.Environment()
    
    # Create URDF robot
    robot_params = ObjectParameters(
        name="urdf_robot",
        object_type=ObjectType.DYNAMIC,
        urdf_path="examples/robots/simple_robot.urdf",
        initial_pose=Pose.from_position_rotation(np.array([0.0, 0.0, 0.1]), Rotation.identity())
    )
    robot = SimulationObject(env, robot_params)
    
    # Setup visualization
    visualizer = PyVistaVisualizer()
    animation_controller = AnimationController(visualizer.plotter, visualizer.pv)
    
    # Load URDF mesh
    print("🔄 Loading URDF robot mesh...")
    mesh_factory = RobotMeshFactory()
    robot_mesh = mesh_factory.create_from_urdf(visualizer.pv, "examples/robots/simple_robot.urdf")
    
    if robot_mesh is None:
        print("❌ Failed to load URDF mesh")
        return
    
    # Add robot to animation controller  
    animation_controller.add_robot("urdf_robot", robot_mesh, color='orange')
    print("✅ URDF robot mesh added")
    
    # Check if robot has individual meshes
    actors = animation_controller.actors.get("urdf_robot")
    if isinstance(actors, dict):
        print(f"🔍 Robot has individual link actors: {list(actors.keys())}")
        
        # Test transformation on each individual actor
        test_positions = [[0,0,0.1], [1,0,0.1], [2,0,0.1]]
        
        for i, pos in enumerate(test_positions):
            print(f"\nStep {i}: Moving to {pos}")
            
            # Create transformation matrix
            transform_matrix = np.eye(4)
            transform_matrix[:3, 3] = pos  # Set translation
            
            # Convert to VTK matrix
            import vtk
            vtk_matrix = vtk.vtkMatrix4x4()
            for r in range(4):
                for c in range(4):
                    vtk_matrix.SetElement(r, c, transform_matrix[r, c])
            
            print(f"  Transform matrix translation: {transform_matrix[:3, 3]}")
            
            # Apply to each individual actor
            for link_name, actor in actors.items():
                print(f"    Setting transform on {link_name}")
                actor.SetUserMatrix(vtk_matrix)
                
                # Check if it took effect
                current_matrix = actor.GetMatrix()
                current_pos = [current_matrix.GetElement(i, 3) for i in range(3)]
                print(f"    Actor {link_name} position: {current_pos}")
            
            time.sleep(1)
    
    else:
        print(f"🔍 Robot has single actor: {type(actors)}")
        
        # Test single actor transformation
        test_positions = [[0,0,0.1], [1,0,0.1], [2,0,0.1]]
        
        for i, pos in enumerate(test_positions):
            print(f"\nStep {i}: Moving to {pos}")
            
            # Update robot pose
            robot.pose.position = np.array(pos)
            
            # Update visualization 
            success = animation_controller.update_robot_pose_efficient("urdf_robot", robot.pose)
            print(f"  Update result: {success}")
            
            # Check matrix
            current_matrix = actors.GetMatrix()
            current_pos = [current_matrix.GetElement(i, 3) for i in range(3)]
            print(f"  Actor position: {current_pos}")
            
            time.sleep(1)

if __name__ == "__main__":
    debug_urdf_movement()