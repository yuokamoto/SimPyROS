#!/usr/bin/env python3
"""
Sample Robot Mesh Factory
Example implementations of geometric robot models for demonstration purposes
"""

import math
from typing import Optional


class SampleRobotFactory:
    """Factory class for creating sample geometric robot meshes"""
    @staticmethod
    def create_robot_mesh(pv_module, robot_type):
        """Create robot mesh based on type"""
        if robot_type == 'basic_robot':
            return SampleRobotFactory.create_basic_robot(pv_module)
        elif robot_type == 'wheeled_robot':
            return SampleRobotFactory.create_wheeled_robot(pv_module)
        elif robot_type == 'quadcopter':
            return SampleRobotFactory.create_quadcopter(pv_module)
        else:
            print(f"Unknown robot type: {robot_type}")
            return None

    @staticmethod
    def create_basic_robot(pv_module) -> Optional:
        """Create a basic robot with base, arm, and gripper"""
        try:
            # Robot base (box)
            base = pv_module.Box(bounds=[-0.5, 0.5, -0.3, 0.3, 0, 0.2])
            
            # Robot arm (cylinder)
            arm = pv_module.Cylinder(
                center=[0.3, 0, 0.4], 
                direction=[0, 0, 1], 
                radius=0.1, 
                height=0.6,
                resolution=16
            )
            
            # End effector (sphere)
            gripper = pv_module.Sphere(
                center=[0.3, 0, 0.8], 
                radius=0.08,
                phi_resolution=16, 
                theta_resolution=16
            )
            
            # Combine meshes
            robot = base + arm + gripper
            return robot
            
        except Exception as e:
            print(f"Error creating basic robot mesh: {e}")
            return None
            
    @staticmethod
    def create_wheeled_robot(pv_module) -> Optional:
        """Create a wheeled robot (like a mobile base)"""
        try:
            # Robot base
            base = pv_module.Box(bounds=[-0.5, 0.5, -0.3, 0.3, 0, 0.2])
            
            # Robot arm
            arm = pv_module.Cylinder(
                center=[0.3, 0, 0.4], 
                direction=[0, 0, 1], 
                radius=0.1, 
                height=0.6,
                resolution=16
            )
            
            # End effector
            gripper = pv_module.Sphere(
                center=[0.3, 0, 0.8], 
                radius=0.08,
                phi_resolution=16, 
                theta_resolution=16
            )
            
            # Wheels
            wheel_positions = [[-0.4, -0.35, -0.1], [-0.4, 0.35, -0.1], 
                             [0.4, -0.35, -0.1], [0.4, 0.35, -0.1]]
            wheels = []
            for pos in wheel_positions:
                wheel = pv_module.Cylinder(
                    center=pos, 
                    direction=[0, 1, 0], 
                    radius=0.12, 
                    height=0.05, 
                    resolution=12
                )
                wheels.append(wheel)
            
            # Combine all parts
            robot = base + arm + gripper
            for wheel in wheels:
                robot = robot + wheel
                
            return robot
            
        except Exception as e:
            print(f"Error creating wheeled robot mesh: {e}")
            return None
            
    @staticmethod
    def create_quadcopter(pv_module) -> Optional:
        """Create a quadcopter mesh"""
        try:
            # Center body
            center = pv_module.Sphere(
                center=[0, 0, 0], 
                radius=0.05, 
                phi_resolution=10, 
                theta_resolution=10
            )
            
            # Propeller arms and props
            arms = []
            for angle in [0, 90, 180, 270]:
                rad = math.radians(angle)
                arm_end = [0.2 * math.cos(rad), 0.2 * math.sin(rad), 0]
                
                arm = pv_module.Cylinder(
                    center=[arm_end[0]/2, arm_end[1]/2, 0], 
                    direction=arm_end, 
                    radius=0.01, 
                    height=0.2, 
                    resolution=6
                )
                prop = pv_module.Cylinder(
                    center=arm_end, 
                    direction=[0, 0, 1], 
                    radius=0.05, 
                    height=0.005, 
                    resolution=8
                )
                arms.append(arm + prop)
            
            result = center
            for arm in arms:
                result = result + arm
            return result
            
        except Exception as e:
            print(f"Error creating quadcopter mesh: {e}")
            return None

    @staticmethod
    def create_humanoid_robot(pv_module) -> Optional:
        """Create a simple humanoid robot"""
        try:
            # Torso
            torso = pv_module.Box(bounds=[-0.15, 0.15, -0.1, 0.1, 0, 0.5])
            
            # Head
            head = pv_module.Sphere(center=[0, 0, 0.65], radius=0.1)
            
            # Arms
            left_arm = pv_module.Cylinder(
                center=[-0.25, 0, 0.3], 
                direction=[0, 0, 1], 
                radius=0.05, 
                height=0.4
            )
            right_arm = pv_module.Cylinder(
                center=[0.25, 0, 0.3], 
                direction=[0, 0, 1], 
                radius=0.05, 
                height=0.4
            )
            
            # Legs
            left_leg = pv_module.Cylinder(
                center=[-0.08, 0, -0.25], 
                direction=[0, 0, 1], 
                radius=0.06, 
                height=0.5
            )
            right_leg = pv_module.Cylinder(
                center=[0.08, 0, -0.25], 
                direction=[0, 0, 1], 
                radius=0.06, 
                height=0.5
            )
            
            # Combine all parts
            robot = torso + head + left_arm + right_arm + left_leg + right_leg
            return robot
            
        except Exception as e:
            print(f"Error creating humanoid robot mesh: {e}")
            return None

    @staticmethod
    def get_available_types():
        """Get list of available sample robot types"""
        return ['basic_robot', 'wheeled_robot', 'quadcopter']