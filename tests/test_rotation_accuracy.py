import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import simpy
import math
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity


def test_rotation_accuracy():
    """Test rotation accuracy with different time steps"""
    
    print("=== Rotation Accuracy Test ===")
    
    for update_interval in [0.1, 0.05, 0.01]:
        env = simpy.Environment()
        
        obj = SimulationObject(env, ObjectParameters(
            name="test_object",
            object_type=ObjectType.DYNAMIC,
            initial_pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
            update_interval=update_interval
        ))
        
        # Set angular velocity for exactly 90 degrees rotation in 1 second
        target_rotation = math.pi / 2  # 90 degrees
        time_duration = 1.0  # 1 second
        angular_velocity = target_rotation / time_duration
        
        obj.set_velocity(Velocity(angular_z=angular_velocity))
        
        print(f"\nUpdate interval: {update_interval}s")
        print(f"Target rotation: {math.degrees(target_rotation):.1f}°")
        print(f"Angular velocity: {math.degrees(angular_velocity):.1f}°/s")
        print(f"Expected steps: {time_duration / update_interval}")
        
        env.run(until=time_duration)
        obj.stop()
        
        final_yaw = obj.get_pose().yaw
        print(f"Final rotation: {math.degrees(final_yaw):.2f}°")
        print(f"Error: {math.degrees(final_yaw - target_rotation):.2f}°")


def test_with_child():
    """Test rotation with attached child object"""
    env = simpy.Environment()
    
    parent_obj = SimulationObject(env, ObjectParameters(
        name="parent", object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=0.0, z=0.0), update_interval=0.01))
    
    child_obj = SimulationObject(env, ObjectParameters(
        name="child", object_type=ObjectType.STATIC,
        initial_pose=Pose(x=1.0, y=0.0, z=0.5)))
    
    parent_obj.connect_to(child_obj, Pose(x=1.0, y=0.0, z=0.5))
    
    print("\n=== Child Attachment Test (Fine Resolution) ===")
    print(f"Initial parent: ({parent_obj.pose.x:.2f}, {parent_obj.pose.y:.2f}) {math.degrees(parent_obj.pose.yaw):.1f}°")
    print(f"Initial child: ({child_obj.pose.x:.2f}, {child_obj.pose.y:.2f}) {math.degrees(child_obj.pose.yaw):.1f}°")
    
    # Rotate exactly 90 degrees
    parent_obj.set_velocity(Velocity(angular_z=math.pi/2))  # 90°/s
    env.run(until=1.0)  # Run for exactly 1 second
    parent_obj.stop()
    
    print(f"Final parent: ({parent_obj.pose.x:.2f}, {parent_obj.pose.y:.2f}) {math.degrees(parent_obj.pose.yaw):.1f}°")
    print(f"Final child: ({child_obj.pose.x:.2f}, {child_obj.pose.y:.2f}) {math.degrees(child_obj.pose.yaw):.1f}°")
    
    # Check if child is at expected position (should be at (0, 1) after 90° rotation)
    expected_x = 0.0
    expected_y = 1.0
    actual_x = child_obj.pose.x
    actual_y = child_obj.pose.y
    
    print(f"Expected child position: ({expected_x:.2f}, {expected_y:.2f})")
    print(f"Position error: ({actual_x - expected_x:.3f}, {actual_y - expected_y:.3f})")


if __name__ == "__main__":
    test_rotation_accuracy()
    test_with_child()