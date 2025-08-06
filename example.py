import simpy
import math
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity


def print_pose(name: str, pose: Pose):
    print(f"{name}: pos=({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f}) " +
          f"rot=({math.degrees(pose.roll):.1f}°, {math.degrees(pose.pitch):.1f}°, {math.degrees(pose.yaw):.1f}°)")


def main():
    env = simpy.Environment()
    
    # Parent object (dynamic robot)
    parent_params = ObjectParameters(
        name="parent_robot",
        object_type=ObjectType.DYNAMIC,
        urdf_path="/path/to/robot.urdf",
        initial_pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        update_interval=0.05
    )
    parent_obj = SimulationObject(env, parent_params)
    
    # Child object (sensor attached to front of robot)
    child_params = ObjectParameters(
        name="front_sensor",
        object_type=ObjectType.STATIC,
        sdf_path="/path/to/sensor.sdf",
        initial_pose=Pose(x=1.0, y=0.0, z=0.5)
    )
    child_obj = SimulationObject(env, child_params)
    
    # Connect child with relative position (1m forward, 0.5m up)
    relative_pose = Pose(x=1.0, y=0.0, z=0.5)
    parent_obj.connect_to(child_obj, relative_pose)
    
    print("=== Initial positions ===")
    print_pose("Parent", parent_obj.get_pose())
    print_pose("Child", child_obj.get_pose())
    
    # Test 1: Pure rotation - child should rotate around parent
    print("\n=== Test 1: Parent rotates 90° (child should move in arc) ===")
    velocity = Velocity(angular_z=math.pi/2)  # 90 degrees per second
    parent_obj.set_velocity(velocity)
    
    env.run(until=1)  # Run for 1 second (90° rotation)
    parent_obj.stop()
    
    print_pose("Parent", parent_obj.get_pose())
    print_pose("Child", child_obj.get_pose())
    
    # Test 2: Translation and rotation
    print("\n=== Test 2: Parent moves forward while rotating ===")
    velocity = Velocity(linear_x=1.0, angular_z=math.pi/4)  # Move forward + rotate 45°/sec
    parent_obj.set_velocity(velocity)
    
    env.run(until=3)  # Run for 2 more seconds
    parent_obj.stop()
    
    print_pose("Parent", parent_obj.get_pose())
    print_pose("Child", child_obj.get_pose())
    
    # Test 3: Teleport with rotation
    print("\n=== Test 3: Teleport parent to new position with 180° rotation ===")
    new_pose = Pose(x=5.0, y=5.0, z=1.0, yaw=math.pi)  # 180 degrees
    parent_obj.teleport(new_pose)
    
    print_pose("Parent", parent_obj.get_pose())
    print_pose("Child", child_obj.get_pose())
    
    # Verify child is still 1m "forward" relative to parent (now backward in world frame)
    expected_child_x = parent_obj.pose.x - 1.0  # Should be behind due to 180° rotation
    expected_child_y = parent_obj.pose.y
    print(f"Expected child position: ({expected_child_x:.2f}, {expected_child_y:.2f})")


if __name__ == "__main__":
    main()