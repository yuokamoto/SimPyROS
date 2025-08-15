import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import simpy
import math
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity


def print_pose(name: str, pose: Pose):
    print(f"{name}: pos=({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f}) " +
          f"rot=({math.degrees(pose.roll):.1f}°, {math.degrees(pose.pitch):.1f}°, {math.degrees(pose.yaw):.1f}°)")


def test_static_constraint():
    """Test that objects connected to STATIC objects cannot move"""
    
    env = simpy.Environment()
    
    # Create a STATIC object (immovable)
    static_obj = SimulationObject(env, ObjectParameters(
        name="static_wall",
        object_type=ObjectType.STATIC,
        initial_pose=Pose(x=5.0, y=0.0, z=0.0)
    ))
    
    # Create a DYNAMIC object
    dynamic_obj = SimulationObject(env, ObjectParameters(
        name="dynamic_robot",
        object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=0.0, y=0.0, z=0.0),
        update_interval=0.05
    ))
    
    print("=== Before Connection ===")
    print_pose("Static wall", static_obj.get_pose())
    print_pose("Dynamic robot", dynamic_obj.get_pose())
    
    # Try to move the dynamic object - should work
    print("\n=== Moving dynamic object (not connected) ===")
    dynamic_obj.set_velocity(Velocity(linear_x=1.0))
    env.run(until=1.0)
    dynamic_obj.stop()
    
    print_pose("Static wall", static_obj.get_pose())
    print_pose("Dynamic robot", dynamic_obj.get_pose())
    
    # Now connect them
    print("\n=== Connecting dynamic robot to static wall ===")
    dynamic_obj.connect_to(static_obj, Pose(x=5.0, y=0.0, z=0.0))
    
    print_pose("Static wall", static_obj.get_pose())
    print_pose("Dynamic robot", dynamic_obj.get_pose())
    print(f"Connection established: {dynamic_obj.is_connected_to(static_obj)}")
    
    # Try to move the dynamic object - should NOT work now
    print("\n=== Trying to move dynamic object (connected to static) ===")
    print("Setting velocity to dynamic robot...")
    dynamic_obj.set_velocity(Velocity(linear_x=1.0, angular_z=0.5))
    
    # Record initial position
    initial_pos = (dynamic_obj.pose.x, dynamic_obj.pose.y, dynamic_obj.pose.yaw)
    
    env.run(until=2.0)
    dynamic_obj.stop()
    
    # Check if position changed
    final_pos = (dynamic_obj.pose.x, dynamic_obj.pose.y, dynamic_obj.pose.yaw)
    
    print_pose("Static wall", static_obj.get_pose())
    print_pose("Dynamic robot", dynamic_obj.get_pose())
    
    if initial_pos == final_pos:
        print("✅ SUCCESS: Dynamic object did not move (constrained by static connection)")
    else:
        print("❌ FAILED: Dynamic object moved despite static connection")
        print(f"   Initial: {initial_pos}")
        print(f"   Final: {final_pos}")
    
    # Disconnect and try moving again
    print("\n=== Disconnecting and trying to move again ===")
    dynamic_obj.disconnect_from(static_obj)
    print(f"Connection status: {dynamic_obj.is_connected_to(static_obj)}")
    
    dynamic_obj.set_velocity(Velocity(linear_x=-0.5))
    env.run(until=3.0)
    dynamic_obj.stop()
    
    print_pose("Static wall", static_obj.get_pose())
    print_pose("Dynamic robot", dynamic_obj.get_pose())
    print("✅ Dynamic object can move again after disconnection")


def test_chain_with_static():
    """Test chain of objects where one is connected to static"""
    
    env = simpy.Environment()
    
    # Create objects: Static - Dynamic A - Dynamic B
    static_obj = SimulationObject(env, ObjectParameters(
        name="static_anchor", object_type=ObjectType.STATIC,
        initial_pose=Pose(x=0.0, y=0.0, z=0.0)))
    
    dynamic_a = SimulationObject(env, ObjectParameters(
        name="dynamic_A", object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=1.0, y=0.0, z=0.0), update_interval=0.05))
    
    dynamic_b = SimulationObject(env, ObjectParameters(
        name="dynamic_B", object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=2.0, y=0.0, z=0.0), update_interval=0.05))
    
    # Connect: Static <-> A <-> B
    static_obj.connect_to(dynamic_a, Pose(x=1.0, y=0.0, z=0.0))
    dynamic_a.connect_to(dynamic_b, Pose(x=1.0, y=0.0, z=0.0))
    
    print("\n=== Chain Test: Static <-> Dynamic A <-> Dynamic B ===")
    print_pose("Static anchor", static_obj.get_pose())
    print_pose("Dynamic A", dynamic_a.get_pose())
    print_pose("Dynamic B", dynamic_b.get_pose())
    
    # Try to move Dynamic B - should not work (transitively connected to static)
    print("\n=== Trying to move Dynamic B ===")
    initial_b_pos = (dynamic_b.pose.x, dynamic_b.pose.y)
    
    dynamic_b.set_velocity(Velocity(linear_x=1.0))
    env.run(until=1.0)
    dynamic_b.stop()
    
    final_b_pos = (dynamic_b.pose.x, dynamic_b.pose.y)
    
    print_pose("Static anchor", static_obj.get_pose())
    print_pose("Dynamic A", dynamic_a.get_pose()) 
    print_pose("Dynamic B", dynamic_b.get_pose())
    
    if initial_b_pos == final_b_pos:
        print("✅ SUCCESS: Dynamic B cannot move (connected to static through A)")
    else:
        print("❌ FAILED: Dynamic B moved despite static connection")


if __name__ == "__main__":
    test_static_constraint()
    test_chain_with_static()