import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import simpy
import math
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity


def print_pose(name: str, pose: Pose):
    print(f"{name}: pos=({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f}) " +
          f"rot=({math.degrees(pose.roll):.1f}°, {math.degrees(pose.pitch):.1f}°, {math.degrees(pose.yaw):.1f}°)")


def main():
    env = simpy.Environment()
    
    # Create three objects: A, B, C
    obj_a = SimulationObject(env, ObjectParameters(
        name="object_A", object_type=ObjectType.DYNAMIC, 
        initial_pose=Pose(x=0.0, y=0.0, z=0.0), update_interval=0.1))
    
    obj_b = SimulationObject(env, ObjectParameters(
        name="object_B", object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=2.0, y=0.0, z=0.0), update_interval=0.1))
    
    obj_c = SimulationObject(env, ObjectParameters(
        name="object_C", object_type=ObjectType.DYNAMIC,
        initial_pose=Pose(x=1.0, y=2.0, z=0.0), update_interval=0.1))
    
    print("=== Initial positions ===")
    print_pose("Object A", obj_a.get_pose())
    print_pose("Object B", obj_b.get_pose()) 
    print_pose("Object C", obj_c.get_pose())
    
    # Connect A to B (B is 2m forward of A)
    obj_a.connect_to(obj_b, Pose(x=2.0, y=0.0, z=0.0))
    
    # Connect B to C (C is at specific relative position to B)
    obj_b.connect_to(obj_c, Pose(x=-1.0, y=2.0, z=0.0))
    
    print(f"\nConnections: A<->B: {obj_a.is_connected_to(obj_b)}")
    print(f"Connections: B<->C: {obj_b.is_connected_to(obj_c)}")
    print(f"A connected to {len(obj_a.get_connected_objects())} objects")
    print(f"B connected to {len(obj_b.get_connected_objects())} objects")
    print(f"C connected to {len(obj_c.get_connected_objects())} objects")
    
    print("\n=== After connecting ===")
    print_pose("Object A", obj_a.get_pose())
    print_pose("Object B", obj_b.get_pose())
    print_pose("Object C", obj_c.get_pose())
    
    # Move A - should affect both B and C
    print("\n=== Moving A (should move B and C too) ===")
    obj_a.set_velocity(Velocity(linear_x=1.0, angular_z=math.pi/4))
    env.run(until=2)
    obj_a.stop()
    
    print_pose("Object A", obj_a.get_pose())
    print_pose("Object B", obj_b.get_pose())
    print_pose("Object C", obj_c.get_pose())
    
    # Move B - should affect both A and C
    print("\n=== Moving B (should move A and C too) ===")
    obj_b.set_velocity(Velocity(linear_y=1.0, angular_z=math.pi/6))
    env.run(until=3)
    obj_b.stop()
    
    print_pose("Object A", obj_a.get_pose())
    print_pose("Object B", obj_b.get_pose())
    print_pose("Object C", obj_c.get_pose())
    
    # Move C - should affect both A and B
    print("\n=== Moving C (should move A and B too) ===")
    obj_c.set_velocity(Velocity(linear_x=-0.5, angular_z=-math.pi/8))
    env.run(until=4)
    obj_c.stop()
    
    print_pose("Object A", obj_a.get_pose())
    print_pose("Object B", obj_b.get_pose())
    print_pose("Object C", obj_c.get_pose())
    
    # Disconnect B from C
    print("\n=== Disconnecting B from C ===")
    obj_b.disconnect_from(obj_c)
    print(f"B<->C connected: {obj_b.is_connected_to(obj_c)}")
    
    # Move A again - should only affect B now
    print("\n=== Moving A after disconnecting C (only B should follow) ===")
    obj_a.set_velocity(Velocity(angular_z=math.pi/2))
    env.run(until=5)
    obj_a.stop()
    
    print_pose("Object A", obj_a.get_pose())
    print_pose("Object B", obj_b.get_pose())
    print_pose("Object C (independent)", obj_c.get_pose())


if __name__ == "__main__":
    main()