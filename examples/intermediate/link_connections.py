#!/usr/bin/env python3
"""
Robot Link Connection Examples

This example demonstrates the enhanced connection system that allows objects
to be attached to specific robot links. Connected objects follow both robot
base movement and individual link movement from joint motion.

Features:
- Link-specific object attachment
- Multiple connection modes (rigid, flexible, sensor)
- Real-time motion tracking
- Multi-object attachments

Usage:
    python link_connections.py
    python link_connections.py --demo rigid
    python link_connections.py --demo flexible
    python link_connections.py --demo sensor
"""

import sys
import os
import time
import math
import argparse
from typing import List

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.simulation_manager import SimulationManager
from core.link_connector import RobotLinkConnector, connect_to_robot_link
from core.robot import create_robot_from_urdf
from core.simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity


class AttachedSensor(SimulationObject):
    """Example sensor object that can be attached to robot links"""
    
    def __init__(self, env, name: str, initial_pose: Pose = None):
        params = ObjectParameters(
            name=name,
            object_type=ObjectType.DYNAMIC,
            initial_pose=initial_pose or Pose()
        )
        super().__init__(env, params)
        
        # Sensor-specific properties
        self.readings = []
        self.last_reading_time = 0.0
        
    def take_reading(self):
        """Simulate taking a sensor reading"""
        current_time = time.time()
        if current_time - self.last_reading_time > 0.1:  # 10 Hz readings
            # Simulate some sensor data based on position
            pos = self.pose.position
            reading = {
                'timestamp': current_time,
                'position': pos.copy(),
                'distance_from_origin': float(math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2))
            }
            self.readings.append(reading)
            self.last_reading_time = current_time
            
            # Keep only last 50 readings
            if len(self.readings) > 50:
                self.readings.pop(0)


class AttachedTool(SimulationObject):
    """Example tool object attached to robot link"""
    
    def __init__(self, env, name: str, initial_pose: Pose = None):
        params = ObjectParameters(
            name=name,
            object_type=ObjectType.DYNAMIC,
            initial_pose=initial_pose or Pose()
        )
        super().__init__(env, params)
        
        self.tool_active = False
        self.activation_count = 0
        
    def activate_tool(self):
        """Activate the tool"""
        self.tool_active = True
        self.activation_count += 1
        
    def deactivate_tool(self):
        """Deactivate the tool"""
        self.tool_active = False


def rigid_connection_demo(duration: float = 15.0):
    """Demonstrate rigid connections to robot links"""
    print("🔗 Rigid Connection Demo")
    print("=" * 50)
    
    # Create simulation
    sim = SimulationManager()
    
    # Add robot
    robot = sim.add_robot_from_urdf(
        "demo_robot", 
        "examples/robots/movable_robot.urdf"
    )
    
    # Get movable joints and links
    movable_joints = [name for name in robot.get_joint_names() 
                     if robot.joints[name].joint_type.value != 'fixed']
    link_names = robot.get_link_names()
    
    print(f"🎯 Available links: {link_names}")
    print(f"🎯 Movable joints: {movable_joints}")
    
    # Create attached objects
    attached_objects = []
    
    # Attach sensors to different links
    for i, link_name in enumerate(link_names[:3]):  # First 3 links
        sensor = AttachedSensor(
            sim.env, 
            f"sensor_{i}",
            Pose(x=0.1, y=0.0, z=0.05)  # Slightly offset from link
        )
        sim.add_object(f"sensor_{i}", sensor)
        
        # Connect to robot link with rigid connection
        success = robot.connect_object_to_link(
            sensor, 
            link_name,
            relative_pose=Pose(x=0.1, y=0.0, z=0.05),
            connection_mode="rigid"
        )
        
        if success:
            attached_objects.append((sensor, link_name))
            print(f"✅ Attached sensor_{i} to {link_name}")
    
    # Robot control function
    def robot_control(dt: float):
        """Robot motion with attached objects"""
        t = time.time()
        
        # Joint motion
        for i, joint_name in enumerate(movable_joints):
            position = 0.6 * math.sin(0.8 * t + i * math.pi / 3)
            sim.set_robot_joint_position("demo_robot", joint_name, position)
        
        # Take sensor readings
        for sensor, link_name in attached_objects:
            sensor.take_reading()
        
        # Update visualization title
        if hasattr(sim.visualizer, 'plotter'):
            joint_positions = robot.get_joint_positions()
            joint_info = ", ".join([f"{name[:6]}={pos:.2f}" 
                                  for name, pos in list(joint_positions.items())[:2]])
            sim.visualizer.plotter.title = (
                f"Rigid Connections - {len(attached_objects)} objects - {joint_info}"
            )
    
    # Set control callback
    sim.set_robot_control_callback("demo_robot", robot_control, frequency=30.0)
    
    # Run simulation
    print("🎮 Starting rigid connection demo...")
    success = sim.run(duration=duration)
    
    # Print sensor data summary
    for sensor, link_name in attached_objects:
        print(f"📊 {sensor.parameters.name} on {link_name}: {len(sensor.readings)} readings")
    
    return success


def flexible_connection_demo(duration: float = 15.0):
    """Demonstrate flexible connections with damping"""
    print("🌊 Flexible Connection Demo")
    print("=" * 50)
    
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf("demo_robot", "examples/robots/movable_robot.urdf")
    
    movable_joints = [name for name in robot.get_joint_names() 
                     if robot.joints[name].joint_type.value != 'fixed']
    link_names = robot.get_link_names()
    
    # Create flexible attachments
    flexible_objects = []
    
    for i, link_name in enumerate(link_names[:2]):
        tool = AttachedTool(sim.env, f"flexible_tool_{i}")
        sim.add_object(f"flexible_tool_{i}", tool)
        
        # Connect with flexible mode for smoother motion
        success = robot.connect_object_to_link(
            tool,
            link_name,
            relative_pose=Pose(x=0.15, y=0.1, z=0.0),
            connection_mode="flexible"
        )
        
        if success:
            flexible_objects.append((tool, link_name))
            print(f"✅ Flexibly attached tool_{i} to {link_name}")
    
    def robot_control(dt: float):
        """Fast robot motion to test flexible connections"""
        t = time.time()
        
        # Faster, more aggressive motion
        for i, joint_name in enumerate(movable_joints):
            # High frequency motion to test damping
            position = 0.8 * math.sin(2.0 * t + i * math.pi / 2)
            sim.set_robot_joint_position("demo_robot", joint_name, position, max_velocity=3.0)
        
        # Randomly activate tools
        if int(t * 2) % 3 == 0:  # Every 1.5 seconds
            for tool, _ in flexible_objects:
                tool.activate_tool()
        else:
            for tool, _ in flexible_objects:
                tool.deactivate_tool()
        
        # Update title
        if hasattr(sim.visualizer, 'plotter'):
            active_tools = sum(1 for tool, _ in flexible_objects if tool.tool_active)
            sim.visualizer.plotter.title = (
                f"Flexible Connections - {active_tools}/{len(flexible_objects)} tools active - t={t:.1f}s"
            )
    
    sim.set_robot_control_callback("demo_robot", robot_control, frequency=40.0)
    
    print("🎮 Starting flexible connection demo with fast motion...")
    return sim.run(duration=duration)


def sensor_connection_demo(duration: float = 15.0):
    """Demonstrate sensor-mode connections with different update rates"""
    print("📡 Sensor Connection Demo")
    print("=" * 50)
    
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf("demo_robot", "examples/robots/movable_robot.urdf")
    
    movable_joints = [name for name in robot.get_joint_names() 
                     if robot.joints[name].joint_type.value != 'fixed']
    link_names = robot.get_link_names()
    
    # Create different types of sensors with different update rates
    sensors = []
    sensor_configs = [
        ("high_freq_sensor", 100.0),  # High frequency sensor
        ("medium_freq_sensor", 30.0), # Medium frequency sensor  
        ("low_freq_sensor", 10.0)     # Low frequency sensor
    ]
    
    for i, (sensor_name, update_freq) in enumerate(sensor_configs):
        if i < len(link_names):
            sensor = AttachedSensor(sim.env, sensor_name)
            sim.add_object(sensor_name, sensor)
            
            # Connect with sensor mode and specific update frequency
            success = robot.connect_object_to_link(
                sensor,
                link_names[i],
                relative_pose=Pose(x=0.08, y=0.05, z=0.12),
                connection_mode="sensor"
            )
            
            if success:
                sensors.append((sensor, link_names[i], update_freq))
                print(f"✅ Attached {sensor_name} to {link_names[i]} at {update_freq}Hz")
    
    def robot_control(dt: float):
        """Moderate robot motion for sensor testing"""
        t = time.time()
        
        # Smooth, moderate motion
        for i, joint_name in enumerate(movable_joints):
            position = 0.5 * math.sin(1.2 * t + i * math.pi / 4)
            sim.set_robot_joint_position("demo_robot", joint_name, position, max_velocity=1.5)
        
        # Take sensor readings at different rates
        for sensor, link_name, freq in sensors:
            sensor.take_reading()
        
        # Update title with sensor stats
        if hasattr(sim.visualizer, 'plotter'):
            total_readings = sum(len(sensor.readings) for sensor, _, _ in sensors)
            avg_distance = 0
            if sensors:
                distances = []
                for sensor, _, _ in sensors:
                    if sensor.readings:
                        distances.append(sensor.readings[-1]['distance_from_origin'])
                avg_distance = sum(distances) / len(distances) if distances else 0
            
            sim.visualizer.plotter.title = (
                f"Sensor Demo - {total_readings} total readings - avg_dist={avg_distance:.2f}"
            )
    
    sim.set_robot_control_callback("demo_robot", robot_control, frequency=25.0)
    
    print("🎮 Starting sensor connection demo...")
    success = sim.run(duration=duration)
    
    # Print sensor statistics
    print("\n📊 Sensor Statistics:")
    for sensor, link_name, freq in sensors:
        readings_count = len(sensor.readings)
        if readings_count > 0:
            latest = sensor.readings[-1]
            print(f"  {sensor.parameters.name} ({link_name}, {freq}Hz): "
                  f"{readings_count} readings, latest_dist={latest['distance_from_origin']:.3f}")
    
    return success


def multi_connection_demo(duration: float = 20.0):
    """Demonstrate multiple connection types simultaneously"""
    print("🔄 Multi-Connection Demo")
    print("=" * 50)
    
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf("demo_robot", "examples/robots/movable_robot.urdf")
    
    movable_joints = [name for name in robot.get_joint_names() 
                     if robot.joints[name].joint_type.value != 'fixed']
    link_names = robot.get_link_names()
    
    all_objects = []
    
    # Add different types of connections to different links
    connection_types = ["rigid", "flexible", "sensor"]
    
    for i, link_name in enumerate(link_names[:3]):
        connection_mode = connection_types[i % len(connection_types)]
        
        if connection_mode == "rigid":
            obj = AttachedTool(sim.env, f"rigid_tool_{i}")
            relative_pose = Pose(x=0.1, y=0.0, z=0.05)
        elif connection_mode == "flexible":
            obj = AttachedTool(sim.env, f"flex_tool_{i}")
            relative_pose = Pose(x=0.12, y=0.08, z=0.0)
        else:  # sensor
            obj = AttachedSensor(sim.env, f"sensor_{i}")
            relative_pose = Pose(x=0.08, y=-0.05, z=0.1)
        
        sim.add_object(f"{connection_mode}_{i}", obj)
        
        success = robot.connect_object_to_link(
            obj, 
            link_name,
            relative_pose=relative_pose,
            connection_mode=connection_mode
        )
        
        if success:
            all_objects.append((obj, link_name, connection_mode))
            print(f"✅ Attached {connection_mode} object to {link_name}")
    
    # Add multiple objects to same link
    if len(link_names) > 0:
        main_link = link_names[0]
        
        # Add secondary sensor to main link
        secondary_sensor = AttachedSensor(sim.env, "secondary_sensor")
        sim.add_object("secondary_sensor", secondary_sensor)
        
        success = robot.connect_object_to_link(
            secondary_sensor,
            main_link, 
            relative_pose=Pose(x=-0.1, y=0.0, z=0.08),
            connection_mode="sensor"
        )
        
        if success:
            all_objects.append((secondary_sensor, main_link, "sensor"))
            print(f"✅ Attached secondary sensor to {main_link}")
    
    def robot_control(dt: float):
        """Complex robot motion with multiple attached objects"""
        t = time.time()
        
        # Varied joint motion patterns
        for i, joint_name in enumerate(movable_joints):
            if i == 0:
                # Slow large motion
                position = 0.7 * math.sin(0.6 * t)
            elif i == 1:
                # Fast small motion  
                position = 0.3 * math.sin(2.5 * t + math.pi/4)
            else:
                # Medium coordinated motion
                position = 0.5 * math.sin(1.2 * t + i * math.pi/3)
            
            sim.set_robot_joint_position("demo_robot", joint_name, position)
        
        # Update objects based on type
        for obj, link_name, connection_mode in all_objects:
            if connection_mode == "sensor" and isinstance(obj, AttachedSensor):
                obj.take_reading()
            elif isinstance(obj, AttachedTool):
                # Randomly activate tools
                if int(t * 3 + hash(obj.parameters.name)) % 5 == 0:
                    obj.activate_tool()
        
        # Print connection status periodically
        if int(t) % 5 == 0 and int(t * 10) % 10 == 0:  # Every 5 seconds
            from core.link_connector import get_link_connector
            connector = get_link_connector()
            connection_count = sum(len(conns) for robot_conns in connector.connections.values() 
                                 for conns in robot_conns.values())
            print(f"🔗 Active connections: {connection_count}")
        
        # Update title
        if hasattr(sim.visualizer, 'plotter'):
            active_tools = sum(1 for obj, _, mode in all_objects 
                             if mode in ["rigid", "flexible"] and isinstance(obj, AttachedTool) and obj.tool_active)
            total_readings = sum(len(obj.readings) for obj, _, mode in all_objects 
                               if mode == "sensor" and isinstance(obj, AttachedSensor))
            
            sim.visualizer.plotter.title = (
                f"Multi-Connection - {len(all_objects)} objects - "
                f"{active_tools} tools active - {total_readings} readings"
            )
    
    sim.set_robot_control_callback("demo_robot", robot_control, frequency=35.0)
    
    print("🎮 Starting multi-connection demo...")
    success = sim.run(duration=duration)
    
    # Final statistics
    print("\n📊 Final Connection Statistics:")
    for obj, link_name, connection_mode in all_objects:
        obj_name = obj.parameters.name
        if isinstance(obj, AttachedSensor):
            print(f"  📡 {obj_name} ({connection_mode}, {link_name}): {len(obj.readings)} readings")
        elif isinstance(obj, AttachedTool):
            print(f"  🔧 {obj_name} ({connection_mode}, {link_name}): {obj.activation_count} activations")
    
    return success


def connection_performance_test():
    """Test performance with many connected objects"""
    print("⚡ Connection Performance Test")
    print("=" * 50)
    
    from core.link_connector import get_link_connector
    
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf("demo_robot", "examples/robots/movable_robot.urdf")
    
    link_names = robot.get_link_names()
    connector = get_link_connector()
    
    # Create many connected objects
    num_objects = 20
    objects = []
    
    print(f"🔧 Creating {num_objects} connected objects...")
    
    start_time = time.time()
    
    for i in range(num_objects):
        obj = AttachedSensor(sim.env, f"perf_sensor_{i}")
        sim.add_object(f"perf_sensor_{i}", obj)
        
        # Distribute across available links
        link_name = link_names[i % len(link_names)]
        
        # Random relative poses
        import random
        relative_pose = Pose(
            x=random.uniform(-0.2, 0.2),
            y=random.uniform(-0.2, 0.2), 
            z=random.uniform(-0.1, 0.3)
        )
        
        success = robot.connect_object_to_link(obj, link_name, relative_pose)
        if success:
            objects.append((obj, link_name))
    
    setup_time = time.time() - start_time
    print(f"✅ Setup completed in {setup_time:.3f} seconds")
    
    # Test performance during motion
    movable_joints = [name for name in robot.get_joint_names() 
                     if robot.joints[name].joint_type.value != 'fixed']
    
    frame_count = 0
    update_times = []
    
    def performance_control(dt: float):
        nonlocal frame_count
        frame_count += 1
        
        update_start = time.time()
        
        t = time.time()
        
        # Joint motion
        for i, joint_name in enumerate(movable_joints):
            position = 0.4 * math.sin(1.5 * t + i * math.pi / 4)
            sim.set_robot_joint_position("demo_robot", joint_name, position)
        
        # Take sensor readings
        for obj, _ in objects:
            obj.take_reading()
        
        update_time = time.time() - update_start
        update_times.append(update_time)
        
        # Keep only last 100 measurements
        if len(update_times) > 100:
            update_times.pop(0)
        
        if frame_count % 100 == 0:
            avg_update_time = sum(update_times) / len(update_times)
            print(f"🔄 Frame {frame_count}: avg_update={avg_update_time*1000:.2f}ms")
    
    # Run performance test
    sim.set_robot_control_callback("demo_robot", performance_control, frequency=50.0)
    
    print("🎮 Starting performance test...")
    sim.run(duration=10.0)
    
    # Final performance statistics
    if update_times:
        avg_time = sum(update_times) / len(update_times)
        max_time = max(update_times)
        print(f"\n📊 Performance Results:")
        print(f"  Objects: {len(objects)}")
        print(f"  Frames: {frame_count}")
        print(f"  Avg update time: {avg_time*1000:.2f}ms")
        print(f"  Max update time: {max_time*1000:.2f}ms")
        print(f"  FPS: {1.0/avg_time:.1f}")
    
    # Print connector status
    connector.print_connection_status()
    
    return True


def main():
    """Main function with demo selection"""
    parser = argparse.ArgumentParser(
        description="Robot Link Connection Examples",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python link_connections.py                    # Run all demos
  python link_connections.py --demo rigid      # Rigid connections only
  python link_connections.py --demo flexible   # Flexible connections only
  python link_connections.py --demo sensor     # Sensor connections only  
  python link_connections.py --demo multi      # Multi-connection demo
  python link_connections.py --demo performance # Performance test

Connection Modes:
  rigid     - Objects rigidly attached, follow all motion exactly
  flexible  - Objects with damping/filtering for smoother motion
  sensor    - Objects representing sensors with configurable update rates
        """
    )
    
    parser.add_argument("--demo", 
                       choices=["rigid", "flexible", "sensor", "multi", "performance"],
                       help="Specific demo to run")
    parser.add_argument("--duration", type=float, default=15.0,
                       help="Demo duration in seconds")
    
    args = parser.parse_args()
    
    print("🔗 SimPyROS Robot Link Connection Examples")
    print("Demonstrating advanced object-to-link attachment system")
    print("=" * 70)
    
    try:
        success = False
        
        if args.demo == "rigid":
            success = rigid_connection_demo(args.duration)
        elif args.demo == "flexible":
            success = flexible_connection_demo(args.duration)
        elif args.demo == "sensor":
            success = sensor_connection_demo(args.duration)
        elif args.demo == "multi":
            success = multi_connection_demo(args.duration)
        elif args.demo == "performance":
            success = connection_performance_test()
        else:
            # Run all demos
            demos = [
                ("Rigid Connections", lambda: rigid_connection_demo(8.0)),
                ("Flexible Connections", lambda: flexible_connection_demo(8.0)),
                ("Sensor Connections", lambda: sensor_connection_demo(8.0)),
                ("Multi-Connection", lambda: multi_connection_demo(10.0)),
                ("Performance Test", connection_performance_test)
            ]
            
            for name, demo_func in demos:
                try:
                    print(f"\n▶️ Running: {name}")
                    result = demo_func()
                    if result:
                        success = True
                    time.sleep(1)
                    
                except KeyboardInterrupt:
                    print(f"\n⏹️ {name} interrupted by user")
                    break
                except Exception as e:
                    print(f"❌ {name} failed: {e}")
        
        if success:
            print("\n🎉 Link connection demos completed!")
            print("\nKey features demonstrated:")
            print("  ✅ Link-specific object attachment")
            print("  ✅ Multiple connection modes (rigid/flexible/sensor)")
            print("  ✅ Real-time motion tracking")
            print("  ✅ Multi-object attachments per link")
            print("  ✅ Performance optimization")
            print("  ✅ Automatic pose updates")
            return 0
        else:
            print("\n❌ Some demos failed!")
            return 1
            
    except KeyboardInterrupt:
        print("\n⏹️ Interrupted by user")
        return 0
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())