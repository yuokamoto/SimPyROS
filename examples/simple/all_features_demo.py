#!/usr/bin/env python3
"""
All Features Integration Demo

This comprehensive example demonstrates all the enhanced SimPyROS features:
1. Simplified simulation management
2. 3D mesh loading from external repositories
3. Robot link connections
4. Multi-robot coordination

This is the ultimate showcase of how the new features work together.

Usage:
    python all_features_demo.py
    python all_features_demo.py --quick     # Shorter demo
    python all_features_demo.py --headless  # No visualization
"""

import sys
import os
import time
import math
import argparse

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.external_mesh_manager import setup_external_repositories
from core.link_connector import get_link_connector
from core.robot import create_robot_from_urdf
from core.simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity


class SmartSensor(SimulationObject):
    """Advanced sensor with data processing capabilities"""
    
    def __init__(self, env, name: str, sensor_type: str = "lidar"):
        params = ObjectParameters(
            name=name,
            object_type=ObjectType.DYNAMIC,
            initial_pose=Pose()
        )
        super().__init__(env, params)
        
        self.sensor_type = sensor_type
        self.readings = []
        self.processed_data = {}
        self.last_update = 0.0
        
    def process_reading(self):
        """Process sensor data based on current position"""
        current_time = time.time()
        pos = self.pose.position
        
        # Simulate different sensor types
        if self.sensor_type == "lidar":
            # Simulate LIDAR distance measurement
            distance = math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
            reading = {
                'type': 'distance',
                'value': distance,
                'timestamp': current_time,
                'position': pos.copy()
            }
        elif self.sensor_type == "camera":
            # Simulate camera field of view
            angle = math.atan2(pos[1], pos[0])
            reading = {
                'type': 'angle', 
                'value': angle,
                'timestamp': current_time,
                'position': pos.copy()
            }
        else:  # generic
            reading = {
                'type': 'generic',
                'value': pos[2],  # Height as value
                'timestamp': current_time,
                'position': pos.copy()
            }
        
        self.readings.append(reading)
        self.last_update = current_time
        
        # Keep only last 20 readings
        if len(self.readings) > 20:
            self.readings.pop(0)
        
        # Update processed data
        if self.readings:
            values = [r['value'] for r in self.readings]
            self.processed_data = {
                'count': len(values),
                'avg': sum(values) / len(values),
                'min': min(values),
                'max': max(values)
            }


def comprehensive_demo(duration: float = 30.0, setup_repos: bool = True):
    """Comprehensive demo showcasing all features"""
    print("🚀 Comprehensive All Features Demo")
    print("=" * 60)
    
    # Phase 1: Setup external repositories
    if setup_repos:
        print("\n📦 Phase 1: Setting up external repositories...")
        setup_success = setup_external_repositories(['turtlebot3'], force_update=False)
        if not setup_success:
            print("⚠️ Repository setup failed, continuing with local robots...")
    
    # Phase 2: Create simulation with advanced configuration
    print("\n🎮 Phase 2: Creating advanced simulation...")
    config = SimulationConfig(
        update_rate=80.0,  # High update rate
        visualization=True,
        visualization_update_rate=40.0,  # Smooth visualization
        window_size=(1400, 900),  # Large window
        auto_setup_scene=True
    )
    
    sim = SimulationManager(config)
    
    # Phase 3: Add multiple robots with different capabilities
    print("\n🤖 Phase 3: Adding multiple robots...")
    
    # Robot 1: Local URDF robot with joints
    robot1 = sim.add_robot_from_urdf(
        "articulated_robot",
        "examples/robots/articulated_arm_robot.urdf",
        initial_pose=Pose(x=0.0, y=0.0, z=0.0)
    )
    
    # Robot 2: Mobile robot
    robot2 = sim.add_robot_from_urdf(
        "mobile_robot",
        "examples/robots/mobile_robot.urdf", 
        initial_pose=Pose(x=2.0, y=0.0, z=0.0)
    )
    
    # Robot 3: Try to add TurtleBot3 with mesh if available
    turtle_robot = None
    from core.external_mesh_manager import ExternalMeshManager
    mesh_manager = ExternalMeshManager()
    turtle_urdf = mesh_manager.get_urdf_path('turtlebot3', 'waffle_pi')
    
    if turtle_urdf:
        print("✅ Adding TurtleBot3 with 3D meshes...")
        turtle_robot = sim.add_robot_from_urdf(
            "turtlebot3",
            turtle_urdf,
            initial_pose=Pose(x=-2.0, y=0.0, z=0.0)
        )
    else:
        print("⚠️ TurtleBot3 not available, using local robot instead")
        turtle_robot = sim.add_robot_from_urdf(
            "turtlebot3_fallback",
            "examples/robots/collision_robot.urdf",
            initial_pose=Pose(x=-2.0, y=0.0, z=0.0)
        )
    
    print(f"✅ Added {len(sim.robots)} robots successfully")
    
    # Phase 4: Add smart sensors with link connections
    print("\n📡 Phase 4: Adding smart sensors with link connections...")
    
    sensors = []
    
    # Add sensors to articulated robot
    robot1_links = robot1.get_link_names()
    for i, link_name in enumerate(robot1_links[:3]):
        sensor_type = ["lidar", "camera", "generic"][i % 3]
        sensor = SmartSensor(sim.env, f"sensor_{i}", sensor_type)
        sim.add_object(f"sensor_{i}", sensor)
        
        # Connect to robot link
        relative_pose = Pose(
            x=0.1, 
            y=0.05 * (i - 1),  # Spread out sensors
            z=0.08
        )
        
        success = robot1.connect_object_to_link(
            sensor, 
            link_name,
            relative_pose=relative_pose,
            connection_mode="sensor"
        )
        
        if success:
            sensors.append((sensor, robot1, link_name))
            print(f"  ✅ Connected {sensor.sensor_type} sensor to {link_name}")
    
    # Add mobile sensor to mobile robot  
    mobile_sensor = SmartSensor(sim.env, "mobile_sensor", "lidar")
    sim.add_object("mobile_sensor", mobile_sensor)
    
    mobile_links = robot2.get_link_names()
    if mobile_links:
        main_link = mobile_links[0]
        success = robot2.connect_object_to_link(
            mobile_sensor,
            main_link,
            relative_pose=Pose(x=0.0, y=0.0, z=0.15),
            connection_mode="rigid"
        )
        if success:
            sensors.append((mobile_sensor, robot2, main_link))
            print(f"  ✅ Connected mobile sensor to {main_link}")
    
    print(f"✅ Connected {len(sensors)} smart sensors")
    
    # Phase 5: Coordinated multi-robot control
    print("\n🎯 Phase 5: Setting up coordinated control...")
    
    start_time = time.time()
    
    def articulated_robot_control(dt: float):
        """Advanced control for articulated robot"""
        t = time.time() - start_time
        
        # Get movable joints
        movable_joints = [name for name in robot1.get_joint_names() 
                         if robot1.joints[name].joint_type.value != 'fixed']
        
        # Complex coordinated motion
        phase = (t % 20) / 20  # 20-second cycle
        
        if phase < 0.3:
            # Slow coordinated motion
            for i, joint_name in enumerate(movable_joints):
                position = 0.4 * math.sin(0.8 * t + i * math.pi / 3)
                sim.set_robot_joint_position("articulated_robot", joint_name, position)
                
        elif phase < 0.7:
            # Individual joint showcase
            active_joint = int((phase - 0.3) * 10) % len(movable_joints)
            for i, joint_name in enumerate(movable_joints):
                if i == active_joint:
                    position = 0.7 * math.sin(3 * t)
                    sim.set_robot_joint_position("articulated_robot", joint_name, position, max_velocity=2.0)
                else:
                    current_pos = robot1.get_joint_state(joint_name).position
                    target = current_pos * 0.9
                    sim.set_robot_joint_position("articulated_robot", joint_name, target)
        else:
            # Return to home
            for joint_name in movable_joints:
                sim.set_robot_joint_position("articulated_robot", joint_name, 0.0, max_velocity=1.0)
    
    def mobile_robot_control(dt: float):
        """Mobile robot autonomous navigation"""
        t = time.time() - start_time
        
        # Circle motion with variable speed
        radius = 1.5
        angular_freq = 0.3
        linear_speed = 0.4 + 0.2 * math.sin(t * 0.5)
        
        velocity = Velocity(
            linear_x=linear_speed,
            angular_z=angular_freq * math.cos(t * 0.3)
        )
        
        sim.set_robot_velocity("mobile_robot", velocity)
    
    def turtle_robot_control(dt: float):
        """TurtleBot3 advanced pattern"""
        t = time.time() - start_time
        
        # Figure-8 pattern
        scale = 0.8
        speed_factor = 1.2
        
        # Parametric equations for figure-8
        angle = speed_factor * t
        linear_x = scale * 0.5 * (1 + math.cos(angle))
        angular_z = scale * math.sin(2 * angle)
        
        velocity = Velocity(linear_x=linear_x, angular_z=angular_z)
        sim.set_robot_velocity("turtlebot3", velocity)
    
    # Set control callbacks with different frequencies
    sim.set_robot_control_callback("articulated_robot", articulated_robot_control, frequency=40.0)
    sim.set_robot_control_callback("mobile_robot", mobile_robot_control, frequency=30.0) 
    sim.set_robot_control_callback("turtlebot3", turtle_robot_control, frequency=25.0)
    
    print("✅ All robot controllers configured")
    
    # Phase 6: Real-time monitoring and visualization updates
    print("\n📊 Phase 6: Starting comprehensive simulation...")
    
    last_status_update = 0
    
    def global_monitor(dt: float):
        """Global monitoring and sensor processing"""
        nonlocal last_status_update
        
        current_time = time.time()
        t = current_time - start_time
        
        # Process all sensor readings
        for sensor, robot, link_name in sensors:
            sensor.process_reading()
        
        # Update visualization title periodically
        if hasattr(sim.visualizer, 'plotter'):
            total_readings = sum(len(sensor.readings) for sensor, _, _ in sensors)
            
            # Get robot positions
            pos1 = robot1.get_pose().position
            pos2 = robot2.get_pose().position
            pos3 = turtle_robot.get_pose().position if turtle_robot else [0, 0, 0]
            
            sim.visualizer.plotter.title = (
                f"All Features Demo - t={t:.1f}s - "
                f"{len(sim.robots)} robots - {total_readings} sensor readings"
            )
        
        # Print periodic status
        if current_time - last_status_update > 5.0:
            connector = get_link_connector()
            connection_count = sum(len(conns) for robot_conns in connector.connections.values()
                                 for conns in robot_conns.values())
            
            print(f"🔄 t={t:.1f}s - {len(sim.robots)} robots active - "
                  f"{connection_count} connections - {len(sensors)} sensors")
            
            last_status_update = current_time
    
    # Add global monitoring
    sim.set_robot_control_callback("_global_monitor", global_monitor, frequency=20.0)
    
    # Phase 7: Run simulation
    print(f"🎮 Running comprehensive demo for {duration} seconds...")
    print("=" * 60)
    success = sim.run(duration=duration)
    
    # Phase 8: Final statistics
    print("\n📈 Final Statistics:")
    print("=" * 40)
    
    # Robot statistics
    for robot_name, robot in sim.robots.items():
        if robot_name != "_global_monitor":
            pos = robot.get_pose().position
            joints = robot.get_joint_positions()
            print(f"🤖 {robot_name}:")
            print(f"   Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            print(f"   Joints: {len(joints)} ({len([j for j in joints.values() if abs(j) > 0.01])} active)")
    
    # Sensor statistics
    print(f"\n📡 Sensor Data Summary:")
    for sensor, robot, link_name in sensors:
        data = sensor.processed_data
        print(f"   {sensor.sensor_type} on {robot.robot_name}:{link_name}:")
        if data:
            print(f"      Readings: {data['count']}, Avg: {data['avg']:.3f}, "
                  f"Range: [{data['min']:.3f}, {data['max']:.3f}]")
    
    # Connection statistics
    connector = get_link_connector()
    connector.print_connection_status()
    
    # Simulation statistics
    sim_info = sim.get_simulation_info()
    print(f"\n🎯 Simulation Performance:")
    print(f"   Duration: {sim_info['elapsed_time']:.1f} seconds")
    print(f"   Average FPS: {sim_info['average_fps']:.1f}")
    print(f"   Total frames: {sim_info['frame_count']}")
    
    return success


def quick_demo(duration: float = 10.0):
    """Quick demo without external repositories"""
    print("⚡ Quick All Features Demo")
    print("=" * 40)
    
    return comprehensive_demo(duration, setup_repos=False)


def headless_demo(duration: float = 15.0):
    """Headless demo for performance testing"""
    print("🖥️ Headless All Features Demo")
    print("=" * 40)
    
    # Force headless mode
    config = SimulationConfig(
        update_rate=100.0,  # Very high update rate
        visualization=False,  # Headless
        visualization_update_rate=0.0
    )
    
    sim = SimulationManager(config)
    
    # Add robots
    robot1 = sim.add_robot_from_urdf("robot1", "examples/robots/articulated_arm_robot.urdf")
    robot2 = sim.add_robot_from_urdf("robot2", "examples/robots/collision_robot.urdf", 
                                    initial_pose=Pose(x=1.0, y=0.0, z=0.0))
    
    # Add sensors
    sensors = []
    for i, link_name in enumerate(robot1.get_link_names()[:2]):
        sensor = SmartSensor(sim.env, f"headless_sensor_{i}")
        sim.add_object(f"headless_sensor_{i}", sensor)
        robot1.connect_object_to_link(sensor, link_name, connection_mode="sensor")
        sensors.append(sensor)
    
    # Simple control
    frame_count = 0
    
    def headless_control(dt: float):
        nonlocal frame_count
        frame_count += 1
        
        t = time.time()
        
        # Robot motion
        movable_joints = [name for name in robot1.get_joint_names() 
                         if robot1.joints[name].joint_type.value != 'fixed']
        
        for i, joint_name in enumerate(movable_joints):
            position = 0.3 * math.sin(2 * t + i)
            sim.set_robot_joint_position("robot1", joint_name, position)
        
        # Sensor processing
        for sensor in sensors:
            sensor.process_reading()
        
        if frame_count % 500 == 0:  # Every 5 seconds at 100Hz
            print(f"🔄 Frame {frame_count}, {len(sensors)} sensors, "
                  f"total readings: {sum(len(s.readings) for s in sensors)}")
    
    sim.set_robot_control_callback("robot1", headless_control, frequency=50.0)
    
    print("🎮 Running headless demo...")
    success = sim.run(duration=duration)
    
    # Performance summary
    sim_info = sim.get_simulation_info()
    print(f"\n📊 Headless Performance:")
    print(f"   Frames: {frame_count}")
    print(f"   Average FPS: {sim_info['average_fps']:.1f}")
    print(f"   Total sensor readings: {sum(len(s.readings) for s in sensors)}")
    
    return success


def main():
    """Main function with demo options"""
    parser = argparse.ArgumentParser(
        description="All Features Integration Demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This demo showcases all enhanced SimPyROS features:

1. Simplified Simulation Management
   - One-line robot loading
   - Automatic lifecycle management
   - Multi-robot coordination

2. 3D Mesh Visualization  
   - External repository integration
   - TurtleBot3 mesh loading
   - Performance optimization

3. Robot Link Connections
   - Sensor attachment to specific links
   - Multiple connection modes
   - Real-time motion tracking

4. Advanced Features
   - High-frequency updates
   - Smart sensor processing
   - Comprehensive monitoring

Examples:
  python all_features_demo.py              # Full demo with external repos
  python all_features_demo.py --quick      # Quick demo without repos
  python all_features_demo.py --headless   # Performance test mode
        """
    )
    
    parser.add_argument("--quick", action="store_true",
                       help="Run quick demo without external repositories")
    parser.add_argument("--headless", action="store_true", 
                       help="Run headless demo for performance testing")
    parser.add_argument("--duration", type=float, default=30.0,
                       help="Demo duration in seconds")
    
    args = parser.parse_args()
    
    print("🚀 SimPyROS All Features Integration Demo")
    print("Showcasing the complete enhanced simulation system")
    print("=" * 70)
    
    try:
        if args.headless:
            # Force headless environment
            os.environ['PYVISTA_OFF_SCREEN'] = 'true'
            success = headless_demo(args.duration)
        elif args.quick:
            success = quick_demo(args.duration)
        else:
            success = comprehensive_demo(args.duration)
        
        if success:
            print("\n🎉 All Features Demo Completed Successfully!")
            print("\n✅ Demonstrated features:")
            print("   📦 Simplified simulation management")
            print("   🤖 Multi-robot coordination") 
            print("   🔗 Robot link connections")
            print("   📡 Smart sensor integration")
            print("   🎨 Advanced 3D visualization")
            print("   ⚡ High-performance updates")
            print("\n🎯 The new SimPyROS system reduces code complexity")
            print("   from ~100+ lines to ~20 lines for basic simulations!")
            return 0
        else:
            print("\n❌ Demo failed!")
            return 1
            
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted by user")
        return 0
    except Exception as e:
        print(f"\n❌ Demo failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())