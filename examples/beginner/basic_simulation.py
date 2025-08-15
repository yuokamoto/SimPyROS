#!/usr/bin/env python3
"""
Basic Simulation Example using SimulationManager

This example demonstrates the simplified interface for robot simulation.
From ~100 lines of setup code down to ~20 lines!

Usage:
    python basic_simulation.py
"""

import sys
import os
import math
import time

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.simulation_manager import SimulationManager
from core.simulation_object import Velocity, Pose


def simple_control_example(unified_process=True):
    """Example 1: Simple joint control with auto-close"""
    print("🤖 Simple Control Example")
    print(f"Architecture: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'}")
    print("=" * 40)
    
    # Explicitly set real-time factor to 1.0 for accurate timing
    from core.simulation_manager import SimulationConfig
    config = SimulationConfig(
        real_time_factor=1.0,  # Ensure 1:1 real-time synchronization
        visualization=True,  # Fixed: Visualization should now work with callbacks
        enable_frequency_grouping=False  # Disable frequency grouping to test individual processes
    )
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            name="my_robot",
            urdf_path="examples/robots/articulated_arm_robot.urdf",
            unified_process=unified_process  # Use parameter from function call
        )
        
        # Debug: Print joint information first
        print("🔍 Debug: Checking robot joints...")
        joint_names = robot.get_joint_names()
        print(f"📋 All joint names: {joint_names}")
        
        movable_joints = []
        for name in joint_names:
            joint = robot.joints[name]
            joint_type = joint.joint_type.value if hasattr(joint.joint_type, 'value') else str(joint.joint_type)
            print(f"  Joint '{name}': type={joint_type}")
            if joint_type in ['revolute', 'prismatic', 'continuous']:
                movable_joints.append(name)
        
        print(f"✅ Movable joints found: {movable_joints}")
        
        # Callback execution counter
        callback_count = 0
        
        def my_control(dt: float):
            """Simple sinusoidal joint motion with debug output"""
            nonlocal callback_count
            callback_count += 1
            
            t = sim.get_sim_time()
            
            # Print debug info for every callback in first 10 calls, then every 100 calls
            if callback_count <= 10 or callback_count % 100 == 0:
                print(f"🔄 Callback #{callback_count}: t={t:.2f}s, dt={dt:.4f}s")
                if callback_count <= 5:
                    print(f"   Movable joints: {movable_joints}")
            
            # Apply sinusoidal motion to movable joints
            for i, joint_name in enumerate(movable_joints):
                amplitude = 0.8  # Larger amplitude for more visible motion
                frequency = 0.5   # Slower frequency for smoother motion
                phase = i * math.pi / 3  # Phase offset between joints
                position = amplitude * math.sin(t * frequency + phase)
                
                # Debug: Print position for first joint in first few calls
                if callback_count <= 5 and i == 0:
                    print(f"   Setting joint '{joint_name}' to position {position:.3f}")
                
                sim.set_robot_joint_position("my_robot", joint_name, position)
        
        sim.set_robot_control_callback("my_robot", my_control, frequency=10.0)
        sim.run(duration=5.0, auto_close=True)
        
    except Exception as e:
        print(f"⚠️ Example error: {e}")
    finally:
        try:
            sim.shutdown()
        except:
            pass


def mobile_robot_example(unified_process=True):
    """Example 2: Mobile robot with auto-close"""
    print("🚗 Mobile Robot Example")
    print("=" * 40)
    
    # Explicitly set real-time factor to 1.0 for accurate timing
    from core.simulation_manager import SimulationConfig
    config = SimulationConfig(
        real_time_factor=1.0,  # Ensure 1:1 real-time synchronization
        visualization=True,  # Fixed: Visualization should now work with callbacks
        enable_frequency_grouping=False
    )
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            name="mobile_robot", 
            urdf_path="examples/robots/mobile_robot.urdf",
                        unified_process=unified_process  # Use parameter from function call
        )
        
        # Debug: Mobile robot callback counter
        mobile_callback_count = 0
        
        def mobile_control(dt: float):
            """Move robot in circle"""
            nonlocal mobile_callback_count
            mobile_callback_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
            
            # Print debug info for first 10 calls, then every 50 calls
            if mobile_callback_count <= 10 or mobile_callback_count % 50 == 0:
                print(f"🚗 Mobile Callback #{mobile_callback_count}: t={t:.2f}s, dt={dt:.4f}s")
                if mobile_callback_count <= 3:
                    print(f"   Setting velocity: linear_x=0.5, angular_z=0.3")
            
            linear_speed = 0.5
            angular_speed = 0.3
            
            velocity = Velocity(
                linear_x=linear_speed,
                angular_z=angular_speed
            )
            sim.set_robot_velocity("mobile_robot", velocity)
        
        sim.set_robot_control_callback("mobile_robot", mobile_control, frequency=10.0)
        sim.run(duration=10.0, auto_close=True)
        
    except Exception as e:
        print(f"⚠️ Example error: {e}")
    finally:
        try:
            sim.shutdown()
        except:
            pass


def multi_robot_example(unified_process=True):
    """Example 3: Multi-robot with auto-close"""
    print("🤖🤖 Multi-Robot Example")  
    print(f"Architecture: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'}")
    print("=" * 40)
    
    # Explicitly set real-time factor to 1.0 for accurate timing
    from core.simulation_manager import SimulationConfig
    config = SimulationConfig(
        real_time_factor=1.0,  # Ensure 1:1 real-time synchronization
        visualization=True,  # Fixed: Visualization should now work with callbacks
        enable_frequency_grouping=False
    )
    sim = SimulationManager(config)
    
    try:
        robot1 = sim.add_robot_from_urdf("robot1", "examples/robots/articulated_arm_robot.urdf", Pose(0, 1, 0, 0, 0, 0), unified_process=False)
        robot2 = sim.add_robot_from_urdf("robot2", "examples/robots/collision_robot.urdf", Pose(0, -1, 0, 0, 0, 0), unified_process=False)
        
        # Debug: Multi robot callback counters
        robot1_callback_count = 0
        robot2_callback_count = 0
        
        def control_robot1(dt):
            """Control first robot"""
            nonlocal robot1_callback_count
            robot1_callback_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
            
            # Print debug info for first few calls
            if robot1_callback_count <= 5 or robot1_callback_count % 25 == 0:
                print(f"🤖1 Robot1 Callback #{robot1_callback_count}: t={t:.2f}s, dt={dt:.4f}s")
            
            joint_names = [name for name in robot1.get_joint_names() 
                          if robot1.joints[name].joint_type.value != 'fixed']
            
            for i, joint_name in enumerate(joint_names):
                position = 0.3 * math.sin(t * 2 + i)
                sim.set_robot_joint_position("robot1", joint_name, position)
        
        def control_robot2(dt):
            """Control second robot"""
            nonlocal robot2_callback_count
            robot2_callback_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
            
            # Print debug info for first few calls
            if robot2_callback_count <= 5 or robot2_callback_count % 25 == 0:
                print(f"🤖2 Robot2 Callback #{robot2_callback_count}: t={t:.2f}s, dt={dt:.4f}s")
            
            joint_names = [name for name in robot2.get_joint_names() 
                          if robot2.joints[name].joint_type.value != 'fixed']
            
            for i, joint_name in enumerate(joint_names):
                position = 0.4 * math.cos(t * 1.5 + i * math.pi / 2)
                sim.set_robot_joint_position("robot2", joint_name, position)
        
        sim.set_robot_control_callback("robot1", control_robot1, frequency=10.0)
        sim.set_robot_control_callback("robot2", control_robot2, frequency=10.0)
        
        sim.run(duration=1.5, auto_close=True)
        
    except Exception as e:
        print(f"⚠️ Example error: {e}")
    finally:
        try:
            sim.shutdown()
        except:
            pass


def multi_robots_performance_demo(num_robots=10, use_frequency_grouping=False, real_time_factor=1.0, visualization=True):
    """Example 4: Multi robots performance test with automatic frequency grouping"""
    print(f"🚀 {num_robots} Robots Performance Demo")
    print(f"Architecture: {'Auto Frequency-Grouped' if use_frequency_grouping else 'Traditional Individual Process'}")
    print(f"Visualization: {'ON' if visualization else 'OFF (Headless)'}")
    print("=" * 40)
    
    from core.simulation_manager import SimulationManager, SimulationConfig
    import time
    
    # Create configuration with automatic frequency grouping
    config = SimulationConfig(
        visualization=visualization,
        update_rate=10.0,     # Higher update rate for better performance
        real_time_factor=real_time_factor,
        enable_frequency_grouping=use_frequency_grouping  # Auto frequency grouping
    )
    
    # Single SimulationManager handles everything automatically
    sim = SimulationManager(config)
    
    try:
        print(f"🏗️ Creating {num_robots} robots with automatic frequency grouping...")
        robots = []
        
        # Calculate grid dimensions
        grid_size = int(math.ceil(math.sqrt(num_robots)))
        
        # Create robots with varied joint_update_rates for automatic grouping
        for i in range(num_robots):
            x = (i % grid_size) * 2.0  # Tighter spacing for better visualization
            y = (i // grid_size) * 2.0
            
            robot_name = f"robot_{i:03d}"
            
            # Use only one robot type for faster loading
            urdf_path = "examples/robots/mobile_robot.urdf"
            
            # Create robot without joint_update_rate parameter
            robot = sim.add_robot_from_urdf(
                name=robot_name,
                urdf_path=urdf_path,
                initial_pose=Pose(x=x, y=y, z=0),
                unified_process=False  # Use independent processes for reliable callback execution
            )
            robots.append((robot_name, robot, i))
            
            # Progress feedback
            if num_robots > 20 and (i + 1) % max(10, num_robots//5) == 0:
                print(f"   Created {i+1}/{num_robots} robots...")
        
        print(f"✅ All {num_robots} robots created with automatic frequency grouping!")
        
        # Performance tracking
        total_callbacks = 0
        start_time = time.time()
        
        def create_auto_frequency_controller(robot_name, robot_id):
            """Create controller that works with automatic frequency grouping"""
            
            # Pre-cache velocity objects for performance
            velocity_cache = {
                'forward': Velocity(linear_x=0.6, angular_z=0),
                'left': Velocity(linear_x=0.4, angular_z=0.5),
                'right': Velocity(linear_x=0.4, angular_z=-0.5),
                'circular_left': Velocity(linear_x=0.5, angular_z=0.3),
                'circular_right': Velocity(linear_x=0.5, angular_z=-0.3),
                'stop': Velocity(linear_x=0, angular_z=0)
            }
            
            def controller(dt):
                nonlocal total_callbacks
                total_callbacks += 1
                
                t = sim.get_sim_time()
                
                # Debug: Print for first few callbacks of first few robots
                if robot_id < 3 and total_callbacks <= 5:
                    print(f"🤖 Robot{robot_id} Callback #{total_callbacks}: t={t:.2f}s, dt={dt:.4f}s")
                
                # High-performance pattern selection using bit operations
                pattern = (robot_id + int(t * 2.0)) & 7  # Dynamic pattern based on time
                
                # Use pre-cached velocity objects for different movement patterns
                if pattern < 2:  # Forward motion
                    velocity = velocity_cache['forward']
                elif pattern < 4:  # Circular motion
                    velocity = velocity_cache['circular_left'] if robot_id % 2 == 0 else velocity_cache['circular_right']
                elif pattern < 6:  # Turn motion
                    velocity = velocity_cache['left'] if robot_id % 2 == 0 else velocity_cache['right']
                else:  # Stop
                    velocity = velocity_cache['stop']
                
                sim.set_robot_velocity(robot_name, velocity)
            
            return controller
        
        # Set up control callbacks - SimulationManager handles frequency grouping automatically
        print("🎮 Setting up controllers with automatic frequency grouping...")
        
        for robot_name, robot_instance, robot_id in robots:
            # Create controller (frequency grouping handled automatically by SimulationManager)
            controller = create_auto_frequency_controller(robot_name, robot_id)
            
            # Use standard frequency for all robots
            robot_frequency = 10.0
            
            # Set callback - SimulationManager handles frequency grouping automatically
            sim.set_robot_control_callback(robot_name, controller, frequency=robot_frequency)
        
        print(f"🚀 Starting {num_robots}-robot simulation...")
        if use_frequency_grouping:
            print("✨ Automatic frequency grouping enabled - SimulationManager will optimize processes")
        else:
            print("🔧 Traditional individual processes mode")
        
        # Run simulation
        duration = 5.0  # Shorter duration for testing
        print(f"Running for {duration}s...")
        
        sim.run(duration=duration, auto_close=True)
        
        # Calculate performance metrics
        elapsed_time = time.time() - start_time
        sim_time = sim.get_sim_time()
        
        print(f"\n📊 {num_robots}-Robot Performance Results:")
        print(f"   Simulation time: {sim_time:.2f}s")
        print(f"   Wall clock time: {elapsed_time:.2f}s") 
        print(f"   Total control callbacks: {total_callbacks:,}")
        print(f"   Average callback rate: {total_callbacks/elapsed_time:.1f} Hz")
        print(f"   Per-robot avg rate: {total_callbacks/elapsed_time/num_robots:.1f} Hz")
        
        # Architecture-specific information  
        if use_frequency_grouping:
            print(f"   Architecture: Auto Frequency-Grouped")
            
            # Get automatic frequency grouping stats
            if hasattr(sim, 'get_frequency_grouping_stats'):
                freq_stats = sim.get_frequency_grouping_stats()
                if freq_stats.get('enabled'):
                    print(f"   Frequency groups created: {freq_stats['total_groups']}")
                    print(f"   Process reduction: {freq_stats['process_reduction_percent']:.1f}%")
                    print(f"   Frequency distribution:")
                    for freq, group_info in freq_stats.get('groups', {}).items():
                        robot_count = group_info['robot_count']
                        total_calls = group_info['total_calls']
                        print(f"     {freq:5.1f} Hz: {robot_count:3d} robots, {total_calls:,} calls")
        else:
            print(f"   Architecture: Traditional Individual Processes ({num_robots} processes)")
            
        # Show simulation info with frequency grouping details
        sim_info = sim.get_simulation_info()
        if 'frequency_grouping' in sim_info and sim_info['frequency_grouping']['enabled']:
            fg_info = sim_info['frequency_grouping']
            print(f"   Total processes: {fg_info['groups']} (vs {fg_info['total_robots']} traditional)")
            print(f"   Active frequencies: {fg_info['frequencies']}")
        
        # Get timing stats if available
        if hasattr(sim, 'get_timing_stats'):
            timing_stats = sim.get_timing_stats()
            if timing_stats:
                print(f"   TimeManager stats:")
                for key, value in timing_stats.items():
                    print(f"     {key}: {value}")
        
        # Performance assessment
        callbacks_per_robot_per_sec = total_callbacks / elapsed_time / num_robots
        if callbacks_per_robot_per_sec > 15.0:
            rating = "🚀 ULTRA-FAST"
        elif callbacks_per_robot_per_sec > 10.0:
            rating = "⚡ EXCELLENT"
        elif callbacks_per_robot_per_sec > 8.0:
            rating = "✅ VERY GOOD"
        elif callbacks_per_robot_per_sec > 5.0:
            rating = "✅ GOOD"
        elif callbacks_per_robot_per_sec > 2.0:
            rating = "⚠️ FAIR"
        else:
            rating = "❌ POOR"
        
        print(f"   {rating}: {callbacks_per_robot_per_sec:.1f} Hz per robot")
            
        # Calculate process reduction based on actual frequency groups created
        process_reduction = 0
        num_processes = num_robots  # Default to traditional count
        
        if use_frequency_grouping and hasattr(sim, 'get_frequency_grouping_stats'):
            freq_stats = sim.get_frequency_grouping_stats()
            if freq_stats.get('enabled'):
                process_reduction = freq_stats['process_reduction_percent']
                num_processes = freq_stats['total_groups']
        
        return {
            'num_robots': num_robots,
            'frequency_grouped': use_frequency_grouping,
            'num_processes': num_processes,
            'simulation_time': sim_time,
            'wall_time': elapsed_time,
            'total_callbacks': total_callbacks,
            'avg_callback_rate': total_callbacks/elapsed_time,
            'per_robot_rate': callbacks_per_robot_per_sec,
            'process_reduction_percent': process_reduction
        }
        
    except Exception as e:
        print(f"❌ {num_robots}-robot demo failed: {e}")
        import traceback
        traceback.print_exc()
        return None
    finally:
        try:
            sim.shutdown()
        except:
            pass


def headless_example():
    """Example 5: Headless simulation (no visualization)"""
    print("\n🖥️ Headless Example")
    print("=" * 40)
    
    from core.simulation_manager import SimulationConfig
    
    # Create headless configuration
    config = SimulationConfig(
        visualization=False,  # No visualization
        update_rate=100.0     # High update rate
    )
    
    sim = SimulationManager(config)
    robot = sim.add_robot_from_urdf("headless_robot", "examples/robots/articulated_arm_robot.urdf")
    
    frame_count = 0
    
    def headless_control(dt):
        """High-frequency control for headless mode"""
        nonlocal frame_count
        frame_count += 1
        
        if frame_count % 100 == 0:  # Print every second at 100Hz
            print(f"🔄 Frame {frame_count}, dt={dt:.4f}s")
        
        # Simple joint motion
        t = sim.get_sim_time()  # Use simulation time for real-time factor control
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        for i, joint_name in enumerate(joint_names):
            position = 0.2 * math.sin(t * 3 + i)
            sim.set_robot_joint_position("headless_robot", joint_name, position)
    
    sim.set_robot_control_callback("headless_robot", headless_control, frequency=10.0)
    sim.run(duration=5.0)
    
    # Print final statistics
    info = sim.get_simulation_info()
    print(f"📊 Final stats: {info['frame_count']} frames, {info['average_fps']:.1f} FPS")
    print("✅ Headless example completed")


def main():
    """Run all examples with automatic progression"""
    print("🚀 SimPyROS Event-Driven Architecture Examples")
    print("This demonstrates the new unified event-driven process architecture")
    print("=" * 60)
    
    # Configurable robot count for performance testing
    robot_count = 100  # Start with smaller number for testing
    
    # Test both architectures for comparison
    examples = [
        ("Simple Robot", lambda: simple_control_example(unified_process=True)),
        # ("Mobile Robot", lambda: mobile_robot_example(unified_process=True)),
        # ("Multi-Robot", lambda: multi_robot_example(unified_process=True)),
        # (f"{robot_count} Robots Demo", lambda: multi_robots_performance_demo(
        #     num_robots=robot_count, 
        #     use_frequency_grouping=False,  # Use working approach
        #     real_time_factor=10.0,
        #     visualization=True  
        # ))
    ]
    
    for i, (name, func) in enumerate(examples):
        try:
            print(f"\n▶️ Running: {name} ({i+1}/{len(examples)})")
            
            func()
            
            # Brief pause between examples
            if name != "Headless Mode":
                print(f"✅ {name} example completed. Next example starting soon...")
                time.sleep(2.0)
            else:
                time.sleep(0.5)  # Short pause for headless
            
        except KeyboardInterrupt:
            print(f"\n⏹️ {name} interrupted by user")
            break
        except Exception as e:
            print(f"❌ {name} failed: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n🎉 All examples completed!")
    print("\nKey benefits of the new interface:")
    print("  ✅ Automatic environment management") 
    print("  ✅ Built-in visualization integration")
    print("  ✅ Graceful shutdown handling")
    print("  ✅ Multi-robot support")
    print("  ✅ Headless mode support")


if __name__ == "__main__":
    main()