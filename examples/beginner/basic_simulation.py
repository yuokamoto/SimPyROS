#!/usr/bin/env python3
"""
Basic Simulation Example using SimulationManager

This example demonstrates the simplified interface for robot simulation.
From ~100 lines of setup code down to ~20 lines!

Usage:
    python basic_simulation.py [options]
    
Options:
    --visualization, --vis       Enable visualization (default: False)
    --visualization-backend      Visualization backend: pyvista, process_separated_pyvista (default: process_separated_pyvista)
    --real-time-factor, --rtf N  Set real-time speed multiplier (default: 1.0)
    --example {simple,mobile,multi,performance,all}  Choose which example to run (default: all)
    --num-robots N               Number of robots for performance demo (default: 10)
    --frequency-grouping         Enable frequency grouping optimization for performance demo
    --enable-monitor             Enable real-time monitor window with simulation statistics

Examples:
    python basic_simulation.py                           # Run all examples headless at 1x speed
    python basic_simulation.py --vis                     # Run all examples with PyVista visualization
    python basic_simulation.py --vis --enable-monitor    # Run with visualization and monitor window
    python basic_simulation.py --vis --visualization-backend pyvista  # Run with standard PyVista
    python basic_simulation.py --vis --visualization-backend pyvista  # Standard PyVista
    python basic_simulation.py --rtf 2.0                 # Run at 2x speed
    python basic_simulation.py --example simple --vis    # Default process-separated PyVista
    python basic_simulation.py --example mobile --rtf 0.5  # Run mobile example at half speed
    python basic_simulation.py --example performance --num-robots 100  # 100 robots performance test
    python basic_simulation.py --example performance --num-robots 50 --frequency-grouping --rtf 5.0  # 50 robots with optimization
"""

import sys
import os
import math
import time
import argparse
import logging

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.simulation_manager import SimulationManager
from core.simulation_object import Velocity, Pose
from core.logger import get_logger, set_log_level, log_debug, log_warning

# Setup logging configuration
DEBUG_LEVEL = os.getenv('SIMPYROS_DEBUG', '1')  # 0=None, 1=Minimal, 2=Verbose
if DEBUG_LEVEL == '0':
    set_log_level('WARNING')  # Suppress most output
elif DEBUG_LEVEL == '1':
    set_log_level('INFO')     # Normal output
elif DEBUG_LEVEL == '2':
    set_log_level('DEBUG')    # Verbose output

logger = get_logger('simpyros.examples')

# DEBUG flag definitions for backward compatibility
DEBUG_VERBOSE = (DEBUG_LEVEL == '2')
DEBUG_MINIMAL = (DEBUG_LEVEL in ['1', '2'])


def simple_control_example(unified_process=True, visualization=False, real_time_factor=1.0, visualization_backend='pyvista', duration=5.0, enable_monitor=False):
    """Example 1: Simple joint control with auto-close
    
    This example demonstrates basic robot joint control using the simplified SimulationManager interface.
    The robot will perform smooth sinusoidal motion on all available movable joints.
    
    Args:
        unified_process: Use unified event-driven process architecture
        visualization: Enable visualization
        real_time_factor: Real-time speed multiplier
        visualization_backend: Visualization backend (pyvista, process_separated_pyvista)
    """
    print("🤖 Simple Control Example")
    print(f"Architecture: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'}")
    print(f"Visualization: {'ON (' + visualization_backend + ')' if visualization else 'OFF'}")
    print(f"Real-time factor: {real_time_factor}x")
    print("=" * 40)
    
    # Use parameters for configuration
    from core.simulation_manager import SimulationConfig
    config = SimulationConfig(
        real_time_factor=real_time_factor,
        visualization=visualization,
        visualization_backend=visualization_backend,
        update_frequency=30.0,  # Optimized update frequency for better real-time performance
        enable_frequency_grouping=False,  # Disable frequency grouping to test individual processes
        enable_monitor=enable_monitor  # Control monitor window creation
    )
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            name="my_robot",
            urdf_path="examples/robots/articulated_arm_robot.urdf",
            unified_process=True  # Use parameter from function call
        )
        
        movable_joints = [name for name in robot.get_joint_names() if robot.joints[name].joint_type.value != 'fixed']
                
        # Callback execution counter
        callback_count = 0
        
        def my_control(dt: float):
            """Simple sinusoidal joint motion with optimized debug output"""
            nonlocal callback_count
            callback_count += 1
            
            t = sim.get_sim_time()
            
            # Apply smooth sinusoidal motion to all movable joints
            if movable_joints:
                for i, joint_name in enumerate(movable_joints):
                    amplitude = 0.8  # Larger amplitude for visibility
                    frequency = 0.5  # Smooth motion frequency
                    phase = i * math.pi / 3  # Phase offset between joints
                    position = amplitude * math.sin(t * frequency + phase)
                    
                    sim.set_robot_joint_position("my_robot", joint_name, position)
        
        sim.set_robot_control_callback("my_robot", my_control, frequency=10.0)
        sim.run(duration=duration, auto_close=True)
        
    except Exception as e:
        log_warning(logger, f"Example error: {e}")
    finally:
        try:
            sim.shutdown()
            # Fast monitor cleanup without excessive waiting
            if hasattr(sim, 'monitor') and sim.monitor:
                logger.debug("Monitor cleanup initiated")
        except:
            pass


def mobile_robot_example(unified_process=True, visualization=False, real_time_factor=1.0, visualization_backend='pyvista', duration=10.0):
    """Example 2: Mobile robot with auto-close
    
    Args:
        unified_process: Use unified event-driven process architecture
        visualization: Enable visualization
        real_time_factor: Real-time speed multiplier
        visualization_backend: Visualization backend (pyvista, process_separated_pyvista)
    """
    print("🚗 Mobile Robot Example")
    print(f"Architecture: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'}")
    print(f"Visualization: {'ON (' + visualization_backend + ')' if visualization else 'OFF'}")
    print(f"Real-time factor: {real_time_factor}x")
    print("=" * 40)
    
    # Use parameters for configuration
    from core.simulation_manager import SimulationConfig
    config = SimulationConfig(
        real_time_factor=real_time_factor,
        visualization=visualization,
        visualization_backend=visualization_backend,
        update_frequency=30.0,  # Optimized update frequency for better real-time performance
        enable_frequency_grouping=False
    )
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            name="mobile_robot", 
            urdf_path="examples/robots/mobile_robot.urdf",
                        unified_process=unified_process  # Use parameter from function call
        )
        
        # Mobile robot callback counter and velocity cache
        mobile_callback_count = 0
        
        # Performance optimization: Pre-cache velocity objects
        velocity_cache = {
            'circular': Velocity(linear_x=0.5, angular_z=0.3),
            'forward': Velocity(linear_x=0.5, angular_z=0.0),
            'turn_left': Velocity(linear_x=0.2, angular_z=0.5),
            'turn_right': Velocity(linear_x=0.2, angular_z=-0.5)
        }
        
        def mobile_control(dt: float):
            """Move robot in circle with optimized velocity handling"""
            nonlocal mobile_callback_count
            mobile_callback_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
                        
            # Use pre-cached velocity object instead of creating new one
            velocity = velocity_cache['circular']
            sim.set_robot_velocity("mobile_robot", velocity)
        
        sim.set_robot_control_callback("mobile_robot", mobile_control, frequency=10.0)
        sim.run(duration=duration, auto_close=True)
        
    except Exception as e:
        log_warning(logger, f"Example error: {e}")
    finally:
        try:
            sim.shutdown()
            # Fast monitor cleanup without excessive waiting
            if hasattr(sim, 'monitor') and sim.monitor:
                logger.debug("Monitor cleanup initiated")
        except:
            pass


def multi_robot_example(unified_process=True, visualization=False, real_time_factor=1.0, visualization_backend='pyvista', duration=1.5):
    """Example 3: Multi-robot with auto-close
    
    Args:
        unified_process: Use unified event-driven process architecture
        visualization: Enable visualization
        real_time_factor: Real-time speed multiplier
        visualization_backend: Visualization backend (pyvista, process_separated_pyvista)
    """
    print("🤖🤖 Multi-Robot Example")  
    print(f"Architecture: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'}")
    print(f"Visualization: {'ON (' + visualization_backend + ')' if visualization else 'OFF'}")
    print(f"Real-time factor: {real_time_factor}x")
    print("=" * 40)
    
    # Use parameters for configuration
    from core.simulation_manager import SimulationConfig
    config = SimulationConfig(
        real_time_factor=real_time_factor,
        visualization=visualization,
        visualization_backend=visualization_backend,
        update_frequency=30.0,  # Optimized update frequency for better real-time performance
        enable_frequency_grouping=False
    )
    sim = SimulationManager(config)
    
    try:
        robot1 = sim.add_robot_from_urdf("robot1", "examples/robots/articulated_arm_robot.urdf", Pose(0, 1, 0, 0, 0, 0), unified_process=False)
        robot2 = sim.add_robot_from_urdf("robot2", "examples/robots/collision_robot.urdf", Pose(0, -1, 0, 0, 0, 0), unified_process=False)
        
        # Multi robot callback counters and joint caching
        robot1_callback_count = 0
        robot2_callback_count = 0
        
        # Performance optimization: Cache movable joint names
        robot1_movable_joints = [name for name in robot1.get_joint_names() 
                                if robot1.joints[name].joint_type.value != 'fixed']
        robot2_movable_joints = [name for name in robot2.get_joint_names() 
                                if robot2.joints[name].joint_type.value != 'fixed']
        
        def control_robot1(dt):
            """Control first robot with optimized joint access"""
            nonlocal robot1_callback_count
            robot1_callback_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
            
            # Use cached joint names instead of querying every time
            for i, joint_name in enumerate(robot1_movable_joints):
                position = 0.3 * math.sin(t * 2 + i)
                sim.set_robot_joint_position("robot1", joint_name, position)
        
        def control_robot2(dt):
            """Control second robot with optimized joint access"""
            nonlocal robot2_callback_count
            robot2_callback_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
                        
            for i, joint_name in enumerate(robot2_movable_joints):
                position = 0.4 * math.cos(t * 1.5 + i * math.pi / 2)
                sim.set_robot_joint_position("robot2", joint_name, position)
        
        sim.set_robot_control_callback("robot1", control_robot1, frequency=10.0)
        sim.set_robot_control_callback("robot2", control_robot2, frequency=10.0)
        
        sim.run(duration=duration, auto_close=True)
        
    except Exception as e:
        log_warning(logger, f"Example error: {e}")
    finally:
        try:
            sim.shutdown()
            # Fast monitor cleanup without excessive waiting
            if hasattr(sim, 'monitor') and sim.monitor:
                logger.debug("Monitor cleanup initiated")
        except:
            pass


def multi_robots_performance_demo(num_robots=100, use_frequency_grouping=True, real_time_factor=10.0, visualization=False, visualization_backend='pyvista', duration=10.0):
    """Example 4: Multi robots performance test optimized for maximum speed verification
    
    This function tests how fast 100 robots can be simulated and calculates the maximum 
    achievable speed factor for robotics simulation benchmarking.
    
    Args:
        num_robots: Number of robots to create (default: 100 for maximum speed test)
        use_frequency_grouping: Enable frequency grouping optimization (default: True for max performance)
        real_time_factor: Real-time speed multiplier (default: 10.0 for high-speed test)
        visualization: Enable visualization (default: False for maximum performance)
        visualization_backend: Visualization backend (pyvista, process_separated_pyvista)
        duration: Simulation duration in seconds (default: 10.0 for comprehensive test)
    """
    logger.info(f"🚀 {num_robots} Robots Maximum Speed Performance Test")
    logger.info(f"Architecture: {'Auto Frequency-Grouped (Optimized)' if use_frequency_grouping else 'Traditional Individual Process'}")
    logger.info(f"Visualization: {'ON (' + visualization_backend + ')' if visualization else 'OFF (Maximum Performance Mode)'}")
    logger.info(f"Target real-time factor: {real_time_factor}x")
    logger.info("=" * 50)
    
    from core.simulation_manager import SimulationManager, SimulationConfig
    import time
    import random
    
    # Create optimized configuration for maximum performance
    config = SimulationConfig(
        visualization=visualization,
        visualization_backend=visualization_backend,
        update_frequency=100.0,    # Maximum update frequency for performance testing
        real_time_factor=real_time_factor,
        enable_frequency_grouping=use_frequency_grouping,  # Enabled by default for optimization
        enable_monitor=True  # Enable monitor for performance demo visibility
    )
    
    # Single SimulationManager handles everything automatically
    sim = SimulationManager(config)
    
    try:
        logger.info(f"🏗️ Creating {num_robots} robots with maximum performance optimization...")
        robots = []
        creation_start_time = time.time()
        
        # Calculate compact grid dimensions for 100+ robots
        grid_size = int(math.ceil(math.sqrt(num_robots)))
        
        # Available robot types for random selection
        robot_types = [
            ("mobile", "examples/robots/mobile_robot.urdf"),
            ("arm", "examples/robots/articulated_arm_robot.urdf")
        ]
        
        # Create robots optimized for maximum performance
        for i in range(num_robots):
            x = (i % grid_size) * 1.5  # Compact spacing for performance
            y = (i // grid_size) * 1.5
            
            robot_name = f"robot_{i:03d}"
            
            # Randomly select robot type for variety
            robot_type, urdf_path = random.choice(robot_types)
            
            # Create robot with optimized settings
            robot = sim.add_robot_from_urdf(
                name=robot_name,
                urdf_path=urdf_path,
                initial_pose=Pose(x=x, y=y, z=0),
                unified_process=True  # Use unified process for maximum performance
            )
            robots.append((robot_name, robot, i, robot_type))
            
            # Progress feedback for large robot counts
            if num_robots >= 50 and (i + 1) % max(10, num_robots//10) == 0:
                elapsed = time.time() - creation_start_time
                rate = (i + 1) / elapsed
                eta = (num_robots - i - 1) / rate if rate > 0 else 0
                logger.info(f"   Created {i+1}/{num_robots} robots ({rate:.1f} robots/sec, ETA: {eta:.1f}s)")
        
        creation_time = time.time() - creation_start_time
        logger.info(f"✅ All {num_robots} robots created in {creation_time:.2f}s ({num_robots/creation_time:.1f} robots/sec)")
        
        # Performance tracking
        total_callbacks = 0
        start_time = time.time()
        
        def create_optimized_controller(robot_name, robot_id, robot_type, robot_instance):
            """Create high-performance controller optimized for 100+ robots with random movement patterns"""
            
            # Pre-cache velocity objects for maximum performance (for mobile robots)
            velocity_cache = {
                'forward_slow': Velocity(linear_x=0.3, angular_z=0),
                'forward_medium': Velocity(linear_x=0.5, angular_z=0),
                'circular_left': Velocity(linear_x=0.4, angular_z=0.3),
                'circular_right': Velocity(linear_x=0.4, angular_z=-0.3),
                'turn_left': Velocity(linear_x=0.2, angular_z=0.4),
                'turn_right': Velocity(linear_x=0.2, angular_z=-0.4),
                'zigzag_left': Velocity(linear_x=0.3, angular_z=0.2),
                'zigzag_right': Velocity(linear_x=0.3, angular_z=-0.2),
                'stop': Velocity(linear_x=0, angular_z=0)
            }
            
            # Get movable joints for arm robots
            movable_joints = []
            if robot_type == "arm":
                movable_joints = [name for name in robot_instance.get_joint_names() 
                                if robot_instance.joints[name].joint_type.value != 'fixed']
            
            def controller(dt):
                nonlocal total_callbacks
                total_callbacks += 1
                
                t = sim.get_sim_time()
                
                # Minimal debug output for performance (only first few robots)
                if robot_id < 2 and total_callbacks <= 3:
                    logger.debug(f"🤖 Robot{robot_id:03d} ({robot_type}) Callback #{total_callbacks}: t={t:.2f}s")
                
                # Random pattern selection with constrained movement to keep robots in view
                time_factor = int(t * 2.0)  # Slower pattern changes for better visibility
                pattern = (robot_id * 3 + time_factor) % 8  # 8 different patterns
                
                if robot_type == "mobile":
                    # Mobile robot movement patterns with reduced speed to stay in camera view
                    if pattern == 0:
                        velocity = velocity_cache['forward_slow']
                    elif pattern == 1:
                        velocity = velocity_cache['circular_left']
                    elif pattern == 2:
                        velocity = velocity_cache['circular_right']
                    elif pattern == 3:
                        velocity = velocity_cache['turn_left']
                    elif pattern == 4:
                        velocity = velocity_cache['turn_right']
                    elif pattern == 5:
                        velocity = velocity_cache['zigzag_left']
                    elif pattern == 6:
                        velocity = velocity_cache['zigzag_right']
                    else:  # pattern == 7
                        velocity = velocity_cache['forward_medium']
                    
                    sim.set_robot_velocity(robot_name, velocity)
                    
                elif robot_type == "arm":
                    # Robot arm joint movement patterns
                    for i, joint_name in enumerate(movable_joints):
                        amplitude = 0.3 + (pattern % 3) * 0.2  # Varying amplitude: 0.3-0.7
                        frequency = 0.3 + (pattern % 4) * 0.1  # Varying frequency: 0.3-0.6
                        phase = i * math.pi / 4 + (pattern % 8) * math.pi / 8  # Varying phase
                        position = amplitude * math.sin(t * frequency + phase)
                        
                        sim.set_robot_joint_position(robot_name, joint_name, position)
            
            return controller
        
        # Set up control callbacks optimized for maximum performance
        logger.info("🎮 Setting up high-performance controllers...")
        setup_start_time = time.time()
        
        for robot_name, robot_instance, robot_id, robot_type in robots:
            # Create optimized controller for maximum performance
            controller = create_optimized_controller(robot_name, robot_id, robot_type, robot_instance)
            
            # Use higher frequency for performance testing
            robot_frequency = 10.0  # Higher frequency for intensive testing
            
            # Set callback with performance optimization
            sim.set_robot_control_callback(robot_name, controller, frequency=robot_frequency)
        
        setup_time = time.time() - setup_start_time
        logger.info(f"✅ All controllers set up in {setup_time:.2f}s")
        
        logger.info(f"🚀 Starting {num_robots}-robot maximum performance simulation...")
        if use_frequency_grouping:
            logger.info("✨ Frequency grouping optimization enabled for maximum performance")
        else:
            logger.info("🔧 Individual processes mode (less optimal for 100+ robots)")
        
        # Run simulation with performance monitoring
        logger.info(f"🏃 Running maximum speed test for {duration}s...")
        actual_start_time = time.time()
        
        sim.run(duration=duration, auto_close=True)
        
        # Calculate comprehensive performance metrics
        actual_elapsed_time = time.time() - actual_start_time
        sim_time = sim.get_sim_time()
        speed_factor_achieved = sim_time / actual_elapsed_time
        
        logger.info(f"\n🏆 {num_robots}-Robot Maximum Performance Results:")
        logger.info("=" * 60)
        logger.info(f"📊 Simulation Metrics:")
        logger.info(f"   Target simulation time: {duration:.2f}s")
        logger.info(f"   Actual simulation time: {sim_time:.2f}s")
        logger.info(f"   Wall clock time: {actual_elapsed_time:.2f}s")
        logger.info(f"   🚀 ACHIEVED SPEED FACTOR: {speed_factor_achieved:.2f}x")
        logger.info(f"   Target speed factor: {real_time_factor:.2f}x")
        logger.info(f"")
        logger.info(f"📈 Callback Performance:")
        logger.info(f"   Total control callbacks: {total_callbacks:,}")
        logger.info(f"   Average callback rate: {total_callbacks/actual_elapsed_time:.1f} Hz")
        logger.info(f"   Per-robot callback rate: {total_callbacks/actual_elapsed_time/num_robots:.2f} Hz")
        logger.info(f"   Expected callbacks: {num_robots * 20.0 * duration:,.0f} (at 20Hz per robot)")
        logger.info(f"   Callback efficiency: {(total_callbacks/(num_robots * 20.0 * duration))*100:.1f}%")
        
        # Detailed architecture performance analysis  
        logger.info(f"🔧 Architecture Analysis:")
        if use_frequency_grouping:
            logger.info(f"   Architecture: Frequency-Grouped Optimization")
            
            # Get detailed frequency grouping statistics
            if hasattr(sim, 'get_frequency_grouping_stats'):
                freq_stats = sim.get_frequency_grouping_stats()
                if freq_stats.get('enabled'):
                    logger.info(f"   Frequency groups created: {freq_stats['total_groups']}")
                    logger.info(f"   Process reduction: {freq_stats['process_reduction_percent']:.1f}%")
                    logger.info(f"   Frequency distribution:")
                    for freq, group_info in freq_stats.get('groups', {}).items():
                        robot_count = group_info.get('robot_count', 0)
                        total_calls = group_info.get('total_calls', 0)
                        logger.info(f"     {freq:5.1f} Hz: {robot_count:3d} robots, {total_calls:,} calls")
        else:
            logger.info(f"   Architecture: Individual Processes ({num_robots} separate processes)")
            
        # System performance analysis
        sim_info = sim.get_simulation_info()
        if 'frequency_grouping' in sim_info and sim_info['frequency_grouping']['enabled']:
            fg_info = sim_info['frequency_grouping']
            logger.info(f"   Actual processes: {fg_info['groups']} (vs {fg_info['total_robots']} without optimization)")
            logger.info(f"   Active frequencies: {fg_info['frequencies']}")
        
        # Timing accuracy analysis
        if hasattr(sim, 'get_timing_stats'):
            timing_stats = sim.get_timing_stats()
            if timing_stats:
                logger.info(f"   📊 TimeManager Performance:")
                for key, value in timing_stats.items():
                    logger.info(f"     {key}: {value}")
        
        
        # Calculate optimization statistics
        process_reduction = 0
        num_processes = num_robots  # Default to traditional count
                
        # Return 
        return 
        
    except Exception as e:
        logger.error(f"❌ {num_robots}-robot maximum performance test failed: {e}")
        import traceback
        traceback.print_exc()
        return {
            'num_robots': num_robots,
            'error': str(e),
            'performance_rating': "❌ FAILED",
            'recommendation': "Check system resources and reduce robot count for debugging."
        }
    finally:
        try:
            sim.shutdown()
            # Fast monitor cleanup without excessive waiting
            if hasattr(sim, 'monitor') and sim.monitor:
                logger.debug("Monitor cleanup initiated")
        except:
            pass



def main():
    """Run all examples with automatic progression"""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='SimPyROS Basic Simulation Examples')
    parser.add_argument('--visualization', '--vis', action='store_true', 
                       help='Enable visualization (default: False)')
    parser.add_argument('--visualization-backend', '--vb', choices=['pyvista', 'process_separated_pyvista'], default='process_separated_pyvista',
                       help='Visualization backend (default: process_separated_pyvista)')
    parser.add_argument('--real-time-factor', '--rtf', type=float, default=1.0,
                       help='Real-time speed multiplier (default: 1.0)')
    parser.add_argument('--example', choices=['simple', 'mobile', 'multi', 'performance', 'all'], default='all',
                       help='Which example to run (default: all)')
    parser.add_argument('--num-robots', type=int, default=10,
                       help='Number of robots for performance demo (default: 10)')
    parser.add_argument('--frequency-grouping', action='store_true',
                       help='Enable frequency grouping optimization for performance demo')
    parser.add_argument('--duration', type=float, default=5.0,
                       help='Simulation duration in seconds (default: 5.0)')
    parser.add_argument('--enable-monitor', action='store_true',
                       help='Enable simulation monitor window (may cause X11 issues)')
    
    args = parser.parse_args()
    
    print("🚀 SimPyROS Event-Driven Architecture Examples")
    print("This demonstrates the new unified event-driven process architecture")
    print(f"Visualization: {'ON (' + args.visualization_backend + ')' if args.visualization else 'OFF'}")
    print(f"Real-time factor: {args.real_time_factor}x")
    print("=" * 60)
        
    # Use command line parameters
    default_visualization = args.visualization
    default_real_time_factor = args.real_time_factor
    default_visualization_backend = args.visualization_backend
    
    # Define all available examples
    all_examples = {
        'simple': ("Simple Robot", lambda: simple_control_example(
            unified_process=True, 
            visualization=default_visualization,
            real_time_factor=default_real_time_factor,
            visualization_backend=default_visualization_backend,
            duration=args.duration,
            enable_monitor=args.enable_monitor
        )),
        'mobile': ("Mobile Robot", lambda: mobile_robot_example(
            unified_process=True,
            visualization=default_visualization,
            real_time_factor=default_real_time_factor,
            visualization_backend=default_visualization_backend,
            duration=args.duration
        )),
        'multi': ("Multi-Robot", lambda: multi_robot_example(
            unified_process=True,
            visualization=default_visualization,
            real_time_factor=default_real_time_factor,
            visualization_backend=default_visualization_backend,
            duration=args.duration
        )),
        'performance': (f"{args.num_robots} Robots Performance Demo", lambda: multi_robots_performance_demo(
            num_robots=args.num_robots,
            use_frequency_grouping=args.frequency_grouping,
            real_time_factor=default_real_time_factor,
            visualization=default_visualization,
            visualization_backend=default_visualization_backend,
            duration=args.duration
        )),
    }
    
    # Select examples based on argument
    if args.example == 'all':
        examples = list(all_examples.values())
    else:
        examples = [all_examples[args.example]]
    
    for i, (name, func) in enumerate(examples):
        try:
            print(f"\n▶️ Running: {name} ({i+1}/{len(examples)})")
            
            func()
            
            print(f"✅ {name} example completed.")
            time.sleep(0.1)  # Minimal pause for log output
                
            # Force garbage collection between examples
            import gc
            gc.collect()
            
        except KeyboardInterrupt:
            print(f"\n⏹️ {name} interrupted by user")
            break
        except Exception as e:
            print(f"❌ {name} failed: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n🎉 All examples completed!")
    
    # Final cleanup to ensure no hanging processes
    print("\n🧹 Performing final cleanup...")
    try:
        from core.multiprocessing_cleanup import cleanup_multiprocessing_resources
        cleanup_multiprocessing_resources()
        print("✅ Final cleanup completed")
    except Exception as e:
        print(f"⚠️ Final cleanup warning: {e}")


if __name__ == "__main__":
    main()