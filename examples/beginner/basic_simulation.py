#!/usr/bin/env python3
"""
Basic Simulation Example using SimulationManager

This example demonstrates the simplified interface for robot simulation.
From ~100 lines of setup code down to ~20 lines!

Usage:
    python basic_simulation.py [options]
    
Options:
    --visualization, --vis       Enable visualization (default: False)
    --visualization-backend      Visualization backend: pyvista, meshcat, process_separated_pyvista (default: process_separated_pyvista)
    --real-time-factor, --rtf N  Set real-time speed multiplier (default: 1.0)
    --example {simple,mobile,multi,performance,all}  Choose which example to run (default: all)
    --num-robots N               Number of robots for performance demo (default: 10)
    --frequency-grouping         Enable frequency grouping optimization for performance demo
    --enable-monitor             Enable real-time monitor window with simulation statistics

Examples:
    python basic_simulation.py                           # Run all examples headless at 1x speed
    python basic_simulation.py --vis                     # Run all examples with PyVista visualization
    python basic_simulation.py --vis --enable-monitor    # Run with visualization and monitor window
    python basic_simulation.py --vis --visualization-backend meshcat  # Run with MeshCat web visualization
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
        visualization_backend: Visualization backend (pyvista, meshcat, process_separated_pyvista)
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
        update_rate=30.0,  # Optimized update rate for better real-time performance
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
        
        # Analyze robot joint configuration
        print("🔍 Analyzing robot joints...")
        joint_names = robot.get_joint_names()
        print(f"📋 Total joints: {len(joint_names)}")
        
        movable_joints = []
        for name in joint_names:
            joint = robot.joints[name]
            joint_type = joint.joint_type.value if hasattr(joint.joint_type, 'value') else str(joint.joint_type)
            if joint_type in ['revolute', 'prismatic', 'continuous']:
                movable_joints.append(name)
                print(f"  ✅ {name}: {joint_type} (movable)")
            else:
                print(f"  ⚪ {name}: {joint_type} (fixed)")
        
        print(f"🎯 Found {len(movable_joints)} movable joints: {movable_joints}")
        
        if not movable_joints:
            print("ℹ️  No movable joints found - robot will remain stationary")
            print("    This is normal for simple demonstration robots")
        
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
            # Ensure monitor is fully stopped before continuing
            if hasattr(sim, 'monitor') and sim.monitor:
                print("⏳ Ensuring monitor cleanup...")
                time.sleep(0.5)  # Give monitor time to fully close
        except:
            pass


def mobile_robot_example(unified_process=True, visualization=False, real_time_factor=1.0, visualization_backend='pyvista', duration=10.0):
    """Example 2: Mobile robot with auto-close
    
    Args:
        unified_process: Use unified event-driven process architecture
        visualization: Enable visualization
        real_time_factor: Real-time speed multiplier
        visualization_backend: Visualization backend (pyvista, meshcat, process_separated_pyvista)
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
        update_rate=30.0,  # Optimized update rate for better real-time performance
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
            
            # Optimized debug output - only when debugging enabled
            if DEBUG_VERBOSE and (mobile_callback_count <= 10 or mobile_callback_count % 50 == 0):
                log_debug(logger, f"Mobile Callback #{mobile_callback_count}: t={t:.2f}s, dt={dt:.4f}s")
                if mobile_callback_count <= 3:
                    log_debug(logger, "Using cached velocity objects for performance")
            
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
            # Ensure monitor is fully stopped before continuing
            if hasattr(sim, 'monitor') and sim.monitor:
                print("⏳ Ensuring monitor cleanup...")
                time.sleep(0.5)  # Give monitor time to fully close
        except:
            pass


def multi_robot_example(unified_process=True, visualization=False, real_time_factor=1.0, visualization_backend='pyvista', duration=1.5):
    """Example 3: Multi-robot with auto-close
    
    Args:
        unified_process: Use unified event-driven process architecture
        visualization: Enable visualization
        real_time_factor: Real-time speed multiplier
        visualization_backend: Visualization backend (pyvista, meshcat, process_separated_pyvista)
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
        update_rate=30.0,  # Optimized update rate for better real-time performance
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
            
            # Optimized debug output
            if DEBUG_VERBOSE and (robot1_callback_count <= 5 or robot1_callback_count % 25 == 0):
                log_debug(logger, f"Robot1 Callback #{robot1_callback_count}: t={t:.2f}s, dt={dt:.4f}s")
            
            # Use cached joint names instead of querying every time
            for i, joint_name in enumerate(robot1_movable_joints):
                position = 0.3 * math.sin(t * 2 + i)
                sim.set_robot_joint_position("robot1", joint_name, position)
        
        def control_robot2(dt):
            """Control second robot with optimized joint access"""
            nonlocal robot2_callback_count
            robot2_callback_count += 1
            
            t = sim.get_sim_time()  # Use simulation time for real-time factor control
            
            # Optimized debug output
            if DEBUG_VERBOSE and (robot2_callback_count <= 5 or robot2_callback_count % 25 == 0):
                log_debug(logger, f"Robot2 Callback #{robot2_callback_count}: t={t:.2f}s, dt={dt:.4f}s")
            
            # Use cached joint names instead of querying every time
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
            # Ensure monitor is fully stopped before continuing
            if hasattr(sim, 'monitor') and sim.monitor:
                print("⏳ Ensuring monitor cleanup...")
                time.sleep(0.5)  # Give monitor time to fully close
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
        visualization_backend: Visualization backend (pyvista, meshcat, process_separated_pyvista)
        duration: Simulation duration in seconds (default: 10.0 for comprehensive test)
    """
    logger.info(f"🚀 {num_robots} Robots Maximum Speed Performance Test")
    logger.info(f"Architecture: {'Auto Frequency-Grouped (Optimized)' if use_frequency_grouping else 'Traditional Individual Process'}")
    logger.info(f"Visualization: {'ON (' + visualization_backend + ')' if visualization else 'OFF (Maximum Performance Mode)'}")
    logger.info(f"Target real-time factor: {real_time_factor}x")
    logger.info("=" * 50)
    
    from core.simulation_manager import SimulationManager, SimulationConfig
    import time
    
    # Create optimized configuration for maximum performance
    config = SimulationConfig(
        visualization=visualization,
        visualization_backend=visualization_backend,
        update_rate=100.0,    # Maximum update rate for performance testing
        real_time_factor=real_time_factor,
        enable_frequency_grouping=use_frequency_grouping,  # Enabled by default for optimization
        enable_monitor=False  # Disable monitor for maximum performance
    )
    
    # Single SimulationManager handles everything automatically
    sim = SimulationManager(config)
    
    try:
        logger.info(f"🏗️ Creating {num_robots} robots with maximum performance optimization...")
        robots = []
        creation_start_time = time.time()
        
        # Calculate compact grid dimensions for 100+ robots
        grid_size = int(math.ceil(math.sqrt(num_robots)))
        
        # Create robots optimized for maximum performance
        for i in range(num_robots):
            x = (i % grid_size) * 1.5  # Compact spacing for performance
            y = (i // grid_size) * 1.5
            
            robot_name = f"robot_{i:03d}"
            
            # Use lightweight mobile robot for fastest simulation
            urdf_path = "examples/robots/mobile_robot.urdf"
            
            # Create robot with optimized settings
            robot = sim.add_robot_from_urdf(
                name=robot_name,
                urdf_path=urdf_path,
                initial_pose=Pose(x=x, y=y, z=0),
                unified_process=True  # Use unified process for maximum performance
            )
            robots.append((robot_name, robot, i))
            
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
        
        def create_optimized_controller(robot_name, robot_id):
            """Create high-performance controller optimized for 100+ robots"""
            
            # Pre-cache velocity objects for maximum performance
            velocity_cache = {
                'forward_fast': Velocity(linear_x=1.0, angular_z=0),
                'circular_left': Velocity(linear_x=0.8, angular_z=0.6),
                'circular_right': Velocity(linear_x=0.8, angular_z=-0.6),
                'turn_left': Velocity(linear_x=0.3, angular_z=0.8),
                'turn_right': Velocity(linear_x=0.3, angular_z=-0.8),
                'zigzag_left': Velocity(linear_x=0.6, angular_z=0.4),
                'zigzag_right': Velocity(linear_x=0.6, angular_z=-0.4),
                'stop': Velocity(linear_x=0, angular_z=0)
            }
            
            def controller(dt):
                nonlocal total_callbacks
                total_callbacks += 1
                
                t = sim.get_sim_time()
                
                # Minimal debug output for performance (only first few robots)
                if robot_id < 2 and total_callbacks <= 3:
                    logger.debug(f"🤖 Robot{robot_id:03d} Callback #{total_callbacks}: t={t:.2f}s")
                
                # Ultra-fast pattern selection using bitwise operations for maximum performance
                time_factor = int(t * 3.0)  # Faster pattern changes
                pattern = (robot_id * 3 + time_factor) & 7  # 8 different patterns
                
                # Select movement pattern for maximum variety and performance
                if pattern == 0:
                    velocity = velocity_cache['forward_fast']
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
                    velocity = velocity_cache['forward_fast']  # Keep moving for performance test
                
                sim.set_robot_velocity(robot_name, velocity)
            
            return controller
        
        # Set up control callbacks optimized for maximum performance
        logger.info("🎮 Setting up high-performance controllers...")
        setup_start_time = time.time()
        
        for robot_name, robot_instance, robot_id in robots:
            # Create optimized controller for maximum performance
            controller = create_optimized_controller(robot_name, robot_id)
            
            # Use higher frequency for performance testing
            robot_frequency = 20.0  # Higher frequency for intensive testing
            
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
        logger.info(f"   Speed efficiency: {(speed_factor_achieved/real_time_factor)*100:.1f}%")
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
        
        # Performance rating and recommendations
        callbacks_per_robot_per_sec = total_callbacks / actual_elapsed_time / num_robots
        if speed_factor_achieved > 20.0:
            rating = "🚀 ULTRA-FAST (>20x)"
            recommendation = "Exceptional performance! Try even higher targets."
        elif speed_factor_achieved > 10.0:
            rating = "⚡ EXCELLENT (>10x)"
            recommendation = "Outstanding performance for 100-robot simulation."
        elif speed_factor_achieved > 5.0:
            rating = "✅ VERY GOOD (>5x)"
            recommendation = "Strong performance, consider frequency grouping optimization."
        elif speed_factor_achieved > 2.0:
            rating = "✅ GOOD (>2x)"
            recommendation = "Acceptable performance, monitor system resources."
        elif speed_factor_achieved > 1.0:
            rating = "⚠️ FAIR (>1x)"
            recommendation = "Enable frequency grouping and reduce visualization."
        else:
            rating = "❌ POOR (<1x)"
            recommendation = "Disable visualization, enable optimization, reduce robot count."
        
        logger.info(f"")
        logger.info(f"🏁 FINAL ASSESSMENT: {rating}")
        logger.info(f"💡 Recommendation: {recommendation}")
        logger.info(f"   Per-robot callback efficiency: {callbacks_per_robot_per_sec:.2f} Hz")
        
        # Calculate optimization statistics
        process_reduction = 0
        num_processes = num_robots  # Default to traditional count
        
        if use_frequency_grouping and hasattr(sim, 'get_frequency_grouping_stats'):
            freq_stats = sim.get_frequency_grouping_stats()
            if freq_stats.get('enabled'):
                process_reduction = freq_stats.get('process_reduction_percent', 0)
                num_processes = freq_stats.get('total_groups', num_robots)
        
        # Return comprehensive performance data
        return {
            'num_robots': num_robots,
            'frequency_grouped': use_frequency_grouping,
            'num_processes': num_processes,
            'target_duration': duration,
            'simulation_time': sim_time,
            'wall_time': actual_elapsed_time,
            'speed_factor_achieved': speed_factor_achieved,
            'target_speed_factor': real_time_factor,
            'speed_efficiency_percent': (speed_factor_achieved/real_time_factor)*100,
            'total_callbacks': total_callbacks,
            'avg_callback_rate': total_callbacks/actual_elapsed_time,
            'per_robot_callback_rate': callbacks_per_robot_per_sec,
            'callback_efficiency_percent': (total_callbacks/(num_robots * 20.0 * duration))*100,
            'process_reduction_percent': process_reduction,
            'performance_rating': rating,
            'recommendation': recommendation
        }
        
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
            # Ensure monitor is fully stopped before continuing
            if hasattr(sim, 'monitor') and sim.monitor:
                print("⏳ Ensuring monitor cleanup...")
                time.sleep(0.5)  # Give monitor time to fully close
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
    
    # Performance optimization: Cache joint names for headless mode
    headless_movable_joints = [name for name in robot.get_joint_names() 
                              if robot.joints[name].joint_type.value != 'fixed']
    
    def headless_control(dt):
        """High-frequency control for headless mode with optimizations"""
        nonlocal frame_count
        frame_count += 1
        
        # Optimized debug output
        if DEBUG_MINIMAL and frame_count % 100 == 0:  # Print every second at 100Hz
            log_debug(logger, f"Frame {frame_count}, dt={dt:.4f}s")
        
        # Simple joint motion with cached joint names
        t = sim.get_sim_time()  # Use simulation time for real-time factor control
        
        for i, joint_name in enumerate(headless_movable_joints):
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
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='SimPyROS Basic Simulation Examples')
    parser.add_argument('--visualization', '--vis', action='store_true', 
                       help='Enable visualization (default: False)')
    parser.add_argument('--visualization-backend', '--vb', choices=['pyvista', 'meshcat', 'process_separated_pyvista'], default='process_separated_pyvista',
                       help='Visualization backend (default: process_separated_pyvista)')
    parser.add_argument('--real-time-factor', '--rtf', type=float, default=1.0,
                       help='Real-time speed multiplier (default: 1.0)')
    parser.add_argument('--example', choices=['simple', 'mobile', 'multi', 'performance', 'all'], default='all',
                       help='Which example to run (default: all)')
    parser.add_argument('--num-robots', type=int, default=100,
                       help='Number of robots for performance demo (default: 100)')
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
    
    # Default to 100 robots for maximum performance testing
    robot_count = 100  # Optimized for maximum speed verification
    
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
            
            # Brief pause between examples with enhanced cleanup
            if name != "Headless Mode":
                print(f"✅ {name} example completed. Next example starting soon...")
                # Additional cleanup time for monitor windows
                time.sleep(2.5)
            else:
                time.sleep(0.5)  # Short pause for headless
                
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
    print("\nKey benefits of the new interface:")
    print("  ✅ Automatic environment management") 
    print("  ✅ Built-in visualization integration")
    print("  ✅ Graceful shutdown handling")
    print("  ✅ Multi-robot support")
    print("  ✅ Headless mode support")
    
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