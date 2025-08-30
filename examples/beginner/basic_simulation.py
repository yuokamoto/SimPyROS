#!/usr/bin/env python3
"""
Basic Simulation Example using SimulationManager

(Updated with performance quick wins: logging, vectorized joint updates, offscreen flag.)
"""

import sys
import os
import math
import time
import argparse
import logging
import numpy as np  # Added for vectorization

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.simulation_manager import SimulationManager, SimulationConfig  # centralize import
from core.simulation_object import Velocity, Pose
from core.utils.logger import get_logger, set_log_level, log_debug, log_warning

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


# ============================
# Helper utilities
# ============================
_joint_phase_cache = {}

def _get_joint_phases(robot_name: str, joint_names, base_phase_step: float) -> np.ndarray:
    """Get or build cached joint phase offsets array."""
    cache_key = (robot_name, len(joint_names), base_phase_step)
    arr = _joint_phase_cache.get(cache_key)
    if arr is None:
        arr = np.arange(len(joint_names), dtype=np.float64) * base_phase_step
        _joint_phase_cache[cache_key] = arr
    return arr


def _vectorized_joint_positions(t: float, freq: float, amplitude: float, phases: np.ndarray) -> np.ndarray:
    """Compute vectorized sinusoidal joint positions."""
    base = t * freq
    return amplitude * np.sin(base + phases, dtype=np.float64)

# ============================
# Examples
# ============================

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
    logger.info("[Simple] Starting Simple Control Example")
    logger.info(f"Architecture: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'} | Visualization: {'ON('+visualization_backend+')' if visualization else 'OFF'} | RTF={real_time_factor}x")

    config = SimulationConfig(
        real_time_factor=real_time_factor,
        visualization=visualization,
        visualization_backend=visualization_backend,
        update_frequency=30.0,
        enable_frequency_grouping=False,
        enable_monitor=enable_monitor
    )
    sim = SimulationManager(config)

    try:
        robot = sim.add_robot_from_urdf(
            name="my_robot",
            urdf_path="examples/robots/articulated_arm_robot.urdf",
            unified_process=True
        )
        movable_joints = [name for name in robot.get_joint_names() if robot.joints[name].joint_type.value != 'fixed']

        # Prepare vectorization cache if enough joints
        use_vector = len(movable_joints) >= 4
        if use_vector:
            phases = _get_joint_phases("my_robot", movable_joints, base_phase_step=math.pi/3)

        def my_control(dt: float):
            t = sim.get_sim_time()
            if movable_joints:
                amplitude = 0.8
                freq = 0.5
                if use_vector:
                    positions = _vectorized_joint_positions(t, freq, amplitude, phases)
                    for joint_name, pos in zip(movable_joints, positions):
                        sim.set_robot_joint_position("my_robot", joint_name, float(pos))
                else:  # fallback small joint count
                    for i, joint_name in enumerate(movable_joints):
                        phase = i * math.pi / 3
                        position = amplitude * math.sin(t * freq + phase)
                        sim.set_robot_joint_position("my_robot", joint_name, position)

        sim.set_robot_control_callback("my_robot", my_control, frequency=10.0)
        sim.run(duration=duration, auto_close=True)
    except Exception as e:
        log_warning(logger, f"Example error: {e}")
    finally:
        try:
            sim.shutdown()
        except Exception:
            pass


def mobile_robot_example(unified_process=True, visualization=False, real_time_factor=1.0, visualization_backend='pyvista', duration=10.0):
    """Example 2: Mobile robot with auto-close
    
    Args:
        unified_process: Use unified event-driven process architecture
        visualization: Enable visualization
        real_time_factor: Real-time speed multiplier
        visualization_backend: Visualization backend (pyvista, process_separated_pyvista)
    """
    logger.info("[Mobile] Starting Mobile Robot Example")
    logger.info(f"Architecture: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'} | Visualization: {'ON('+visualization_backend+')' if visualization else 'OFF'} | RTF={real_time_factor}x")

    config = SimulationConfig(
        real_time_factor=real_time_factor,
        visualization=visualization,
        visualization_backend=visualization_backend,
        update_frequency=30.0,
        enable_frequency_grouping=False
    )
    sim = SimulationManager(config)

    try:
        sim.add_robot_from_urdf(
            name="mobile_robot", 
            urdf_path="examples/robots/mobile_robot.urdf",
            unified_process=unified_process
        )
        velocity_cache = {
            'circular': Velocity(linear_x=0.5, angular_z=0.3)
        }
        def mobile_control(dt: float):
            velocity = velocity_cache['circular']
            sim.set_robot_velocity("mobile_robot", velocity)
        sim.set_robot_control_callback("mobile_robot", mobile_control, frequency=10.0)
        sim.run(duration=duration, auto_close=True)
    except Exception as e:
        log_warning(logger, f"Example error: {e}")
    finally:
        try:
            sim.shutdown()
        except Exception:
            pass


def multi_robot_example(unified_process=True, visualization=False, real_time_factor=1.0, visualization_backend='pyvista', duration=1.5):
    """Example 3: Multi-robot with auto-close
    
    Args:
        unified_process: Use unified event-driven process architecture
        visualization: Enable visualization
        real_time_factor: Real-time speed multiplier
        visualization_backend: Visualization backend (pyvista, process_separated_pyvista)
    """
    logger.info("[Multi] Starting Multi-Robot Example")
    logger.info(f"Architecture: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'} | Visualization: {'ON('+visualization_backend+')' if visualization else 'OFF'} | RTF={real_time_factor}x")

    config = SimulationConfig(
        real_time_factor=real_time_factor,
        visualization=visualization,
        visualization_backend=visualization_backend,
        update_frequency=30.0,
        enable_frequency_grouping=False
    )
    sim = SimulationManager(config)

    try:
        robot1 = sim.add_robot_from_urdf("robot1", "examples/robots/articulated_arm_robot.urdf", Pose(0, 1, 0, 0, 0, 0), unified_process=False)
        robot2 = sim.add_robot_from_urdf("robot2", "examples/robots/collision_robot.urdf", Pose(0, -1, 0, 0, 0, 0), unified_process=False)

        robot1_movable = [n for n in robot1.get_joint_names() if robot1.joints[n].joint_type.value != 'fixed']
        robot2_movable = [n for n in robot2.get_joint_names() if robot2.joints[n].joint_type.value != 'fixed']
        use_vec1 = len(robot1_movable) >= 4
        use_vec2 = len(robot2_movable) >= 4
        if use_vec1:
            phases1 = _get_joint_phases("robot1", robot1_movable, base_phase_step=1.0)
        if use_vec2:
            phases2 = _get_joint_phases("robot2", robot2_movable, base_phase_step=math.pi/2)

        def control_robot1(dt):
            t = sim.get_sim_time()
            freq = 2.0
            amp = 0.3
            if use_vec1:
                positions = _vectorized_joint_positions(t, freq, amp, phases1)
                for jn, pos in zip(robot1_movable, positions):
                    sim.set_robot_joint_position("robot1", jn, float(pos))
            else:
                for i, jn in enumerate(robot1_movable):
                    position = amp * math.sin(t * freq + i)
                    sim.set_robot_joint_position("robot1", jn, position)

        def control_robot2(dt):
            t = sim.get_sim_time()
            amp = 0.4
            base_freq = 1.5
            if use_vec2:
                positions = amp * np.cos(t * base_freq + phases2)
                for jn, pos in zip(robot2_movable, positions):
                    sim.set_robot_joint_position("robot2", jn, float(pos))
            else:
                for i, jn in enumerate(robot2_movable):
                    position = amp * math.cos(t * base_freq + i * math.pi / 2)
                    sim.set_robot_joint_position("robot2", jn, position)

        sim.set_robot_control_callback("robot1", control_robot1, frequency=10.0)
        sim.set_robot_control_callback("robot2", control_robot2, frequency=10.0)
        sim.run(duration=duration, auto_close=True)
    except Exception as e:
        log_warning(logger, f"Example error: {e}")
    finally:
        try:
            sim.shutdown()
        except Exception:
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
    parser.add_argument('--visualization', '--vis', action='store_true', help='Enable visualization (default: False)')
    parser.add_argument('--visualization-backend', '--vb', choices=['pyvista', 'process_separated_pyvista'], default='process_separated_pyvista', help='Visualization backend (default: process_separated_pyvista)')
    parser.add_argument('--real-time-factor', '--rtf', type=float, default=1.0, help='Real-time speed multiplier (0.0=max speed)')
    parser.add_argument('--example', choices=['simple', 'mobile', 'multi', 'performance', 'all'], default='all', help='Which example to run (default: all)')
    parser.add_argument('--num-robots', type=int, default=10, help='Number of robots for performance demo (default: 10)')
    parser.add_argument('--frequency-grouping', action='store_true', help='Enable frequency grouping optimization for performance demo')
    parser.add_argument('--duration', type=float, default=5.0, help='Simulation duration in seconds (default: 5.0)')
    parser.add_argument('--enable-monitor', action='store_true', help='Enable simulation monitor window (may cause X11 issues)')
    parser.add_argument('--offscreen', action='store_true', help='Force PyVista offscreen rendering (sets PYVISTA_OFF_SCREEN=1)')
    parser.add_argument('--no-monitor', action='store_true', help='Disable monitor regardless of config')
    args = parser.parse_args()

    if args.offscreen:
        os.environ['PYVISTA_OFF_SCREEN'] = '1'
    if args.no_monitor:
        args.enable_monitor = False

    logger.info("Launching SimPyROS examples suite")
    logger.info(f"Visualization: {'ON('+args.visualization_backend+')' if args.visualization else 'OFF'} | RTF={args.real_time_factor}x")

    # Example selection logic (reuse existing mapping but updated function names)
    default_visualization = args.visualization
    default_real_time_factor = args.real_time_factor
    default_visualization_backend = args.visualization_backend

    examples = []
    # Build mapping (subset reflecting updated functions)
    mapping = {
        'simple': lambda: simple_control_example(True, default_visualization, default_real_time_factor, default_visualization_backend, args.duration, args.enable_monitor),
        'mobile': lambda: mobile_robot_example(True, default_visualization, default_real_time_factor, default_visualization_backend, args.duration),
        'multi': lambda: multi_robot_example(True, default_visualization, default_real_time_factor, default_visualization_backend, args.duration)
    }
    if args.example == 'all':
        examples = [mapping['simple'], mapping['mobile'], mapping['multi']]
    else:
        if args.example in mapping:
            examples = [mapping[args.example]]
        else:
            # fall back to performance example by delegating to existing function (kept in file above)
            from examples.beginner.basic_simulation import multi_robots_performance_demo
            examples = [lambda: multi_robots_performance_demo(
                num_robots=args.num_robots,
                use_frequency_grouping=args.frequency_grouping,
                real_time_factor=default_real_time_factor,
                visualization=default_visualization,
                visualization_backend=default_visualization_backend,
                duration=args.duration
            )]

    for i, func in enumerate(examples):
        logger.info(f"Running example {i+1}/{len(examples)}")
        try:
            func()
            logger.info("Example completed")
            time.sleep(0.05)
        except KeyboardInterrupt:
            logger.warning("Interrupted by user")
            break
        except Exception as e:
            logger.error(f"Example failed: {e}")

    logger.info("All requested examples finished")