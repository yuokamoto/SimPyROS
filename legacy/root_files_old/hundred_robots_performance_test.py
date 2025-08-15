#!/usr/bin/env python3
"""
100 Robots Performance Test - SimPyROS

Dedicated test for evaluating SimPyROS performance with 100 simultaneous robots.
Tests both unified event-driven and multi-process architectures.

Usage:
    python hundred_robots_performance_test.py
"""

import sys
import os
import time
import math

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose

def run_hundred_robots_test(unified_process=True, duration=15.0):
    """Run 100 robots performance test"""
    
    print(f"\n{'='*70}")
    print(f"🚀 100 ROBOTS PERFORMANCE TEST")
    print(f"Architecture: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'}")
    print(f"Duration: {duration}s")
    print(f"{'='*70}")
    
    # Create headless simulation for maximum performance
    config = SimulationConfig(
        visualization=False,  # Headless for accurate performance measurement
        update_rate=50.0,     # Base update rate
        real_time_factor=0.0  # Maximum simulation speed
    )
    
    sim = SimulationManager(config)
    
    try:
        print("🏗️ Creating 100 robots...")
        robots = []
        
        # Create robots in different formations
        for i in range(100):
            if i < 50:
                # First 50: 10x5 grid
                x = (i % 10) * 4.0
                y = (i // 10) * 4.0
            else:
                # Second 50: Circular formation
                angle = (i - 50) * 2 * math.pi / 50
                radius = 25.0
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
            
            robot_name = f"robot_{i:03d}"
            
            # Mix different robot types for realistic test
            robot_types = [
                "examples/robots/mobile_robot.urdf",
                "examples/robots/articulated_arm_robot.urdf", 
                "examples/robots/collision_robot.urdf"
            ]
            urdf_path = robot_types[i % 3]
            
            robot = sim.add_robot_from_urdf(
                name=robot_name,
                urdf_path=urdf_path,
                initial_pose=Pose(x=x, y=y, z=0, yaw=i * 0.1),
                joint_update_rate=10.0,  # 10 Hz joint updates
                unified_process=unified_process
            )
            robots.append((robot_name, robot, i))
            
            # Progress indicator
            if (i + 1) % 25 == 0:
                print(f"   ✓ Created {i+1}/100 robots...")
        
        print("✅ All robots created successfully!")
        
        # Performance tracking
        total_callbacks = 0
        total_joint_commands = 0
        total_velocity_commands = 0
        start_time = time.time()
        
        def create_diverse_controller(robot_name, robot_instance, robot_id):
            """Create realistic, diverse control patterns"""
            def controller(dt):
                nonlocal total_callbacks, total_joint_commands, total_velocity_commands
                total_callbacks += 1
                
                t = sim.get_sim_time()
                
                # 5 different behavior patterns
                behavior = robot_id % 5
                
                if behavior == 0:  # Patrol pattern
                    # Move forward, then turn
                    cycle_time = (t + robot_id * 0.1) % 8.0
                    if cycle_time < 5.0:
                        velocity = Velocity(linear_x=0.5, angular_z=0)
                    else:
                        velocity = Velocity(linear_x=0, angular_z=0.5)
                    sim.set_robot_velocity(robot_name, velocity)
                    total_velocity_commands += 1
                    
                elif behavior == 1:  # Complex joint dance
                    joint_names = [name for name in robot_instance.get_joint_names() 
                                  if robot_instance.joints[name].joint_type.value != 'fixed']
                    for j, joint_name in enumerate(joint_names):
                        # Multi-frequency joint motion
                        pos = (0.4 * math.sin(t * 0.7 + j * math.pi/3 + robot_id * 0.05) +
                               0.2 * math.cos(t * 1.3 + j * math.pi/6 + robot_id * 0.03))
                        sim.set_robot_joint_position(robot_name, joint_name, pos)
                        total_joint_commands += 1
                
                elif behavior == 2:  # Figure-8 motion
                    scale = 1.0 + 0.3 * math.sin(robot_id * 0.1)
                    velocity = Velocity(
                        linear_x=0.6 * scale,
                        angular_z=0.4 * math.sin(t * 1.2 + robot_id * 0.1) * scale
                    )
                    sim.set_robot_velocity(robot_name, velocity)
                    total_velocity_commands += 1
                
                elif behavior == 3:  # Oscillating joints with base motion
                    # Base motion
                    if int(t + robot_id * 0.1) % 6 < 4:
                        velocity = Velocity(
                            linear_x=0.3 * math.sin(t * 0.5 + robot_id * 0.1),
                            angular_z=0.2 * math.cos(t * 0.8 + robot_id * 0.1)
                        )
                        sim.set_robot_velocity(robot_name, velocity)
                        total_velocity_commands += 1
                    
                    # Joint motion
                    joint_names = [name for name in robot_instance.get_joint_names() 
                                  if robot_instance.joints[name].joint_type.value != 'fixed']
                    for j, joint_name in enumerate(joint_names[:2]):  # First 2 joints
                        pos = 0.5 * math.sin(t * 1.8 + j * math.pi/2 + robot_id * 0.02)
                        sim.set_robot_joint_position(robot_name, joint_name, pos)
                        total_joint_commands += 1
                
                else:  # Random walk with joint wiggle
                    # Random walk motion
                    random_factor = math.sin(robot_id * 1.234 + t * 0.3)
                    velocity = Velocity(
                        linear_x=0.4 + 0.2 * random_factor,
                        angular_z=0.3 * math.sin(t * 0.6 + robot_id * 0.456)
                    )
                    sim.set_robot_velocity(robot_name, velocity)
                    total_velocity_commands += 1
                    
                    # Joint wiggle
                    joint_names = [name for name in robot_instance.get_joint_names() 
                                  if robot_instance.joints[name].joint_type.value != 'fixed']
                    if len(joint_names) > 0:
                        joint_name = joint_names[robot_id % len(joint_names)]
                        pos = 0.3 * math.sin(t * 2.1 + robot_id * 0.789)
                        sim.set_robot_joint_position(robot_name, joint_name, pos)
                        total_joint_commands += 1
            
            return controller
        
        # Set up varied control frequencies
        print("🎮 Setting up diverse control patterns...")
        for robot_name, robot_instance, robot_id in robots:
            controller = create_diverse_controller(robot_name, robot_instance, robot_id)
            # Varied frequencies from 5-20 Hz
            frequency = 5.0 + (robot_id % 16)
            sim.set_robot_control_callback(robot_name, controller, frequency=frequency)
        
        print(f"🚀 Starting 100-robot stress test...")
        print(f"Architecture comparison:")
        if unified_process:
            print(f"   Unified: 100 processes (1 per robot)")
        else:
            print(f"   Legacy: 400 processes (4 per robot)")
        
        print(f"Running intensive simulation for {duration}s...")
        
        # Record system info if possible
        try:
            import psutil
            process = psutil.Process()
            initial_memory = process.memory_info().rss / 1024 / 1024
            initial_threads = len(process.threads())
            print(f"Initial system state: {initial_memory:.1f}MB, {initial_threads} threads")
        except ImportError:
            initial_memory = 0
            initial_threads = 0
        
        # Run simulation
        sim.run(duration=duration)
        
        # Calculate comprehensive performance metrics
        elapsed_time = time.time() - start_time
        sim_time = sim.get_sim_time()
        
        # Final system state
        try:
            final_memory = process.memory_info().rss / 1024 / 1024
            final_threads = len(process.threads())
        except:
            final_memory = 0
            final_threads = 0
        
        print(f"\n🏆 100-ROBOT PERFORMANCE RESULTS")
        print(f"{'='*50}")
        print(f"Simulation Statistics:")
        print(f"   Simulation time: {sim_time:.2f}s")
        print(f"   Wall clock time: {elapsed_time:.2f}s")
        print(f"   Speed ratio: {sim_time/elapsed_time:.2f}x real-time")
        
        print(f"\nControl Performance:")
        print(f"   Total callbacks: {total_callbacks:,}")
        print(f"   Joint commands: {total_joint_commands:,}")
        print(f"   Velocity commands: {total_velocity_commands:,}")
        print(f"   Avg callback rate: {total_callbacks/elapsed_time:.1f} Hz")
        print(f"   Per-robot rate: {total_callbacks/elapsed_time/100:.1f} Hz")
        
        if initial_memory > 0:
            print(f"\nSystem Resources:")
            print(f"   Memory usage: {initial_memory:.1f} → {final_memory:.1f} MB (Δ{final_memory-initial_memory:+.1f})")
            print(f"   Thread count: {initial_threads} → {final_threads} (Δ{final_threads-initial_threads:+d})")
        
        print(f"\nArchitecture:")
        print(f"   Type: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'}")
        print(f"   Expected processes: {100 if unified_process else 400}")
        
        # Performance assessment
        per_robot_hz = total_callbacks / elapsed_time / 100
        total_commands = total_joint_commands + total_velocity_commands
        
        if per_robot_hz > 12.0:
            rating = "🌟 OUTSTANDING"
        elif per_robot_hz > 8.0:
            rating = "✅ EXCELLENT" 
        elif per_robot_hz > 5.0:
            rating = "✅ GOOD"
        elif per_robot_hz > 3.0:
            rating = "⚠️ FAIR"
        else:
            rating = "❌ POOR"
            
        print(f"\n{rating}: {per_robot_hz:.1f} Hz per robot")
        print(f"Total robot commands: {total_commands:,}")
        
        # Get detailed timing stats
        timing_stats = sim.get_timing_stats()
        if timing_stats:
            print(f"\nTimeManager Details:")
            for key, value in timing_stats.items():
                print(f"   {key}: {value}")
        
        return {
            'unified_process': unified_process,
            'num_robots': 100,
            'simulation_time': sim_time,
            'wall_time': elapsed_time,
            'total_callbacks': total_callbacks,
            'joint_commands': total_joint_commands,
            'velocity_commands': total_velocity_commands,
            'per_robot_hz': per_robot_hz,
            'memory_delta': final_memory - initial_memory if initial_memory > 0 else 0,
            'thread_delta': final_threads - initial_threads if initial_threads > 0 else 0
        }
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return None
        
    finally:
        try:
            sim.shutdown()
        except:
            pass

def compare_architectures():
    """Compare unified vs multi-process with 100 robots"""
    print("🏁 100-Robot Architecture Comparison")
    print("="*80)
    
    results = []
    
    # Test configurations
    tests = [
        ("Unified Event-Driven", True, 15.0),
        ("Multi-Process Legacy", False, 15.0)
    ]
    
    for name, unified, duration in tests:
        print(f"\n⏳ Testing {name}...")
        
        try:
            result = run_hundred_robots_test(
                unified_process=unified,
                duration=duration
            )
            
            if result:
                results.append((name, result))
            
            # Brief pause between tests
            time.sleep(3.0)
            
        except KeyboardInterrupt:
            print("\n⏹️ Tests interrupted by user")
            break
        except Exception as e:
            print(f"❌ {name} test failed: {e}")
    
    # Final comparison
    if len(results) >= 2:
        print(f"\n🏆 FINAL ARCHITECTURE COMPARISON")
        print("="*80)
        
        unified_result = next((r for name, r in results if r['unified_process']), None)
        multi_result = next((r for name, r in results if not r['unified_process']), None)
        
        if unified_result and multi_result:
            perf_improvement = (unified_result['per_robot_hz'] / multi_result['per_robot_hz'] - 1) * 100
            memory_improvement = multi_result['memory_delta'] - unified_result['memory_delta']
            
            print(f"Performance Improvement:")
            print(f"   Per-robot Hz: {perf_improvement:+.1f}% ({multi_result['per_robot_hz']:.1f} → {unified_result['per_robot_hz']:.1f})")
            print(f"   Memory usage: {memory_improvement:+.1f} MB less with unified")
            print(f"   Process reduction: {300} fewer processes (400 → 100)")
            
            if perf_improvement > 30:
                print(f"   🌟 UNIFIED ARCHITECTURE: MAJOR IMPROVEMENT")
            elif perf_improvement > 10:
                print(f"   ✅ UNIFIED ARCHITECTURE: SIGNIFICANT IMPROVEMENT")
            elif perf_improvement > 0:
                print(f"   ✅ UNIFIED ARCHITECTURE: IMPROVEMENT")
            else:
                print(f"   ⚠️ PERFORMANCE: MIXED RESULTS")

def main():
    """Main test function"""
    print("🧪 SimPyROS 100-Robot Performance Test")
    print("Evaluating event-driven architecture scalability")
    
    try:
        compare_architectures()
        
    except KeyboardInterrupt:
        print("\n⏹️ Testing interrupted by user")
    except Exception as e:
        print(f"❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n🎉 100-robot performance testing complete!")

if __name__ == "__main__":
    main()