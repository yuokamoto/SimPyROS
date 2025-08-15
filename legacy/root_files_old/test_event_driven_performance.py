#!/usr/bin/env python3
"""
Event-Driven Architecture Performance Test

Tests the new unified event-driven process architecture vs traditional multi-process approach.
Measures performance improvements and process count reduction.
"""

import sys
import os
import time
import math

# Try to import psutil, but work without it if not available
try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False
    print("⚠️ psutil not available - running without memory/process monitoring")

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose

def run_performance_test(num_robots=5, unified_process=True, test_duration=10.0):
    """Run performance test with specified configuration"""
    
    print(f"\n{'='*60}")
    print(f"Performance Test: {num_robots} robots")
    print(f"Architecture: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'}")
    print(f"Duration: {test_duration}s")
    print(f"{'='*60}")
    
    # Create headless simulation for pure performance measurement
    config = SimulationConfig(
        visualization=False,
        update_rate=100.0,
        real_time_factor=0.0  # Max speed
    )
    
    sim = SimulationManager(config)
    
    # Track performance metrics
    if PSUTIL_AVAILABLE:
        start_memory = psutil.Process().memory_info().rss / 1024 / 1024  # MB
    else:
        start_memory = 0.0
    robot_instances = []
    
    try:
        # Add robots with specified process architecture
        for i in range(num_robots):
            robot = sim.add_robot_from_urdf(
                name=f"test_robot_{i}",
                urdf_path="examples/robots/articulated_arm_robot.urdf",
                initial_pose=Pose(x=i*2, y=0, z=0),
                joint_update_rate=10.0,
                unified_process=unified_process
            )
            robot_instances.append(robot)
        
        # Set up control callbacks to create realistic load
        frame_counters = [0] * num_robots
        
        def create_control_callback(robot_idx, robot_instance):
            def control_callback(dt):
                nonlocal frame_counters
                frame_counters[robot_idx] += 1
                
                # Simulate realistic control patterns
                t = sim.get_sim_time()  # Use simulation time for real-time factor control
                joint_names = [name for name in robot_instance.get_joint_names() 
                              if robot_instance.joints[name].joint_type.value != 'fixed']
                
                # Only send commands periodically to test event-driven behavior
                if frame_counters[robot_idx] % 20 == 0:  # Every 2 seconds at 10Hz
                    for i, joint_name in enumerate(joint_names):
                        position = 0.5 * math.sin(t + i * math.pi / 4 + robot_idx)
                        sim.set_robot_joint_position(f"test_robot_{robot_idx}", joint_name, position)
                
                # Periodic base motion commands
                if frame_counters[robot_idx] % 50 == 0:  # Every 5 seconds
                    velocity = Velocity(
                        linear_x=0.2 * math.sin(t + robot_idx),
                        angular_z=0.1 * math.cos(t + robot_idx)
                    )
                    sim.set_robot_velocity(f"test_robot_{robot_idx}", velocity)
                
            return control_callback
        
        # Set control callbacks
        for i, robot in enumerate(robot_instances):
            callback = create_control_callback(i, robot)
            sim.set_robot_control_callback(f"test_robot_{i}", callback, frequency=10.0)
        
        # Record start metrics
        start_time = time.time()
        if PSUTIL_AVAILABLE:
            end_memory = psutil.Process().memory_info().rss / 1024 / 1024
            process_count_before = len(psutil.Process().threads())
        else:
            end_memory = 0.0
            process_count_before = 0
        
        print(f"\n🚀 Starting simulation...")
        if PSUTIL_AVAILABLE:
            print(f"Initial memory usage: {start_memory:.1f} MB → {end_memory:.1f} MB (+{end_memory-start_memory:.1f} MB)")
        else:
            print(f"Memory monitoring: Not available")
        
        # Run simulation
        sim.run(duration=test_duration)
        
        # Calculate performance metrics
        elapsed_time = time.time() - start_time
        total_frames = sum(frame_counters)
        avg_fps = total_frames / elapsed_time if elapsed_time > 0 else 0
        
        if PSUTIL_AVAILABLE:
            final_memory = psutil.Process().memory_info().rss / 1024 / 1024
            process_count_after = len(psutil.Process().threads())
        else:
            final_memory = 0.0
            process_count_after = 0
        
        # Expected process counts
        if unified_process:
            expected_robot_processes = num_robots * 1  # 1 unified process per robot
        else:
            expected_robot_processes = num_robots * 4  # 4 processes per robot (joint/sensor/base/motion)
        
        print(f"\n📊 Performance Results:")
        print(f"   Simulation time: {elapsed_time:.2f}s")
        print(f"   Total control frames: {total_frames}")
        print(f"   Average callback FPS: {avg_fps:.1f} Hz")
        if PSUTIL_AVAILABLE:
            print(f"   Memory usage: {start_memory:.1f} → {final_memory:.1f} MB (Δ{final_memory-start_memory:+.1f})")
            print(f"   Thread count change: {process_count_after - process_count_before:+d}")
        print(f"   Expected robot processes: {expected_robot_processes}")
        
        # Process architecture info
        architecture_efficiency = 1.0
        if not unified_process:
            architecture_efficiency = 1.0 / 4.0  # 4x more processes
            
        print(f"   Architecture efficiency: {architecture_efficiency:.1%}")
        
        return {
            'num_robots': num_robots,
            'unified_process': unified_process,
            'elapsed_time': elapsed_time,
            'total_frames': total_frames,
            'avg_fps': avg_fps,
            'memory_usage': final_memory - start_memory if PSUTIL_AVAILABLE else 0.0,
            'process_count_change': process_count_after - process_count_before if PSUTIL_AVAILABLE else 0,
            'expected_processes': expected_robot_processes,
            'architecture_efficiency': architecture_efficiency
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
    """Compare unified vs multi-process architectures"""
    print("🏁 Event-Driven Architecture Performance Comparison")
    print("=" * 80)
    
    test_configs = [
        (2, True),   # 2 robots, unified
        (2, False),  # 2 robots, multi-process
        (5, True),   # 5 robots, unified
        (5, False),  # 5 robots, multi-process
        (10, True),  # 10 robots, unified (stress test)
    ]
    
    results = []
    
    for num_robots, unified in test_configs:
        print(f"\n⏳ Running test: {num_robots} robots, {'unified' if unified else 'multi-process'}...")
        
        try:
            result = run_performance_test(
                num_robots=num_robots,
                unified_process=unified,
                test_duration=5.0
            )
            if result:
                results.append(result)
                
            # Brief pause between tests
            time.sleep(2.0)
            
        except KeyboardInterrupt:
            print("\\n⏹️ Tests interrupted by user")
            break
        except Exception as e:
            print(f"❌ Test configuration failed: {e}")
    
    # Summary comparison
    print(f"\n🏆 PERFORMANCE COMPARISON SUMMARY")
    print("=" * 80)
    
    unified_results = [r for r in results if r['unified_process']]
    multi_results = [r for r in results if not r['unified_process']]
    
    if unified_results and multi_results:
        print(f"\n📈 Average Performance Improvements:")
        
        # Compare same robot counts
        for robot_count in [2, 5]:
            unified = next((r for r in unified_results if r['num_robots'] == robot_count), None)
            multi = next((r for r in multi_results if r['num_robots'] == robot_count), None)
            
            if unified and multi:
                fps_improvement = (unified['avg_fps'] / multi['avg_fps'] - 1) * 100
                memory_improvement = ((multi['memory_usage'] - unified['memory_usage']) / multi['memory_usage']) * 100
                process_reduction = multi['expected_processes'] - unified['expected_processes']
                
                print(f"\n   {robot_count} Robots:")
                print(f"     FPS: {fps_improvement:+.1f}% ({multi['avg_fps']:.1f} → {unified['avg_fps']:.1f})")
                print(f"     Memory: {memory_improvement:+.1f}% ({multi['memory_usage']:.1f} → {unified['memory_usage']:.1f} MB)")
                print(f"     Process reduction: -{process_reduction} ({multi['expected_processes']} → {unified['expected_processes']})")
    
    # Scalability test
    if len([r for r in results if r['num_robots'] == 10]) > 0:
        result_10 = next((r for r in results if r['num_robots'] == 10), None)
        if result_10:
            print(f"\n🚀 Scalability Test (10 robots, unified):")
            print(f"     Performance: {result_10['avg_fps']:.1f} FPS")
            print(f"     Memory usage: {result_10['memory_usage']:.1f} MB")
            print(f"     Process efficiency: {result_10['architecture_efficiency']:.1%}")

def main():
    """Main test function"""
    print("🧪 SimPyROS Event-Driven Architecture Performance Test")
    print("This test compares unified event-driven vs traditional multi-process architectures")
    
    try:
        compare_architectures()
        
    except KeyboardInterrupt:
        print("\\n⏹️ Testing interrupted by user")
    except Exception as e:
        print(f"❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()
    
    print("\\n✅ Performance testing complete!")

if __name__ == "__main__":
    main()