#!/usr/bin/env python3
"""
Real-time Factor Debugging Test

Tests if unified_process can achieve real_time_factor=1.0 accurately
"""

import sys
import os
import time
import math

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose

def test_realtime_factor(unified_process=True, real_time_factor=1.0, duration=5.0):
    """Test real-time factor accuracy"""
    
    print(f"\n{'='*60}")
    print(f"Real-time Factor Test")
    print(f"Architecture: {'Unified Event-Driven' if unified_process else 'Multi-Process Legacy'}")
    print(f"Target real_time_factor: {real_time_factor}x")
    print(f"Duration: {duration}s")
    print(f"{'='*60}")
    
    # Create headless simulation
    config = SimulationConfig(
        visualization=False,
        update_rate=50.0,  # Reasonable update rate
        real_time_factor=real_time_factor
    )
    
    sim = SimulationManager(config)
    
    try:
        # Add single robot
        robot = sim.add_robot_from_urdf(
            name="test_robot",
            urdf_path="examples/robots/articulated_arm_robot.urdf",
            initial_pose=Pose(x=0, y=0, z=0),
            joint_update_rate=10.0,
            unified_process=unified_process
        )
        
        frame_count = 0
        
        def simple_control_callback(dt):
            nonlocal frame_count
            frame_count += 1
            
            # Light control load - only every 10 frames (1 second at 10Hz)
            if frame_count % 10 == 0:
                t = sim.get_sim_time()  # Use simulation time for real-time factor control
                joint_names = [name for name in robot.get_joint_names() 
                              if robot.joints[name].joint_type.value != 'fixed']
                
                # Simple sinusoidal motion
                for i, joint_name in enumerate(joint_names[:2]):  # Only first 2 joints
                    position = 0.3 * math.sin(t + i * math.pi / 4)
                    sim.set_robot_joint_position("test_robot", joint_name, position)
        
        sim.set_robot_control_callback("test_robot", simple_control_callback, frequency=10.0)
        
        # Record timing
        wall_start_time = time.time()
        
        print(f"\n🚀 Starting simulation...")
        print(f"Target: {duration}s simulation should take {duration/real_time_factor:.2f}s wall time")
        
        # Run simulation
        sim.run(duration=duration)
        
        # Calculate actual timing
        wall_elapsed_time = time.time() - wall_start_time
        sim_time = sim.get_sim_time()
        
        # Get timing stats from time manager
        timing_stats = sim.get_timing_stats()
        
        print(f"\n📊 Timing Results:")
        print(f"   Simulation time: {sim_time:.3f}s")
        print(f"   Wall clock time: {wall_elapsed_time:.3f}s")
        print(f"   Target wall time: {duration/real_time_factor:.3f}s")
        print(f"   Actual real-time factor: {sim_time/wall_elapsed_time:.3f}x")
        print(f"   Target real-time factor: {real_time_factor:.3f}x")
        print(f"   Control callbacks: {frame_count}")
        print(f"   Average callback rate: {frame_count/wall_elapsed_time:.1f} Hz")
        
        # Accuracy calculation
        target_wall_time = duration / real_time_factor
        timing_error = abs(wall_elapsed_time - target_wall_time) / target_wall_time * 100
        actual_factor = sim_time / wall_elapsed_time if wall_elapsed_time > 0 else 0
        factor_error = abs(actual_factor - real_time_factor) / real_time_factor * 100
        
        print(f"\n📈 Accuracy Analysis:")
        print(f"   Wall time error: {timing_error:.1f}%")
        print(f"   Real-time factor error: {factor_error:.1f}%")
        
        if timing_stats:
            print(f"   TimeManager stats:")
            for key, value in timing_stats.items():
                print(f"     {key}: {value}")
        
        # Performance assessment
        if timing_error < 5.0:  # Within 5%
            print(f"   ✅ GOOD: Timing accuracy within 5%")
        elif timing_error < 10.0:  # Within 10%
            print(f"   ⚠️ FAIR: Timing accuracy within 10%")
        else:
            print(f"   ❌ POOR: Timing accuracy >10%")
        
        return {
            'unified_process': unified_process,
            'target_factor': real_time_factor,
            'actual_factor': actual_factor,
            'sim_time': sim_time,
            'wall_time': wall_elapsed_time,
            'timing_error': timing_error,
            'factor_error': factor_error,
            'callback_count': frame_count
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

def compare_realtime_factors():
    """Compare different real-time factors and architectures"""
    print("🕐 Real-time Factor Accuracy Comparison")
    print("=" * 70)
    
    test_configs = [
        (True, 1.0),   # Unified, 1x speed
        (False, 1.0),  # Multi-process, 1x speed  
        (True, 0.5),   # Unified, 0.5x speed
        (True, 2.0),   # Unified, 2x speed
    ]
    
    results = []
    
    for unified, factor in test_configs:
        print(f"\n⏳ Testing: {'unified' if unified else 'multi-process'}, {factor}x factor...")
        
        try:
            result = test_realtime_factor(
                unified_process=unified,
                real_time_factor=factor,
                duration=3.0  # Shorter test for comparison
            )
            if result:
                results.append(result)
                
            # Brief pause between tests
            time.sleep(1.0)
            
        except KeyboardInterrupt:
            print("\\n⏹️ Tests interrupted by user")
            break
        except Exception as e:
            print(f"❌ Test configuration failed: {e}")
    
    # Summary
    print(f"\n🏆 REAL-TIME FACTOR ACCURACY SUMMARY")
    print("=" * 70)
    
    for result in results:
        arch = "Unified" if result['unified_process'] else "Multi-Proc"
        print(f"{arch:10} {result['target_factor']:4.1f}x: "
              f"actual={result['actual_factor']:5.2f}x "
              f"error={result['factor_error']:5.1f}% "
              f"timing_err={result['timing_error']:5.1f}%")
    
    # Find best accuracy for 1.0x
    factor_1_results = [r for r in results if abs(r['target_factor'] - 1.0) < 0.1]
    if factor_1_results:
        best_1x = min(factor_1_results, key=lambda x: x['factor_error'])
        print(f"\n🎯 Best 1.0x accuracy: {'Unified' if best_1x['unified_process'] else 'Multi-Proc'} "
              f"({best_1x['factor_error']:.1f}% error)")

def main():
    """Main test function"""
    print("🧪 SimPyROS Real-time Factor Debugging")
    print("Testing real-time factor accuracy with different configurations")
    
    try:
        compare_realtime_factors()
        
    except KeyboardInterrupt:
        print("\\n⏹️ Testing interrupted by user")
    except Exception as e:
        print(f"❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()
    
    print("\\n✅ Real-time factor debugging complete!")

if __name__ == "__main__":
    main()