#!/usr/bin/env python3
"""
Test real-time factor performance and timing accuracy
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig


def test_timing_performance():
    """Test real-time factor accuracy"""
    print("🕐 Testing Real-Time Factor Performance...")
    
    # Test different real-time factors
    test_factors = [0.5, 1.0, 2.0, 5.0]
    duration = 2.0  # Short test duration
    
    for factor in test_factors:
        print(f"\n🧪 Testing real_time_factor={factor}")
        
        config = SimulationConfig(
            real_time_factor=factor,
            visualization=False,  # No visualization for accurate timing
            enable_frequency_grouping=False,
            update_rate=10.0
        )
        
        sim = SimulationManager(config)
        
        try:
            # Add simple robot
            robot = sim.add_robot_from_urdf(
                name="timing_robot",
                urdf_path="examples/robots/mobile_robot.urdf",
                unified_process=False
            )
            
            callback_count = 0
            start_wall_time = time.time()
            
            def timing_callback(dt: float):
                nonlocal callback_count
                callback_count += 1
                
                if callback_count % 10 == 0:
                    sim_time = sim.get_sim_time()
                    wall_time = time.time() - start_wall_time
                    actual_factor = sim_time / wall_time if wall_time > 0 else 0
                    print(f"  Callback #{callback_count}: sim_time={sim_time:.2f}s, wall_time={wall_time:.2f}s, actual_factor={actual_factor:.2f}x")
            
            sim.set_robot_control_callback("timing_robot", timing_callback, frequency=10.0)
            
            print(f"🚀 Running simulation for {duration}s with factor {factor}x...")
            wall_start = time.time()
            sim.run(duration=duration, auto_close=True)
            wall_end = time.time()
            
            # Calculate results
            wall_elapsed = wall_end - wall_start
            sim_time = sim.get_sim_time()
            actual_factor = sim_time / wall_elapsed if wall_elapsed > 0 else 0
            error_percent = abs(actual_factor - factor) / factor * 100 if factor > 0 else 0
            
            print(f"📊 Results for factor {factor}x:")
            print(f"   Expected sim time: {duration:.2f}s")
            print(f"   Actual sim time: {sim_time:.2f}s")
            print(f"   Wall clock time: {wall_elapsed:.2f}s")
            print(f"   Expected factor: {factor:.2f}x")
            print(f"   Actual factor: {actual_factor:.2f}x")
            print(f"   Error: {error_percent:.1f}%")
            print(f"   Total callbacks: {callback_count}")
            
            if error_percent < 5.0:
                print(f"   ✅ GOOD: Error < 5%")
            elif error_percent < 10.0:
                print(f"   ⚠️ FAIR: Error < 10%")
            else:
                print(f"   ❌ POOR: Error > 10%")
                
        except Exception as e:
            print(f"❌ Test failed for factor {factor}: {e}")
            import traceback
            traceback.print_exc()
        finally:
            try:
                sim.shutdown()
            except:
                pass
            
            time.sleep(0.5)  # Brief pause between tests


def test_minimal_timing():
    """Test minimal simulation for timing accuracy"""
    print("\n🔬 Testing Minimal Timing (no callbacks)...")
    
    config = SimulationConfig(
        real_time_factor=1.0,
        visualization=False,
        enable_frequency_grouping=False,
        update_rate=100.0
    )
    
    sim = SimulationManager(config)
    
    try:
        # No robots, just timing
        duration = 3.0
        
        print(f"🚀 Running minimal simulation for {duration}s...")
        wall_start = time.time()
        sim.run(duration=duration, auto_close=True)
        wall_end = time.time()
        
        # Calculate results
        wall_elapsed = wall_end - wall_start
        sim_time = sim.get_sim_time()
        actual_factor = sim_time / wall_elapsed if wall_elapsed > 0 else 0
        error_percent = abs(actual_factor - 1.0) * 100
        
        print(f"📊 Minimal Timing Results:")
        print(f"   Expected sim time: {duration:.2f}s")
        print(f"   Actual sim time: {sim_time:.2f}s")
        print(f"   Wall clock time: {wall_elapsed:.2f}s")
        print(f"   Actual factor: {actual_factor:.2f}x")
        print(f"   Error: {error_percent:.1f}%")
        
        if error_percent < 2.0:
            print(f"   ✅ EXCELLENT: Minimal overhead")
        elif error_percent < 5.0:
            print(f"   ✅ GOOD: Low overhead")
        else:
            print(f"   ⚠️ HIGH OVERHEAD: {error_percent:.1f}%")
        
    except Exception as e:
        print(f"❌ Minimal test failed: {e}")
    finally:
        try:
            sim.shutdown()
        except:
            pass


if __name__ == "__main__":
    test_timing_performance()
    test_minimal_timing()