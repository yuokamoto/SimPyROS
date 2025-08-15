#!/usr/bin/env python3
"""
Compare unified vs independent robot processes performance
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig


def test_process_architecture(unified_process: bool, description: str):
    """Test robot process architecture performance"""
    print(f"\n🧪 Testing {description}...")
    
    config = SimulationConfig(
        real_time_factor=1.0,
        visualization=False,  # No visualization for pure timing test
        enable_frequency_grouping=False,
        update_rate=100.0  # High update rate to stress test
    )
    
    sim = SimulationManager(config)
    
    try:
        # Add simple robot
        robot = sim.add_robot_from_urdf(
            name="test_robot",
            urdf_path="examples/robots/mobile_robot.urdf",
            unified_process=unified_process
        )
        
        callback_count = 0
        wall_start = time.time()
        
        def test_callback(dt: float):
            nonlocal callback_count
            callback_count += 1
            
            # Simple joint control to make the robot work
            t = sim.get_sim_time()
            angle = 0.5 * (1 + 1.5 * t)  # Dynamic angle
            sim.set_robot_joint_position("test_robot", "base_to_mast", angle)
            
            if callback_count % 20 == 0:
                wall_time = time.time() - wall_start
                actual_factor = t / wall_time if wall_time > 0 else 0
                print(f"  #{callback_count}: sim_time={t:.2f}s, wall_time={wall_time:.2f}s, factor={actual_factor:.2f}x")
        
        sim.set_robot_control_callback("test_robot", test_callback, frequency=20.0)
        
        print(f"🚀 Running for 3 seconds ({description})...")
        wall_start_sim = time.time()
        sim.run(duration=3.0, auto_close=True)
        wall_end_sim = time.time()
        
        # Calculate results
        wall_elapsed = wall_end_sim - wall_start_sim
        sim_time = sim.get_sim_time()
        actual_factor = sim_time / wall_elapsed if wall_elapsed > 0 else 0
        error_percent = abs(actual_factor - 1.0) * 100
        
        print(f"📊 Results for {description}:")
        print(f"   Expected sim time: 3.00s")
        print(f"   Actual sim time: {sim_time:.2f}s")
        print(f"   Wall clock time: {wall_elapsed:.2f}s")
        print(f"   Actual factor: {actual_factor:.2f}x")
        print(f"   Error: {error_percent:.1f}%")
        print(f"   Total callbacks: {callback_count}")
        
        if error_percent < 5.0:
            print(f"   ✅ GOOD: Error < 5%")
        elif error_percent < 10.0:
            print(f"   ⚠️ FAIR: Error < 10%")
        else:
            print(f"   ❌ POOR: Error > 10%")
        
        return actual_factor, error_percent
        
    except Exception as e:
        print(f"❌ Test failed for {description}: {e}")
        import traceback
        traceback.print_exc()
        return 0.0, 100.0
    finally:
        try:
            sim.shutdown()
        except:
            pass
        
        time.sleep(0.5)  # Brief pause between tests


def test_visualization_impact():
    """Test impact of visualization on timing"""
    print(f"\n🧪 Testing Visualization Impact...")
    
    for viz_enabled in [False, True]:
        viz_desc = "WITH Visualization" if viz_enabled else "WITHOUT Visualization"
        print(f"\n📊 Test: {viz_desc}")
        
        config = SimulationConfig(
            real_time_factor=1.0,
            visualization=viz_enabled,
            enable_frequency_grouping=False,
            update_rate=60.0  # Lower rate for visualization
        )
        
        sim = SimulationManager(config)
        
        try:
            robot = sim.add_robot_from_urdf(
                name="viz_robot",
                urdf_path="examples/robots/mobile_robot.urdf",
                unified_process=True  # Use unified process
            )
            
            callback_count = 0
            
            def viz_callback(dt: float):
                nonlocal callback_count
                callback_count += 1
                
                # Simple movement
                t = sim.get_sim_time()
                angle = 0.5 * (1 + 1.5 * t)
                sim.set_robot_joint_position("viz_robot", "base_to_mast", angle)
            
            sim.set_robot_control_callback("viz_robot", viz_callback, frequency=10.0)
            
            print(f"🚀 Running 2s test with visualization={viz_enabled}...")
            wall_start = time.time()
            sim.run(duration=2.0, auto_close=True)
            wall_end = time.time()
            
            # Results
            wall_elapsed = wall_end - wall_start
            sim_time = sim.get_sim_time()
            actual_factor = sim_time / wall_elapsed if wall_elapsed > 0 else 0
            error_percent = abs(actual_factor - 1.0) * 100
            
            print(f"   📊 {viz_desc} Results:")
            print(f"      Sim time: {sim_time:.2f}s")
            print(f"      Wall time: {wall_elapsed:.2f}s")
            print(f"      Factor: {actual_factor:.2f}x")
            print(f"      Error: {error_percent:.1f}%")
            print(f"      Callbacks: {callback_count}")
            
        except Exception as e:
            print(f"❌ Visualization test failed: {e}")
        finally:
            try:
                sim.shutdown()
            except:
                pass
            time.sleep(1.0)  # Longer pause for visualization shutdown


if __name__ == "__main__":
    print("🔬 Robot Process Architecture Performance Comparison")
    print("=" * 60)
    
    # Test 1: Independent Processes
    factor1, error1 = test_process_architecture(
        unified_process=False, 
        description="Independent Processes (Legacy)"
    )
    
    # Test 2: Unified Process
    factor2, error2 = test_process_architecture(
        unified_process=True, 
        description="Unified Process (New)"
    )
    
    # Test 3: Visualization Impact
    test_visualization_impact()
    
    # Summary
    print(f"\n🎯 PERFORMANCE SUMMARY")
    print(f"=" * 40)
    print(f"Independent Processes: {factor1:.2f}x (error: {error1:.1f}%)")
    print(f"Unified Process:       {factor2:.2f}x (error: {error2:.1f}%)")
    
    if error2 < error1:
        improvement = error1 - error2
        print(f"✅ Unified process is {improvement:.1f}% more accurate")
    elif error1 < error2:
        degradation = error2 - error1
        print(f"⚠️ Unified process is {degradation:.1f}% less accurate")
    else:
        print(f"🤝 Both architectures have similar performance")