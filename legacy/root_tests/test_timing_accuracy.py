#!/usr/bin/env python3
"""
Test real-time factor timing accuracy to verify processing time compensation
"""
import time
import math
from core.simulation_manager import SimulationManager, SimulationConfig

def test_real_time_factor_accuracy():
    print("⏱️ Testing Real-Time Factor Accuracy")
    print("=" * 50)
    
    # Test with different real-time factors
    test_cases = [
        (1.0, 2.0, "1.0x speed for 2 seconds"),
        (2.0, 1.0, "2.0x speed for 1 second (should complete in 0.5 real seconds)"),
        (0.5, 1.0, "0.5x speed for 1 second (should complete in 2.0 real seconds)")
    ]
    
    for real_time_factor, sim_duration, description in test_cases:
        print(f"\n🧪 Test Case: {description}")
        print("-" * 40)
        
        config = SimulationConfig(
            time_step=0.01,  # 10ms steps
            real_time_factor=real_time_factor,
            visualization=False  # Headless for accurate timing
        )
        
        sim = SimulationManager(config)
        
        try:
            robot = sim.add_robot_from_urdf(
                'timing_test_robot', 
                'examples/robots/articulated_arm_robot.urdf'
            )
            
            update_count = 0
            start_real_time = time.time()
            
            def timing_control(dt):
                nonlocal update_count
                update_count += 1
                
                # Simple joint motion for processing load
                sim_time = robot.simulation_manager.get_sim_time()
                joint_names = [name for name in robot.get_joint_names() 
                              if robot.joints[name].joint_type.value != 'fixed']
                
                for i, joint_name in enumerate(joint_names):
                    position = 0.3 * math.sin(sim_time * 3 + i * math.pi / 4)
                    sim.set_robot_joint_position('timing_test_robot', joint_name, position)
            
            sim.set_robot_control_callback('timing_test_robot', timing_control, frequency=50.0)
            
            print(f"⏳ Starting simulation: {sim_duration}s sim time at {real_time_factor}x factor")
            print(f"   Expected real duration: {sim_duration / real_time_factor:.2f}s")
            
            # Run simulation
            sim.run(duration=sim_duration)
            
            # Measure actual timing
            actual_real_time = time.time() - start_real_time
            expected_real_time = sim_duration / real_time_factor
            accuracy_ratio = expected_real_time / actual_real_time if actual_real_time > 0 else 0
            
            print(f"\n📊 Timing Results:")
            print(f"   Simulation time: {sim.get_sim_time():.3f}s")
            print(f"   Expected real time: {expected_real_time:.3f}s") 
            print(f"   Actual real time: {actual_real_time:.3f}s")
            print(f"   Accuracy ratio: {accuracy_ratio:.3f} (1.0 = perfect)")
            print(f"   Control updates: {update_count}")
            
            # Get timing statistics from SimulationManager
            stats = sim.get_timing_stats()
            if stats['samples'] > 0:
                print(f"   Avg processing time: {stats['avg_processing_time']*1000:.2f}ms")
                print(f"   Avg sleep time: {stats['avg_sleep_time']*1000:.2f}ms")
                print(f"   Target real dt: {stats['target_real_dt']*1000:.2f}ms")
                print(f"   Actual avg dt: {stats['actual_avg_dt']*1000:.2f}ms")
            
            # Evaluate accuracy
            if 0.95 <= accuracy_ratio <= 1.05:
                print("✅ PASS: Timing accuracy within 5%")
            elif 0.90 <= accuracy_ratio <= 1.10:
                print("⚠️  ACCEPTABLE: Timing accuracy within 10%")
            else:
                print("❌ FAIL: Timing accuracy outside acceptable range")
                
        except Exception as e:
            print(f"❌ Test error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            sim.shutdown()
            time.sleep(0.5)  # Brief pause between tests

def test_max_speed_mode():
    print(f"\n🚀 Testing Maximum Speed Mode (real_time_factor=0.0)")
    print("-" * 50)
    
    config = SimulationConfig(
        time_step=0.01,
        real_time_factor=0.0,  # Max speed
        visualization=False
    )
    
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            'max_speed_robot', 
            'examples/robots/articulated_arm_robot.urdf'
        )
        
        update_count = 0
        start_real_time = time.time()
        
        def max_speed_control(dt):
            nonlocal update_count
            update_count += 1
            
            # Processing intensive joint control
            sim_time = robot.simulation_manager.get_sim_time()
            joint_names = [name for name in robot.get_joint_names() 
                          if robot.joints[name].joint_type.value != 'fixed']
            
            for i, joint_name in enumerate(joint_names):
                position = 0.4 * math.sin(sim_time * 5 + i * math.pi / 3) * math.cos(sim_time * 2)
                sim.set_robot_joint_position('max_speed_robot', joint_name, position)
        
        sim.set_robot_control_callback('max_speed_robot', max_speed_control, frequency=100.0)
        
        sim_duration = 5.0  # 5 seconds of simulation time
        print(f"⚡ Running {sim_duration}s simulation at maximum speed...")
        
        sim.run(duration=sim_duration)
        
        actual_real_time = time.time() - start_real_time
        speed_ratio = sim_duration / actual_real_time if actual_real_time > 0 else 0
        
        print(f"\n🏃 Maximum Speed Results:")
        print(f"   Simulation time: {sim.get_sim_time():.3f}s")
        print(f"   Real time taken: {actual_real_time:.3f}s")
        print(f"   Speed ratio: {speed_ratio:.1f}x faster than real-time")
        print(f"   Control updates: {update_count}")
        print(f"   Update rate achieved: {update_count / actual_real_time:.1f} Hz")
        
        if speed_ratio > 5.0:
            print("✅ EXCELLENT: Achieved >5x real-time speed")
        elif speed_ratio > 2.0:
            print("✅ GOOD: Achieved >2x real-time speed")
        else:
            print("⚠️  SLOW: Max speed mode not significantly faster")
            
    except Exception as e:
        print(f"❌ Max speed test error: {e}")
    finally:
        sim.shutdown()

if __name__ == "__main__":
    test_real_time_factor_accuracy()
    test_max_speed_mode()
    print("\n🎉 All timing accuracy tests completed!")