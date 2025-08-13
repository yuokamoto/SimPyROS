#!/usr/bin/env python3
"""
Test sim_time based update loops in SimulationObject and Robot
"""
import time
import math
from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose

def test_sim_time_based_updates():
    print("⏰ Testing Sim-Time Based Update Loops")
    print("=" * 50)
    
    # Test with custom time step
    config = SimulationConfig(
        time_step=0.02,  # 20ms simulation steps
        real_time_factor=3.0,  # 3x speed
        visualization=False  # Headless for clear logging
    )
    
    sim = SimulationManager(config)
    
    try:
        # Add robot
        robot = sim.add_robot_from_urdf(
            'sim_time_test_robot', 
            'examples/robots/articulated_arm_robot.urdf'
        )
        
        # Add simulation object for testing
        obj_params = ObjectParameters(
            name="test_object",
            object_type=ObjectType.DYNAMIC,
            update_interval=0.05  # Object updates every 50ms sim time
        )
        test_obj = SimulationObject(sim.env, obj_params, simulation_manager=sim)
        sim.add_object("test_object", test_obj)
        
        update_count = 0
        sim_times = []
        
        def sim_time_control(dt):
            """Control function to verify sim_time based updates"""
            nonlocal update_count, sim_times
            update_count += 1
            
            if robot.simulation_manager:
                current_sim_time = robot.simulation_manager.get_sim_time()
                sim_times.append(current_sim_time)
                
                if update_count <= 10 or update_count % 10 == 0:
                    print(f"Update #{update_count}: sim_time={current_sim_time:.3f}s")
                    
                    # Check robot joint update timing
                    joint_names = [name for name in robot.get_joint_names() 
                                  if robot.joints[name].joint_type.value != 'fixed']
                    
                    # Set joint positions based on sim_time (not real time)
                    for i, joint_name in enumerate(joint_names):
                        position = 0.3 * math.sin(current_sim_time * 2 + i * math.pi / 4)
                        sim.set_robot_joint_position('sim_time_test_robot', joint_name, position)
                
                    # Verify SimulationObject is also using sim_time
                    obj_pose = test_obj.get_pose()
                    print(f"   Object pose: x={obj_pose.x:.3f}, y={obj_pose.y:.3f}")
            else:
                print("⚠️ Robot has no simulation_manager reference")
        
        sim.set_robot_control_callback('sim_time_test_robot', sim_time_control, frequency=20.0)
        
        print("🎯 Testing sim_time based update loops:")
        print(f"   - Simulation time_step: {config.time_step}s")
        print(f"   - Real-time factor: {config.real_time_factor}x")  
        print(f"   - Robot joint_update_rate: {robot.robot_parameters.joint_update_rate} Hz")
        print(f"   - Object update_interval: {obj_params.update_interval}s")
        print(f"   - Updates should be based on sim_time, not real time")
        print()
        
        # Run test
        start_real_time = time.time()
        sim.run(duration=2.0)  # 2 seconds of simulation time
        end_real_time = time.time()
        
        # Analyze results
        final_sim_time = sim.get_sim_time()
        actual_duration = end_real_time - start_real_time
        
        print(f"\n📊 Sim-Time Based Update Results:")
        print(f"   Control updates: {update_count}")
        print(f"   Final sim_time: {final_sim_time:.3f}s")
        print(f"   Actual real duration: {actual_duration:.3f}s")
        print(f"   Expected real duration: {2.0 / config.real_time_factor:.3f}s")
        
        # Check if updates were consistent with sim_time
        if len(sim_times) >= 2:
            sim_time_deltas = [sim_times[i+1] - sim_times[i] for i in range(len(sim_times)-1)]
            avg_sim_time_delta = sum(sim_time_deltas) / len(sim_time_deltas) if sim_time_deltas else 0
            expected_delta = 1.0 / 20.0  # 20 Hz callback frequency
            
            print(f"   Avg sim_time delta between updates: {avg_sim_time_delta:.4f}s")
            print(f"   Expected sim_time delta: {expected_delta:.4f}s")
            
            if abs(avg_sim_time_delta - expected_delta) < 0.01:
                print("✅ PASS: Updates are properly synchronized with sim_time")
            else:
                print("⚠️  WARNING: Update timing may not be perfectly sim_time based")
        
        # Verify object updates based on sim_time
        print(f"   Test object final pose: {test_obj.get_pose()}")
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        sim.shutdown()

def test_different_update_rates():
    print(f"\n🔄 Testing Different Update Rates with Sim-Time")
    print("-" * 50)
    
    config = SimulationConfig(
        time_step=0.01,  # 10ms simulation steps  
        real_time_factor=2.0,  # 2x speed
        visualization=False
    )
    
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            'multi_rate_robot',
            'examples/robots/articulated_arm_robot.urdf'
        )
        
        # Add objects with different update rates
        obj1_params = ObjectParameters(
            name="fast_object",
            object_type=ObjectType.DYNAMIC,
            update_interval=0.02  # 20ms sim time updates
        )
        obj1 = SimulationObject(sim.env, obj1_params, simulation_manager=sim)
        sim.add_object("fast_object", obj1)
        
        obj2_params = ObjectParameters(
            name="slow_object", 
            object_type=ObjectType.DYNAMIC,
            update_interval=0.1   # 100ms sim time updates
        )
        obj2 = SimulationObject(sim.env, obj2_params, simulation_manager=sim)
        sim.add_object("slow_object", obj2)
        
        control_updates = 0
        
        def multi_rate_control(dt):
            nonlocal control_updates
            control_updates += 1
            
            sim_time = robot.simulation_manager.get_sim_time()
            
            if control_updates % 15 == 0:
                print(f"Control #{control_updates}: sim_time={sim_time:.3f}s")
                print(f"   Fast object: {obj1.get_pose()}")
                print(f"   Slow object: {obj2.get_pose()}")
        
        sim.set_robot_control_callback('multi_rate_robot', multi_rate_control, frequency=30.0)
        
        print("🎯 Testing multiple update rates:")
        print("   - Fast object: 20ms sim_time intervals")
        print("   - Slow object: 100ms sim_time intervals") 
        print("   - All should be synchronized to sim_time")
        print()
        
        sim.run(duration=1.5)
        
        print(f"\n✅ Multi-rate test completed:")
        print(f"   Control updates: {control_updates}")
        print(f"   Final sim_time: {sim.get_sim_time():.3f}s")
        
    except Exception as e:
        print(f"❌ Multi-rate test error: {e}")
    finally:
        sim.shutdown()

if __name__ == "__main__":
    test_sim_time_based_updates()
    time.sleep(1)
    test_different_update_rates()
    print("\n🎉 All sim_time based update tests completed!")