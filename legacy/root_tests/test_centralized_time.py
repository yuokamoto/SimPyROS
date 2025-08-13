#!/usr/bin/env python3
"""
Test centralized time management (memo.txt item 10)
"""
import time
import math
from core.simulation_manager import SimulationManager, SimulationConfig

def test_centralized_time():
    print("🕐 Testing Centralized Time Management (memo.txt item 10)")
    print("=" * 60)
    
    # Test normal speed
    config = SimulationConfig(
        real_time_factor=1.0,
        time_step=0.1,  # 100ms steps
        visualization=True
    )
    
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            'time_test_robot', 
            'examples/robots/articulated_arm_robot.urdf'
        )
        
        control_calls = 0
        
        def time_aware_control(dt):
            """Control function that uses centralized time"""
            nonlocal control_calls
            control_calls += 1
            
            # Access time through simulation manager reference
            if robot.simulation_manager:
                sim_time = robot.simulation_manager.get_sim_time()
                real_time = robot.simulation_manager.get_real_time()
                time_step = robot.simulation_manager.get_time_step()
                
                if control_calls % 5 == 0:  # Print every 5th call
                    print(f"Control #{control_calls}: sim_time={sim_time:.2f}s, real_time={real_time:.2f}s, step={time_step:.3f}s")
                
                # Simple joint motion based on simulation time
                joint_names = [name for name in robot.get_joint_names() 
                              if robot.joints[name].joint_type.value != 'fixed']
                
                for i, joint_name in enumerate(joint_names):
                    position = 0.3 * math.sin(sim_time + i * math.pi / 4)
                    sim.set_robot_joint_position('time_test_robot', joint_name, position)
            else:
                print("⚠️ No simulation manager reference available")
        
        sim.set_robot_control_callback('time_test_robot', time_aware_control, frequency=10.0)
        
        print("🎯 Testing centralized time access...")
        print("   - Robot should access sim_time through simulation_manager reference")
        print("   - Visualizer should display sim time, real time, and time step")
        print("   - Time should advance in 0.1s steps")
        print()
        
        sim.run(duration=3.0, auto_close=True)
        
        print(f"\n✅ Test completed with {control_calls} control calls")
        print(f"   Final sim_time: {sim.get_sim_time():.2f}s")
        print(f"   Final real_time: {sim.get_real_time():.2f}s")
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        sim.shutdown()

def test_max_speed():
    print("\n🚀 Testing MAX SPEED Mode (real_time_factor=0.0)")
    print("=" * 60)
    
    # Test max speed mode
    config = SimulationConfig(
        real_time_factor=0.0,  # Max speed
        time_step=0.01,
        visualization=False  # Headless for max speed
    )
    
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            'max_speed_robot', 
            'examples/robots/articulated_arm_robot.urdf'
        )
        
        control_calls = 0
        start_real_time = time.time()
        
        def max_speed_control(dt):
            """Control for max speed test"""
            nonlocal control_calls
            control_calls += 1
            
            if control_calls % 100 == 0:  # Print every 100th call
                sim_time = robot.simulation_manager.get_sim_time()
                real_elapsed = time.time() - start_real_time
                speed_ratio = sim_time / real_elapsed if real_elapsed > 0 else 0
                print(f"Control #{control_calls}: sim_time={sim_time:.2f}s, real={real_elapsed:.2f}s, ratio={speed_ratio:.1f}x")
        
        sim.set_robot_control_callback('max_speed_robot', max_speed_control, frequency=100.0)
        
        print("🎯 Testing max speed mode...")
        print("   - real_time_factor=0.0 should run as fast as possible")
        print("   - Should achieve much higher than 1x real-time ratio")
        print()
        
        sim.run(duration=2.0)  # 2 seconds of sim time
        
        final_real_elapsed = time.time() - start_real_time
        final_sim_time = sim.get_sim_time()
        final_ratio = final_sim_time / final_real_elapsed if final_real_elapsed > 0 else 0
        
        print(f"\n✅ Max speed test completed:")
        print(f"   Control calls: {control_calls}")
        print(f"   Sim time: {final_sim_time:.2f}s")
        print(f"   Real time: {final_real_elapsed:.2f}s")
        print(f"   Speed ratio: {final_ratio:.1f}x")
        
        if final_ratio > 5.0:
            print("🚀 Max speed mode working correctly!")
        else:
            print("⚠️ Max speed mode may not be optimal")
        
    except Exception as e:
        print(f"❌ Max speed test error: {e}")
    finally:
        sim.shutdown()

if __name__ == "__main__":
    test_centralized_time()
    time.sleep(1)  # Brief pause between tests
    test_max_speed()
    print("\n🎉 All centralized time management tests completed!")