#!/usr/bin/env python3
"""
Test unified time management in SimulationObject and Robot update functions
"""
import time
import math
from core.simulation_manager import SimulationManager, SimulationConfig

def test_unified_time_updates():
    print("🕐 Testing Unified Time Updates in SimulationObject and Robot")
    print("=" * 65)
    
    # Test with custom time step
    config = SimulationConfig(
        time_step=0.05,  # 50ms steps
        real_time_factor=2.0,  # 2x speed
        visualization=True
    )
    
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            'unified_test_robot', 
            'examples/robots/articulated_arm_robot.urdf'
        )
        
        update_count = 0
        
        def unified_time_control(dt):
            """Control function to test unified time management"""
            nonlocal update_count
            update_count += 1
            
            if robot.simulation_manager:
                sim_time = robot.simulation_manager.get_sim_time()
                time_step = robot.simulation_manager.get_time_step()
                
                if update_count % 10 == 0:
                    print(f"Update #{update_count}: sim_time={sim_time:.3f}s, time_step={time_step:.3f}s, dt={dt:.3f}s")
                
                # Verify that robot's joint control loop uses same time step
                joint_names = [name for name in robot.get_joint_names() 
                              if robot.joints[name].joint_type.value != 'fixed']
                
                for i, joint_name in enumerate(joint_names):
                    # Use sim_time for deterministic motion
                    position = 0.4 * math.sin(sim_time * 2 + i * math.pi / 3)
                    sim.set_robot_joint_position('unified_test_robot', joint_name, position)
            else:
                print("⚠️ Robot has no simulation_manager reference")
        
        sim.set_robot_control_callback('unified_test_robot', unified_time_control, frequency=20.0)
        
        print("🎯 Testing unified time management:")
        print("   - SimulationObject._update_state() should use simulation_manager.get_time_step()")
        print("   - Robot._joint_control_loop() should use simulation_manager.get_time_step()")  
        print("   - All update functions should use same centralized time step")
        print(f"   - Current time_step: {config.time_step}s")
        print(f"   - Real-time factor: {config.real_time_factor}x")
        print()
        
        # Run test
        sim.run(duration=3.0, auto_close=True)
        
        print(f"\n✅ Unified time test completed:")
        print(f"   Control updates: {update_count}")
        print(f"   Final sim_time: {sim.get_sim_time():.3f}s")
        print(f"   Expected sim_time: ~3.0s")
        print(f"   Time accuracy: {abs(sim.get_sim_time() - 3.0):.3f}s difference")
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        sim.shutdown()

def test_dynamic_time_step_change():
    print("\n🔄 Testing Dynamic Time Step Changes")
    print("=" * 50)
    
    config = SimulationConfig(
        time_step=0.1,  # Start with 100ms
        real_time_factor=1.0,
        visualization=False  # Headless for speed
    )
    
    sim = SimulationManager(config)
    
    try:
        robot = sim.add_robot_from_urdf(
            'dynamic_test_robot', 
            'examples/robots/articulated_arm_robot.urdf'
        )
        
        update_count = 0
        
        def dynamic_control(dt):
            nonlocal update_count
            update_count += 1
            
            sim_time = robot.simulation_manager.get_sim_time()
            time_step = robot.simulation_manager.get_time_step()
            
            # Change time step at sim_time=1.0s
            if 0.95 <= sim_time <= 1.05 and time_step > 0.05:
                robot.simulation_manager.set_time_step(0.02)  # Change to 20ms
                print(f"🔄 Changed time step to 0.02s at sim_time={sim_time:.3f}s")
            
            if update_count % 25 == 0:
                print(f"Update #{update_count}: sim_time={sim_time:.3f}s, step={time_step:.3f}s")
        
        sim.set_robot_control_callback('dynamic_test_robot', dynamic_control, frequency=50.0)
        
        print("🎯 Testing dynamic time step changes:")
        print("   - Start with 0.1s time steps")
        print("   - Change to 0.02s time steps at sim_time=1.0s")
        print("   - All update functions should adapt immediately")
        print()
        
        sim.run(duration=2.0)
        
        final_time_step = sim.get_time_step()
        print(f"\n✅ Dynamic time step test completed:")
        print(f"   Control updates: {update_count}")
        print(f"   Final time_step: {final_time_step:.3f}s")
        print(f"   Final sim_time: {sim.get_sim_time():.3f}s")
        
        if final_time_step == 0.02:
            print("🔄 Time step change successful!")
        else:
            print("⚠️ Time step change may not have worked")
        
    except Exception as e:
        print(f"❌ Dynamic test error: {e}")
    finally:
        sim.shutdown()

if __name__ == "__main__":
    test_unified_time_updates()
    time.sleep(1)
    test_dynamic_time_step_change()
    print("\n🎉 All unified time management tests completed!")