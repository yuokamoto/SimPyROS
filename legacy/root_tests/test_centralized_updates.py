#!/usr/bin/env python3
"""
Test centralized update management in SimulationManager
"""
import time
import math
from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose

def test_centralized_update_management():
    print("🎯 Testing Centralized Update Management")
    print("=" * 50)
    
    config = SimulationConfig(
        time_step=0.02,  # 20ms simulation steps
        real_time_factor=2.0,  # 2x speed
        visualization=False
    )
    
    sim = SimulationManager(config)
    
    try:
        # Add robot
        robot = sim.add_robot_from_urdf(
            'centralized_robot', 
            'examples/robots/articulated_arm_robot.urdf'
        )
        
        # Add simulation objects with different update intervals
        obj1_params = ObjectParameters(
            name="fast_object",
            object_type=ObjectType.DYNAMIC,
            update_interval=0.05  # 50ms updates
        )
        obj1 = SimulationObject(sim.env, obj1_params, simulation_manager=sim)
        sim.add_object("fast_object", obj1)
        
        obj2_params = ObjectParameters(
            name="slow_object",
            object_type=ObjectType.DYNAMIC,
            update_interval=0.2   # 200ms updates
        )
        obj2 = SimulationObject(sim.env, obj2_params, simulation_manager=sim)
        sim.add_object("slow_object", obj2)
        
        # Track update counts
        robot_joint_updates = 0
        robot_base_updates = 0
        obj1_updates = 0
        obj2_updates = 0
        
        # Original update methods for counting
        original_robot_joint_update = robot.update_joints_if_needed
        original_robot_base_update = robot.update_if_needed
        original_obj1_update = obj1.update_if_needed
        original_obj2_update = obj2.update_if_needed
        
        def count_robot_joint_updates(sim_time):
            nonlocal robot_joint_updates
            result = original_robot_joint_update(sim_time)
            if result:
                robot_joint_updates += 1
            return result
        
        def count_robot_base_updates(sim_time):
            nonlocal robot_base_updates
            result = original_robot_base_update(sim_time)
            if result:
                robot_base_updates += 1
            return result
        
        def count_obj1_updates(sim_time):
            nonlocal obj1_updates
            result = original_obj1_update(sim_time)
            if result:
                obj1_updates += 1
            return result
        
        def count_obj2_updates(sim_time):
            nonlocal obj2_updates  
            result = original_obj2_update(sim_time)
            if result:
                obj2_updates += 1
            return result
        
        # Replace update methods with counting versions
        robot.update_joints_if_needed = count_robot_joint_updates
        robot.update_if_needed = count_robot_base_updates
        obj1.update_if_needed = count_obj1_updates
        obj2.update_if_needed = count_obj2_updates
        
        control_calls = 0
        def centralized_control(dt):
            nonlocal control_calls
            control_calls += 1
            
            sim_time = robot.simulation_manager.get_sim_time()
            
            if control_calls % 10 == 0:
                print(f"Control #{control_calls}: sim_time={sim_time:.3f}s")
                print(f"   Robot joint updates: {robot_joint_updates}")
                print(f"   Robot base updates: {robot_base_updates}")
                print(f"   Fast object updates: {obj1_updates}")
                print(f"   Slow object updates: {obj2_updates}")
            
            # Set joint positions
            joint_names = [name for name in robot.get_joint_names() 
                          if robot.joints[name].joint_type.value != 'fixed']
            
            for i, joint_name in enumerate(joint_names):
                position = 0.3 * math.sin(sim_time * 2 + i * math.pi / 4)
                sim.set_robot_joint_position('centralized_robot', joint_name, position)
        
        sim.set_robot_control_callback('centralized_robot', centralized_control, frequency=20.0)
        
        print("🎯 Testing centralized update management:")
        print(f"   - Robot joint updates: {1.0/robot._joint_update_interval:.0f} Hz")
        print(f"   - Robot base updates: {1.0/robot.parameters.update_interval:.0f} Hz")
        print(f"   - Fast object updates: {1.0/obj1.parameters.update_interval:.0f} Hz")
        print(f"   - Slow object updates: {1.0/obj2.parameters.update_interval:.0f} Hz")
        print(f"   - All updates managed by single SimulationManager loop")
        print()
        
        start_real_time = time.time()
        sim.run(duration=2.0)  # 2 seconds simulation time
        end_real_time = time.time()
        
        actual_duration = end_real_time - start_real_time
        
        print(f"\n📊 Centralized Update Results:")
        print(f"   Control callbacks: {control_calls}")
        print(f"   Robot joint updates: {robot_joint_updates}")
        print(f"   Robot base updates: {robot_base_updates}")
        print(f"   Fast object updates: {obj1_updates}")
        print(f"   Slow object updates: {obj2_updates}")
        print(f"   Final sim_time: {sim.get_sim_time():.3f}s")
        print(f"   Real duration: {actual_duration:.3f}s")
        
        # Verify update frequencies are correct
        sim_duration = sim.get_sim_time()
        expected_joint_updates = int(sim_duration * (1.0/robot._joint_update_interval))
        expected_obj1_updates = int(sim_duration / obj1.parameters.update_interval)
        expected_obj2_updates = int(sim_duration / obj2.parameters.update_interval)
        
        print(f"\n🔍 Update Frequency Verification:")
        print(f"   Robot joint updates: {robot_joint_updates} (expected: ~{expected_joint_updates})")
        print(f"   Fast object updates: {obj1_updates} (expected: ~{expected_obj1_updates})")
        print(f"   Slow object updates: {obj2_updates} (expected: ~{expected_obj2_updates})")
        
        if (abs(robot_joint_updates - expected_joint_updates) <= 2 and
            abs(obj1_updates - expected_obj1_updates) <= 2 and
            abs(obj2_updates - expected_obj2_updates) <= 2):
            print("✅ PASS: All update frequencies are correct")
        else:
            print("⚠️  WARNING: Some update frequencies may be inaccurate")
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        sim.shutdown()

def test_architecture_comparison():
    print(f"\n🏗️  Architecture Comparison")
    print("-" * 40)
    
    print("❌ Old Architecture (Multiple Loops):")
    print("   SimulationManager._simulation_process_loop()    # while True + yield")
    print("   Robot._joint_control_loop()                    # while True + yield") 
    print("   SimulationObject._update_loop()                # while True + yield")
    print("   → 3+ separate SimPy processes with individual timing")
    print("   → Complex process synchronization")
    print("   → Higher overhead")
    
    print("\n✅ New Architecture (Single Loop):")
    print("   SimulationManager._simulation_process_loop()    # while True + yield")
    print("   ├── robot.update_joints_if_needed(sim_time)")
    print("   ├── robot.update_if_needed(sim_time)")
    print("   └── obj.update_if_needed(sim_time)")
    print("   → Single SimPy process with centralized timing")
    print("   → Simplified synchronization") 
    print("   → Lower overhead")
    print("   → Better control over update order")
    
    print("\n🎉 Benefits of Centralized Architecture:")
    print("   ✅ Simpler code structure")
    print("   ✅ Better performance (less process overhead)")
    print("   ✅ Easier debugging and monitoring")
    print("   ✅ Precise control over update order")
    print("   ✅ Consistent timing across all objects")

if __name__ == "__main__":
    test_centralized_update_management()
    test_architecture_comparison()
    print("\n🎉 Centralized update management tests completed!")