#!/usr/bin/env python3
"""
Test auto progression between examples
"""
import time
import math
from core.simulation_manager import SimulationManager
from core.simulation_object import Velocity, Pose

def test_auto_progression():
    print("🧪 Testing Auto Progression Between Examples")
    print("=" * 50)
    
    # Create a single simulation manager
    sim = SimulationManager()
    
    # Test examples with shorter durations
    test_examples = [
        ("Test 1: Simple Joint Motion", lambda: test_simple_joints(sim)),
        ("Test 2: Mobile Robot", lambda: test_mobile_robot(sim)),
    ]
    
    for i, (name, func) in enumerate(test_examples):
        try:
            print(f"\n▶️ Running: {name} ({i+1}/{len(test_examples)})")
            
            # Reset before each test (except first)
            if i > 0:
                print("🔄 Resetting simulation for next test...")
                sim.reset_simulation()
                time.sleep(1.0)
            
            func()
            print(f"✅ {name} completed. Next test starting soon...")
            time.sleep(1.0)
            
        except Exception as e:
            print(f"❌ {name} failed: {e}")
            break
    
    # Clean shutdown
    sim.shutdown()
    print("\n🎉 Auto progression test completed!")

def test_simple_joints(sim):
    """Test simple joint motion with auto-close"""
    print("🤖 Testing simple joint motion (4 seconds)")
    
    robot = sim.add_robot_from_urdf(
        'test_robot', 
        'examples/robots/articulated_arm_robot.urdf'
    )
    
    def control(dt):
        t = time.time()
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        for i, joint_name in enumerate(joint_names):
            position = 0.4 * math.sin(t * 2 + i * math.pi / 4)
            sim.set_robot_joint_position('test_robot', joint_name, position)
    
    sim.set_robot_control_callback('test_robot', control, frequency=30.0)
    sim.run(duration=4.0, auto_close=True)  # Should auto-close after 4 seconds

def test_mobile_robot(sim):
    """Test mobile robot motion with auto-close"""
    print("🚗 Testing mobile robot motion (3 seconds)")
    
    robot = sim.add_robot_from_urdf(
        'mobile_robot', 
        'examples/robots/mobile_robot.urdf'
    )
    
    def control(dt):
        t = time.time()
        velocity = Velocity(linear_x=0.3, angular_z=0.5 * math.sin(t))
        sim.set_robot_velocity('mobile_robot', velocity)
    
    sim.set_robot_control_callback('mobile_robot', control, frequency=20.0)
    sim.run(duration=3.0, auto_close=True)  # Should auto-close after 3 seconds

if __name__ == "__main__":
    test_auto_progression()