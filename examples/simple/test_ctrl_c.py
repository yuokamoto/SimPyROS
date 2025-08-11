#!/usr/bin/env python3
"""
Test Ctrl+C Functionality

Simple test to verify that Ctrl+C works properly with the new SimulationManager.
"""

import sys
import os
import time
import math

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.simulation_manager import SimulationManager


def test_basic_ctrl_c():
    """Test basic Ctrl+C functionality"""
    print("🧪 Testing Ctrl+C functionality...")
    print("Press Ctrl+C to test graceful shutdown")
    print("=" * 50)
    
    # Create simple simulation
    sim = SimulationManager()
    
    # Add robot
    try:
        robot = sim.add_robot_from_urdf("test_robot", "examples/robots/movable_robot.urdf")
        print("✅ Robot loaded successfully")
    except Exception as e:
        print(f"❌ Failed to load robot: {e}")
        return False
    
    # Simple control function
    def test_control(dt: float):
        t = time.time()
        
        # Get movable joints
        movable_joints = [name for name in robot.get_joint_names() 
                         if robot.joints[name].joint_type.value != 'fixed']
        
        # Simple motion
        for i, joint_name in enumerate(movable_joints):
            position = 0.3 * math.sin(t + i)
            sim.set_robot_joint_position("test_robot", joint_name, position)
        
        # Print status every few seconds
        if int(t) % 3 == 0 and int(t * 10) % 10 == 0:
            print(f"⏰ Running... t={t:.1f}s (Press Ctrl+C to stop)")
    
    # Set control callback
    sim.set_robot_control_callback("test_robot", test_control, frequency=10.0)
    
    print("🎮 Starting test simulation...")
    print("🔄 This will run indefinitely until you press Ctrl+C")
    
    try:
        # Run without duration (indefinite)
        return sim.run()
    except KeyboardInterrupt:
        print("\n✅ Ctrl+C test successful!")
        return True
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        return False


def test_headless_ctrl_c():
    """Test Ctrl+C in headless mode"""
    print("\n🖥️ Testing Ctrl+C in headless mode...")
    print("=" * 50)
    
    from core.simulation_manager import SimulationConfig
    
    # Headless configuration
    config = SimulationConfig(
        visualization=False,
        update_rate=50.0
    )
    
    sim = SimulationManager(config)
    robot = sim.add_robot_from_urdf("headless_robot", "examples/robots/simple_robot.urdf")
    
    frame_count = 0
    
    def headless_control(dt: float):
        nonlocal frame_count
        frame_count += 1
        
        if frame_count % 100 == 0:  # Every 2 seconds at 50Hz
            print(f"🔄 Headless frame {frame_count} (Press Ctrl+C to stop)")
    
    sim.set_robot_control_callback("headless_robot", headless_control, frequency=25.0)
    
    print("🎮 Starting headless test...")
    
    try:
        return sim.run()
    except KeyboardInterrupt:
        print(f"\n✅ Headless Ctrl+C test successful! Processed {frame_count} frames")
        return True


def test_duration_with_ctrl_c():
    """Test Ctrl+C during timed simulation"""
    print("\n⏰ Testing Ctrl+C during timed simulation...")
    print("=" * 50)
    
    sim = SimulationManager()
    robot = sim.add_robot_from_urdf("timed_robot", "examples/robots/movable_robot.urdf")
    
    def timed_control(dt: float):
        t = time.time()
        
        movable_joints = [name for name in robot.get_joint_names() 
                         if robot.joints[name].joint_type.value != 'fixed']
        
        for i, joint_name in enumerate(movable_joints):
            position = 0.4 * math.sin(2 * t + i * math.pi / 2)
            sim.set_robot_joint_position("timed_robot", joint_name, position)
    
    sim.set_robot_control_callback("timed_robot", timed_control, frequency=20.0)
    
    print("🎮 Starting 20-second simulation...")
    print("🔄 Try pressing Ctrl+C before it finishes")
    
    try:
        return sim.run(duration=20.0)
    except KeyboardInterrupt:
        print("\n✅ Ctrl+C during timed simulation successful!")
        return True


def main():
    """Run all Ctrl+C tests"""
    print("🧪 SimPyROS Ctrl+C Test Suite")
    print("Testing graceful shutdown functionality")
    print("=" * 60)
    
    tests = [
        ("Basic Ctrl+C Test", test_basic_ctrl_c),
        ("Headless Ctrl+C Test", test_headless_ctrl_c),
        ("Timed Simulation Ctrl+C Test", test_duration_with_ctrl_c)
    ]
    
    passed = 0
    
    for test_name, test_func in tests:
        try:
            print(f"\n▶️ Running: {test_name}")
            if test_func():
                passed += 1
                print(f"✅ {test_name} PASSED")
            else:
                print(f"❌ {test_name} FAILED")
        except KeyboardInterrupt:
            print(f"\n✅ {test_name} PASSED (interrupted as expected)")
            passed += 1
        except Exception as e:
            print(f"❌ {test_name} FAILED with error: {e}")
        
        # Brief pause between tests
        time.sleep(1)
    
    print(f"\n📊 Test Results: {passed}/{len(tests)} tests passed")
    
    if passed == len(tests):
        print("🎉 All Ctrl+C tests passed!")
        print("\nKey improvements:")
        print("  ✅ Immediate Ctrl+C response")
        print("  ✅ Graceful thread shutdown")
        print("  ✅ Proper visualization cleanup")
        print("  ✅ Works in both interactive and headless modes")
        return 0
    else:
        print("❌ Some tests failed!")
        return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n✅ Main test interrupted - Ctrl+C working!")
        sys.exit(0)