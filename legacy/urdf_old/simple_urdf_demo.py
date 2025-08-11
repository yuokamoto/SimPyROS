#!/usr/bin/env python3
"""
Simple URDF Demo - Quick example of the new URDFRobotVisualizer library
Shows how easy it is to load -> control -> visualize robots
"""

import sys
import os
import time
import math

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import simpy
from robot import create_robot_from_urdf
from pyvista_visualizer import create_urdf_robot_visualizer, quick_robot_demo


def simple_library_demo():
    """Demonstrate how simple the new library makes robot visualization"""
    
    print("🚀 Simple URDF Library Demo")
    print("=" * 50)
    print("This demo shows how easy it is to:")
    print("  1. Load a robot from URDF")
    print("  2. Set joint commands")
    print("  3. See the results in 3D")
    print("=" * 50)
    
    # Step 1: Create simulation and robot (same as before)
    env = simpy.Environment()
    urdf_path = "examples/robots/movable_robot.urdf"
    
    robot = create_robot_from_urdf(env, urdf_path, "demo_robot")
    print("✅ Step 1: Robot created from URDF")
    
    # Step 2: Create visualizer and load robot (NEW - one line!)
    visualizer = create_urdf_robot_visualizer()
    visualizer.load_robot("my_robot", robot, urdf_path)
    print("✅ Step 2: Robot loaded into visualizer")
    
    # Step 3: Get robot info (NEW - simple interface)
    robot_info = visualizer.get_robot_info("my_robot")
    movable_joints = robot_info['movable_joints']
    print(f"✅ Step 3: Found movable joints: {movable_joints}")
    
    # Step 4: Control joints with automatic visualization (NEW!)
    print("✅ Step 4: Starting joint motion...")
    
    def simple_motion_demo():
        """Simple motion using the new library interface"""
        for i in range(100):  # 10 seconds at 10Hz
            t = i * 0.1
            
            # Set joint positions with one command per joint
            for j, joint_name in enumerate(movable_joints):
                position = 0.5 * math.sin(t + j * math.pi / 2)
                # This one line sets the joint AND updates the visualization!
                visualizer.set_joint_command("my_robot", joint_name, position, max_velocity=1.0)
            
            time.sleep(0.1)  # 10 Hz
        
        print("✅ Motion completed!")
    
    try:
        if visualizer.display_available:
            import threading
            
            # Start motion in background
            motion_thread = threading.Thread(target=simple_motion_demo)
            motion_thread.daemon = True
            motion_thread.start()
            
            print("🎮 Interactive window opened - close to exit")
            visualizer.plotter.show()
        else:
            # Headless mode
            simple_motion_demo()
            
            os.makedirs("../../output", exist_ok=True)
            visualizer.plotter.screenshot("../../output/simple_urdf_demo.png")
            print("📸 Screenshot saved: ../../output/simple_urdf_demo.png")
        
        return True
        
    except Exception as e:
        print(f"❌ Demo error: {e}")
        return False


def even_simpler_demo():
    """Ultra-simple demo using the quick_robot_demo function"""
    print("\n🌟 Even Simpler Demo - One Function Call!")
    print("=" * 50)
    
    # Create robot
    env = simpy.Environment()
    urdf_path = "examples/robots/movable_robot.urdf"
    robot = create_robot_from_urdf(env, urdf_path, "quick_demo_robot")
    
    # Everything else in ONE function call!
    success = quick_robot_demo(
        robot_instance=robot,
        urdf_path=urdf_path,
        robot_name="quick_robot", 
        duration=8.0,
        demo_type="mixed"
    )
    
    if success:
        print("✅ Ultra-simple demo completed!")
    else:
        print("❌ Ultra-simple demo failed!")
    
    return success


def main():
    """Run both demos"""
    print("🤖 SimPyROS URDF Library Demo")
    print("Showing the power of the new URDFRobotVisualizer!")
    print("=" * 70)
    
    try:
        # Demo 1: Step by step
        print("\n📚 DEMO 1: Step-by-step library usage")
        simple_library_demo()
        
        time.sleep(2)  # Brief pause
        
        # Demo 2: Ultra simple
        print("\n⚡ DEMO 2: Ultra-simple one-function approach")  
        even_simpler_demo()
        
        print("\n🎉 Both demos completed!")
        print("The new library makes robot visualization incredibly easy:")
        print("  - visualizer.load_robot(name, robot, urdf)")
        print("  - visualizer.set_joint_command(name, joint, position)")
        print("  - quick_robot_demo(robot, urdf, name, duration)")
        
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()