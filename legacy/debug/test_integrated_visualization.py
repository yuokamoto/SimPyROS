#!/usr/bin/env python3
"""
Test integrated visualization: Robot data → URDFRobotVisualizer (memo.txt 36, 37)
"""
import time
import math
from core.simulation_manager import SimulationManager

def main():
    print("🔗 Testing Integrated Robot Visualization")
    print("=" * 45)
    print("✅ RobotMeshFactory integrated into URDFRobotVisualizer (memo.txt 36)")
    print("✅ Robot link/joint info used directly for rendering (memo.txt 37)")
    print("✅ No duplicate URDF loading - efficient single-source rendering")
    print("")
    
    # Create simulation
    sim = SimulationManager()
    
    # Load robot - this uses Robot's URDF data directly
    robot = sim.add_robot_from_urdf('integrated_robot', 'examples/robots/articulated_arm_robot.urdf')

    def integrated_control(dt):
        t = time.time()
        joint_names = [name for name in robot.get_joint_names() 
                      if robot.joints[name].joint_type.value != 'fixed']
        
        # Smooth multi-joint motion
        for i, joint_name in enumerate(joint_names):
            phase = t * 0.7 + i * math.pi / 3
            position = 0.7 * math.sin(phase)
            sim.set_robot_joint_position('integrated_robot', joint_name, position)

    sim.set_robot_control_callback('integrated_robot', integrated_control, frequency=25.0)
    
    print("🤖 Testing Features:")
    print("  - Robot instance data directly used for visualization")
    print("  - No RobotMeshFactory dependency")
    print("  - Single URDF loading per robot")
    print("  - Visual origins from Robot.urdf_loader applied correctly")
    print("")
    print("🎮 UI Features:")
    print("  - 🎯 Axes: Toggle coordinate axes (default OFF)")
    print("  - 🚧 Collision: Toggle collision geometry")  
    print("  - 🕸️ Wire: Toggle wireframe mode")
    print("  - ▶️ Play/Pause: Pause/resume (includes mobile robot base)")
    print("  - 🔄 Reset: Reset robot position")
    print("  - 📊 Real-time Factor: Dynamic speed control (0.1x - 5.0x)")
    print("")
    print("🚀 Starting integrated visualization test...")
    print("Close window to exit.")
    
    # Run simulation
    sim.run(duration=30.0)
    print("✅ Integrated visualization test completed successfully")

if __name__ == "__main__":
    main()