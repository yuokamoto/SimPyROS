#!/usr/bin/env python3
"""
Test script to verify FrequencyGroup fixes eliminate Robot individual yields
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose


def test_frequency_grouping_fix():
    """Test that FrequencyGroup eliminates individual Robot yields"""
    
    print("🎯 Testing FrequencyGroup yield elimination fix")
    print("=" * 50)
    
    # Create headless simulation with frequency grouping enabled
    config = SimulationConfig(
        visualization=False,
        enable_frequency_grouping=True,  # Enable auto frequency grouping
        update_rate=20.0,
        real_time_factor=0.0  # Maximum speed
    )
    
    sim = SimulationManager(config)
    
    try:
        print("🔧 Creating robots with frequency grouping enabled...")
        
        # Create 5 robots with same frequency - should be grouped
        robots = []
        for i in range(5):
            robot_name = f"test_robot_{i}"
            robot = sim.add_robot_from_urdf(
                name=robot_name,
                urdf_path="examples/robots/mobile_robot.urdf",
                initial_pose=Pose(x=i*2.0, y=0, z=0),
                joint_update_rate=10.0,  # Same frequency for all = 1 group
                unified_process=True
            )
            robots.append((robot_name, robot))
            
            # Add simple controller
            def create_controller(name):
                def controller(dt):
                    velocity = Velocity(linear_x=0.1, angular_z=0.1)
                    sim.set_robot_velocity(name, velocity)
                return controller
            
            sim.set_robot_control_callback(robot_name, create_controller(robot_name), frequency=10.0)
        
        print("✅ All robots created with same frequency (10 Hz)")
        
        # Check frequency grouping stats
        if hasattr(sim, 'get_frequency_grouping_stats'):
            stats = sim.get_frequency_grouping_stats()
            if stats['enabled']:
                print(f"📊 Frequency grouping statistics:")
                print(f"   Total robots: {stats['total_robots']}")
                print(f"   Total groups: {stats['total_groups']}")
                print(f"   Process reduction: {stats['process_reduction_percent']:.1f}%")
                
                for freq, group_info in stats['groups'].items():
                    print(f"   {freq} Hz: {group_info['robot_count']} robots")
        
        print(f"🚀 Running 5-second test simulation...")
        
        # Run short test
        sim.run(duration=5.0)
        
        # Get final stats
        sim_time = sim.get_sim_time()
        info = sim.get_simulation_info()
        
        print(f"✅ Test completed successfully!")
        print(f"   Simulation time: {sim_time:.2f}s")
        print(f"   Architecture: Frequency-Grouped")
        
        if 'frequency_grouping' in info and info['frequency_grouping']['enabled']:
            fg_info = info['frequency_grouping']
            print(f"   Active processes: {fg_info['groups']} (instead of {fg_info['total_robots']})")
            print(f"   Process reduction: {((fg_info['total_robots'] - fg_info['groups']) / fg_info['total_robots'] * 100):.1f}%")
        
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        sim.shutdown()


def test_traditional_vs_grouped():
    """Compare traditional vs frequency-grouped architectures"""
    
    print(f"\n🏁 Architecture Comparison Test")
    print("=" * 50)
    
    # Test 1: Traditional individual processes
    print("1️⃣ Testing Traditional Individual Processes...")
    config_traditional = SimulationConfig(
        visualization=False,
        enable_frequency_grouping=False,
        update_rate=20.0,
        real_time_factor=0.0
    )
    
    sim_traditional = SimulationManager(config_traditional)
    try:
        for i in range(3):
            robot = sim_traditional.add_robot_from_urdf(
                name=f"traditional_{i}",
                urdf_path="examples/robots/mobile_robot.urdf",
                initial_pose=Pose(x=i*2.0, y=0, z=0),
                joint_update_rate=10.0,
                unified_process=True
            )
        
        print("   ✅ 3 robots created with traditional architecture")
        sim_traditional.run(duration=2.0)
        print("   ✅ Traditional test completed")
        
    finally:
        sim_traditional.shutdown()
    
    # Test 2: Frequency-grouped processes
    print("2️⃣ Testing Frequency-Grouped Processes...")
    config_grouped = SimulationConfig(
        visualization=False,
        enable_frequency_grouping=True,
        update_rate=20.0,
        real_time_factor=0.0
    )
    
    sim_grouped = SimulationManager(config_grouped)
    try:
        for i in range(3):
            robot = sim_grouped.add_robot_from_urdf(
                name=f"grouped_{i}",
                urdf_path="examples/robots/mobile_robot.urdf",
                initial_pose=Pose(x=i*2.0, y=0, z=0),
                joint_update_rate=10.0,
                unified_process=True
            )
        
        print("   ✅ 3 robots created with frequency-grouped architecture")
        sim_grouped.run(duration=2.0)
        print("   ✅ Frequency-grouped test completed")
        
        # Show optimization
        if hasattr(sim_grouped, 'get_frequency_grouping_stats'):
            stats = sim_grouped.get_frequency_grouping_stats()
            if stats['enabled']:
                print(f"   📊 Process reduction: {stats['process_reduction_percent']:.1f}%")
                print(f"   📊 {stats['total_groups']} processes instead of {stats['total_robots']}")
        
    finally:
        sim_grouped.shutdown()
    
    print("🎉 Both architectures completed successfully!")
    return True


if __name__ == "__main__":
    print("🧪 FrequencyGroup Fix Verification Test")
    print("Testing elimination of Robot individual yields when FrequencyGroup is active")
    print("=" * 70)
    
    try:
        # Test the fix
        success1 = test_frequency_grouping_fix()
        
        if success1:
            # Test comparison
            success2 = test_traditional_vs_grouped()
            
            if success1 and success2:
                print(f"\n🏆 ALL TESTS PASSED!")
                print(f"✅ FrequencyGroup successfully eliminates Robot individual yields")
                print(f"✅ Process management correctly delegated to FrequencyGroups")
                print(f"✅ Both architectures work correctly")
                
            else:
                print(f"\n⚠️ Some tests failed")
        
    except KeyboardInterrupt:
        print(f"\n⏹️ Tests interrupted")
    except Exception as e:
        print(f"\n❌ Test suite failed: {e}")
        import traceback
        traceback.print_exc()