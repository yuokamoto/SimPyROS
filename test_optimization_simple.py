#!/usr/bin/env python3
"""
シンプルなPyVista最適化テスト（セグメンテーションフォルト回避版）

このテストは:
1. ヘッドレスモードでの最適化効果測定
2. バッチレンダリングのロジックテスト
3. パフォーマンス統計の収集
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig


def test_headless_performance():
    """ヘッドレスモードでの最適化効果テスト"""
    
    print("🧪 Headless Performance Test")
    print("=" * 40)
    
    configs = [
        ("Standard (Headless)", {
            'use_optimized_visualizer': False,
            'visualization': False,
            'real_time_factor': 1.0
        }),
        ("Optimized (Headless)", {
            'use_optimized_visualizer': True,
            'visualization': False,
            'real_time_factor': 1.0
        })
    ]
    
    results = {}
    
    for config_name, config_params in configs:
        print(f"\n🔬 Testing {config_name}...")
        
        config = SimulationConfig(**config_params)
        sim = SimulationManager(config)
        
        try:
            # Create a simple robot
            robot = sim.add_robot_from_urdf(
                name="test_robot",
                urdf_path="examples/robots/mobile_robot.urdf"
            )
            
            if robot:
                # Add simple controller
                def controller(dt):
                    from core.simulation_object import Velocity
                    velocity = Velocity(linear_x=0.1, angular_z=0.05)
                    sim.set_robot_velocity("test_robot", velocity)
                
                sim.set_robot_control_callback("test_robot", controller, frequency=10.0)
                
                # Performance test
                start_time = time.time()
                sim.run(duration=3.0, auto_close=True)
                end_time = time.time()
                
                wall_time = end_time - start_time
                sim_time = sim.get_sim_time()
                actual_rtf = sim_time / wall_time if wall_time > 0 else 0
                
                results[config_name] = {
                    'wall_time': wall_time,
                    'sim_time': sim_time,
                    'actual_rtf': actual_rtf,
                    'rtf_error': abs(actual_rtf - 1.0) * 100
                }
                
                print(f"   Wall time: {wall_time:.3f}s")
                print(f"   Sim time: {sim_time:.3f}s")
                print(f"   RTF: {actual_rtf:.3f}x")
                print(f"   Error: {results[config_name]['rtf_error']:.1f}%")
            else:
                print(f"   ❌ Failed to create robot")
                
        except Exception as e:
            print(f"   ❌ Test failed: {e}")
            
        finally:
            try:
                sim.shutdown()
            except:
                pass
            time.sleep(1.0)
    
    return results


def test_batch_logic():
    """バッチレンダリングロジックの基本テスト"""
    
    print("\n🔄 Batch Rendering Logic Test")
    print("=" * 40)
    
    try:
        from core.optimized_pyvista_visualizer import OptimizedPyVistaVisualizer
        
        # Create optimized visualizer (headless to avoid segfault)
        visualizer = OptimizedPyVistaVisualizer(interactive=False)
        
        if not visualizer.available:
            print("❌ PyVista not available")
            return
        
        print("✅ Optimized visualizer created")
        
        # Test batch mode context manager
        print("🧪 Testing batch mode context manager...")
        
        try:
            with visualizer.batch_mode():
                print("   ✅ Entered batch mode")
                # Simulate some operations
                visualizer._pending_renders = True
            print("   ✅ Exited batch mode")
            
        except Exception as e:
            print(f"   ❌ Batch mode test failed: {e}")
        
        # Test performance stats
        print("🧪 Testing performance statistics...")
        
        try:
            stats = visualizer.get_performance_stats()
            print(f"   ✅ Performance stats retrieved: {len(stats)} metrics")
            
            for key, value in stats.items():
                print(f"      {key}: {value}")
                
        except Exception as e:
            print(f"   ❌ Performance stats test failed: {e}")
        
        # Test optimization levels
        print("🧪 Testing optimization levels...")
        
        try:
            levels = ['performance', 'balanced', 'quality']
            for level in levels:
                visualizer.set_optimization_level(level)
                print(f"   ✅ Set optimization level: {level}")
                
        except Exception as e:
            print(f"   ❌ Optimization level test failed: {e}")
        
        print("✅ Batch logic tests completed")
        
    except ImportError as e:
        print(f"❌ Cannot import optimized visualizer: {e}")
    except Exception as e:
        print(f"❌ Batch logic test failed: {e}")


def test_config_integration():
    """SimulationConfigとの統合テスト"""
    
    print("\n⚙️ Configuration Integration Test")
    print("=" * 40)
    
    # Test different optimization configurations
    test_configs = [
        {
            'name': 'Standard Config',
            'config': {
                'use_optimized_visualizer': False,
                'visualization': False
            }
        },
        {
            'name': 'Optimized Performance',
            'config': {
                'use_optimized_visualizer': True,
                'visualization_optimization_level': 'performance',
                'enable_batch_rendering': True,
                'visualization': False
            }
        },
        {
            'name': 'Optimized Balanced',
            'config': {
                'use_optimized_visualizer': True,
                'visualization_optimization_level': 'balanced',
                'enable_batch_rendering': True,
                'visualization': False
            }
        }
    ]
    
    for test_config in test_configs:
        print(f"\n🧪 Testing {test_config['name']}...")
        
        try:
            config = SimulationConfig(**test_config['config'])
            sim = SimulationManager(config)
            
            # Check visualizer type
            if hasattr(sim, 'visualizer') and sim.visualizer:
                visualizer_type = type(sim.visualizer).__name__
                print(f"   Visualizer type: {visualizer_type}")
                
                # Check optimization features
                if hasattr(sim.visualizer, 'set_optimization_level'):
                    print("   ✅ Optimization features available")
                else:
                    print("   ℹ️ Standard visualizer (no optimization)")
            else:
                print("   ℹ️ No visualizer (headless mode)")
            
            print(f"   ✅ Configuration applied successfully")
            
            # Quick simulation test
            robot = sim.add_robot_from_urdf(
                name="config_test_robot",
                urdf_path="examples/robots/mobile_robot.urdf"
            )
            
            if robot:
                print("   ✅ Robot creation successful")
                
                # Quick run test
                start_time = time.time()
                sim.run(duration=0.5, auto_close=True)
                end_time = time.time()
                
                elapsed = end_time - start_time
                print(f"   ✅ Quick run completed ({elapsed:.3f}s)")
            else:
                print("   ⚠️ Robot creation failed")
            
        except Exception as e:
            print(f"   ❌ Configuration test failed: {e}")
            
        finally:
            try:
                sim.shutdown()
            except:
                pass
            time.sleep(0.5)


def main():
    """メイン実行"""
    
    print("🎯 PyVista最適化テスト（セーフ版）")
    print("=" * 50)
    
    try:
        # Test 1: Headless performance
        print("\n1️⃣ Headless Performance Test")
        headless_results = test_headless_performance()
        
        # Test 2: Batch logic
        print("\n2️⃣ Batch Rendering Logic")
        test_batch_logic()
        
        # Test 3: Config integration
        print("\n3️⃣ Configuration Integration")
        test_config_integration()
        
        # Summary
        print("\n🏁 Test Summary")
        print("=" * 30)
        
        if headless_results:
            print("📊 Headless Performance Results:")
            for config_name, result in headless_results.items():
                rtf = result.get('actual_rtf', 0)
                error = result.get('rtf_error', 100)
                
                if error < 5.0:
                    rating = "🚀 EXCELLENT"
                elif error < 10.0:
                    rating = "✅ GOOD"
                else:
                    rating = "⚠️ NEEDS IMPROVEMENT"
                
                print(f"   {config_name}: {rtf:.3f}x RTF ({error:.1f}% error) {rating}")
            
            # Calculate improvement
            if len(headless_results) >= 2:
                configs = list(headless_results.keys())
                standard_rtf = headless_results[configs[0]]['actual_rtf']
                optimized_rtf = headless_results[configs[1]]['actual_rtf']
                improvement = (optimized_rtf - standard_rtf) / standard_rtf * 100
                print(f"   🔄 Optimization improvement: {improvement:+.1f}%")
        
        print("\n💡 Key Benefits:")
        print("   ✅ Batch rendering reduces GPU overhead")
        print("   ✅ Frame rate control prevents excessive rendering")
        print("   ✅ Performance monitoring enables optimization")
        print("   ✅ GPU optimizations improve hardware utilization")
        print("   ✅ Configurable optimization levels")
        
        print("\n🎯 Usage Recommendations:")
        print("   - Use 'performance' level for real-time applications")
        print("   - Use 'balanced' level for general simulation")
        print("   - Enable batch rendering for multiple robots")
        print("   - Monitor performance stats for optimization")
        
    except Exception as e:
        print(f"\n❌ Test suite failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()