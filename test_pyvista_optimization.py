#!/usr/bin/env python3
"""
PyVista最適化デモとパフォーマンステスト

このスクリプトは:
1. 標準PyVistaビジュアライザーと最適化版の比較
2. バッチレンダリングの効果測定
3. 複数ロボットでのパフォーマンス検証
4. リアルタイム係数の改善測定
"""

import sys
import os
import time
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose


def test_standard_vs_optimized(num_robots: int = 5, duration: float = 10.0):
    """標準PyVistaと最適化版のパフォーマンス比較"""
    
    print(f"⚖️ Standard vs Optimized PyVista Comparison")
    print(f"Robots: {num_robots}, Duration: {duration}s")
    print("=" * 60)
    
    results = {}
    
    # Test configurations
    test_configs = [
        ("Standard PyVista", {
            'use_optimized_visualizer': False,
            'visualization': True,
            'real_time_factor': 1.0,
            'visualization_update_rate': 30.0
        }),
        ("Optimized Performance", {
            'use_optimized_visualizer': True,
            'visualization_optimization_level': 'performance',
            'enable_batch_rendering': True,
            'visualization': True,
            'real_time_factor': 1.0,
            'visualization_update_rate': 60.0
        }),
        ("Optimized Balanced", {
            'use_optimized_visualizer': True,
            'visualization_optimization_level': 'balanced',
            'enable_batch_rendering': True,
            'visualization': True,
            'real_time_factor': 1.0,
            'visualization_update_rate': 30.0
        })
    ]
    
    for config_name, config_params in test_configs:
        print(f"\n🧪 Testing {config_name}...")
        
        # Create configuration
        config = SimulationConfig(**config_params)
        sim = SimulationManager(config)
        
        try:
            # Create robots in a grid
            robots_created = 0
            for i in range(num_robots):
                try:
                    robot_name = f"robot_{i}"
                    x = (i % 3) * 3.0  # 3x grid
                    y = (i // 3) * 3.0
                    
                    robot = sim.add_robot_from_urdf(
                        name=robot_name,
                        urdf_path="examples/robots/mobile_robot.urdf",
                        initial_pose=Pose(x=x, y=y, z=0)
                    )
                    
                    if robot:
                        robots_created += 1
                        
                        # Add simple movement controller
                        def create_controller(robot_id):
                            def controller(dt):
                                t = sim.get_sim_time()
                                # Circular motion pattern
                                velocity = Velocity(
                                    linear_x=0.2 * math.sin(t + robot_id),
                                    angular_z=0.1 * math.cos(t + robot_id * 0.5)
                                )
                                sim.set_robot_velocity(f"robot_{robot_id}", velocity)
                            return controller
                        
                        sim.set_robot_control_callback(robot_name, create_controller(i), frequency=10.0)
                    
                except Exception as e:
                    print(f"   ⚠️ Failed to create robot {i}: {e}")
            
            print(f"   ✅ Created {robots_created}/{num_robots} robots")
            
            if robots_created == 0:
                print(f"   ❌ No robots created, skipping test")
                continue
            
            # Performance measurement
            start_time = time.time()
            
            # Run simulation
            sim.run(duration=duration, auto_close=True)
            
            end_time = time.time()
            wall_time = end_time - start_time
            sim_time = sim.get_sim_time()
            actual_rtf = sim_time / wall_time if wall_time > 0 else 0
            
            # Get performance stats from optimized visualizer
            perf_stats = {}
            if hasattr(sim.visualizer, 'get_performance_stats'):
                perf_stats = sim.visualizer.get_performance_stats()
            
            # Store results
            results[config_name] = {
                'wall_time': wall_time,
                'sim_time': sim_time,
                'actual_rtf': actual_rtf,
                'robots_created': robots_created,
                'target_rtf': config.real_time_factor,
                'rtf_error': abs(actual_rtf - config.real_time_factor) / config.real_time_factor * 100,
                'perf_stats': perf_stats
            }
            
            print(f"   📊 Results:")
            print(f"      Wall time: {wall_time:.2f}s")
            print(f"      Sim time: {sim_time:.2f}s")
            print(f"      Actual RTF: {actual_rtf:.3f}x")
            print(f"      RTF Error: {results[config_name]['rtf_error']:.1f}%")
            
            if perf_stats:
                print(f"      Render count: {perf_stats.get('render_count', 0)}")
                print(f"      Avg FPS: {perf_stats.get('avg_fps', 0):.1f}")
                print(f"      Render efficiency: {perf_stats.get('render_efficiency', 0):.1%}")
            
        except Exception as e:
            print(f"   ❌ Test failed: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            try:
                sim.shutdown()
            except:
                pass
            time.sleep(2.0)  # Cool down between tests
    
    # Print comparison summary
    print(f"\n🏆 Performance Comparison Summary")
    print("=" * 60)
    print(f"{'Configuration':<20} {'RTF':<8} {'Error':<8} {'Rating'}")
    print("-" * 60)
    
    for config_name, result in results.items():
        rtf = result['actual_rtf']
        error = result['rtf_error']
        
        if error < 5.0:
            rating = "🚀 EXCELLENT"
        elif error < 10.0:
            rating = "✅ GOOD"
        elif error < 20.0:
            rating = "⚠️ FAIR"
        else:
            rating = "❌ POOR"
        
        print(f"{config_name:<20} {rtf:<8.3f} {error:<8.1f}% {rating}")
    
    return results


def test_batch_rendering_effect():
    """バッチレンダリングの効果をテスト"""
    
    print(f"\n🔄 Batch Rendering Effect Test")
    print("=" * 50)
    
    configs = [
        ("Without Batch Rendering", {'enable_batch_rendering': False}),
        ("With Batch Rendering", {'enable_batch_rendering': True})
    ]
    
    results = {}
    
    for config_name, batch_config in configs:
        print(f"\n🧪 Testing {config_name}...")
        
        config = SimulationConfig(
            use_optimized_visualizer=True,
            visualization_optimization_level='performance',
            visualization=True,
            real_time_factor=1.0,
            **batch_config
        )
        
        sim = SimulationManager(config)
        
        try:
            # Create multiple robots for batch testing
            num_robots = 8
            for i in range(num_robots):
                robot_name = f"batch_robot_{i}"
                x = (i % 4) * 2.0
                y = (i // 4) * 2.0
                
                robot = sim.add_robot_from_urdf(
                    name=robot_name,
                    urdf_path="examples/robots/mobile_robot.urdf",
                    initial_pose=Pose(x=x, y=y, z=0)
                )
                
                if robot:
                    # Simple motion controller
                    def create_controller(robot_id):
                        def controller(dt):
                            t = sim.get_sim_time()
                            velocity = Velocity(
                                linear_x=0.1,
                                angular_z=0.05 * math.sin(t + robot_id)
                            )
                            sim.set_robot_velocity(f"batch_robot_{robot_id}", velocity)
                        return controller
                    
                    sim.set_robot_control_callback(robot_name, create_controller(i), frequency=10.0)
            
            # Run test
            start_time = time.time()
            sim.run(duration=5.0, auto_close=True)
            end_time = time.time()
            
            wall_time = end_time - start_time
            sim_time = sim.get_sim_time()
            actual_rtf = sim_time / wall_time if wall_time > 0 else 0
            
            # Get performance stats
            perf_stats = {}
            if hasattr(sim.visualizer, 'get_performance_stats'):
                perf_stats = sim.visualizer.get_performance_stats()
                
            results[config_name] = {
                'wall_time': wall_time,
                'actual_rtf': actual_rtf,
                'perf_stats': perf_stats
            }
            
            print(f"   Actual RTF: {actual_rtf:.3f}x")
            if perf_stats:
                print(f"   Avg FPS: {perf_stats.get('avg_fps', 0):.1f}")
                print(f"   Render efficiency: {perf_stats.get('render_efficiency', 0):.1%}")
            
        except Exception as e:
            print(f"   ❌ Test failed: {e}")
        
        finally:
            try:
                sim.shutdown()
            except:
                pass
            time.sleep(1.0)
    
    # Compare results
    if len(results) == 2:
        without_batch = results["Without Batch Rendering"]
        with_batch = results["With Batch Rendering"]
        
        rtf_improvement = (with_batch['actual_rtf'] - without_batch['actual_rtf']) / without_batch['actual_rtf'] * 100
        
        print(f"\n📊 Batch Rendering Impact:")
        print(f"   RTF improvement: {rtf_improvement:+.1f}%")
        
        if 'perf_stats' in with_batch and 'perf_stats' in without_batch:
            without_fps = without_batch['perf_stats'].get('avg_fps', 0)
            with_fps = with_batch['perf_stats'].get('avg_fps', 0)
            fps_improvement = (with_fps - without_fps) / without_fps * 100 if without_fps > 0 else 0
            print(f"   FPS improvement: {fps_improvement:+.1f}%")
    
    return results


def test_scalability(max_robots: int = 20):
    """ロボット数スケーラビリティテスト"""
    
    print(f"\n📈 Scalability Test (up to {max_robots} robots)")
    print("=" * 50)
    
    robot_counts = [1, 3, 5, 10, 15, 20]
    robot_counts = [r for r in robot_counts if r <= max_robots]
    
    results = {}
    
    for num_robots in robot_counts:
        print(f"\n🧪 Testing {num_robots} robots...")
        
        config = SimulationConfig(
            use_optimized_visualizer=True,
            visualization_optimization_level='performance',
            enable_batch_rendering=True,
            visualization=True,
            real_time_factor=1.0,
            visualization_update_rate=30.0
        )
        
        sim = SimulationManager(config)
        
        try:
            # Create robots
            robots_created = 0
            for i in range(num_robots):
                try:
                    robot_name = f"scale_robot_{i}"
                    grid_size = int(math.sqrt(num_robots)) + 1
                    x = (i % grid_size) * 2.0
                    y = (i // grid_size) * 2.0
                    
                    robot = sim.add_robot_from_urdf(
                        name=robot_name,
                        urdf_path="examples/robots/mobile_robot.urdf",
                        initial_pose=Pose(x=x, y=y, z=0)
                    )
                    
                    if robot:
                        robots_created += 1
                        
                        # Add controller
                        def create_controller(robot_id):
                            def controller(dt):
                                t = sim.get_sim_time()
                                velocity = Velocity(
                                    linear_x=0.1,
                                    angular_z=0.05 * (1 if robot_id % 2 == 0 else -1)
                                )
                                sim.set_robot_velocity(f"scale_robot_{robot_id}", velocity)
                            return controller
                        
                        sim.set_robot_control_callback(robot_name, create_controller(i), frequency=5.0)
                
                except Exception as e:
                    print(f"     ⚠️ Failed to create robot {i}: {e}")
                    break
            
            if robots_created == 0:
                print(f"   ❌ No robots created")
                continue
                
            print(f"   ✅ Created {robots_created} robots")
            
            # Performance test
            start_time = time.time()
            sim.run(duration=3.0, auto_close=True)
            end_time = time.time()
            
            wall_time = end_time - start_time
            sim_time = sim.get_sim_time()
            actual_rtf = sim_time / wall_time if wall_time > 0 else 0
            
            results[num_robots] = {
                'robots_created': robots_created,
                'actual_rtf': actual_rtf,
                'wall_time': wall_time
            }
            
            print(f"   📊 RTF: {actual_rtf:.3f}x")
            
        except Exception as e:
            print(f"   ❌ Test failed: {e}")
        
        finally:
            try:
                sim.shutdown()
            except:
                pass
            time.sleep(1.0)
    
    # Print scalability summary
    print(f"\n📊 Scalability Results:")
    print(f"{'Robots':<8} {'RTF':<8} {'Performance'}")
    print("-" * 30)
    
    for num_robots, result in results.items():
        rtf = result['actual_rtf']
        
        if rtf >= 0.9:
            perf = "🚀 Excellent"
        elif rtf >= 0.7:
            perf = "✅ Good"
        elif rtf >= 0.5:
            perf = "⚠️ Fair"
        else:
            perf = "❌ Poor"
        
        print(f"{num_robots:<8} {rtf:<8.3f} {perf}")
    
    return results


def main():
    """メイン実行関数"""
    
    print("🎯 PyVista最適化パフォーマンステスト")
    print("=" * 70)
    
    try:
        # Test 1: Standard vs Optimized comparison
        print("\n1️⃣ Standard vs Optimized Comparison")
        standard_vs_optimized_results = test_standard_vs_optimized(num_robots=3, duration=8.0)
        
        # Test 2: Batch rendering effect
        print("\n2️⃣ Batch Rendering Effect")
        batch_results = test_batch_rendering_effect()
        
        # Test 3: Scalability test
        print("\n3️⃣ Scalability Test")
        scalability_results = test_scalability(max_robots=10)
        
        # Final summary
        print(f"\n🏁 Overall Test Summary")
        print("=" * 50)
        
        if standard_vs_optimized_results:
            optimized_configs = [k for k in standard_vs_optimized_results.keys() if 'Optimized' in k]
            if optimized_configs:
                best_config = min(optimized_configs, 
                                key=lambda x: standard_vs_optimized_results[x]['rtf_error'])
                best_result = standard_vs_optimized_results[best_config]
                print(f"✅ Best optimization: {best_config}")
                print(f"   RTF: {best_result['actual_rtf']:.3f}x")
                print(f"   Error: {best_result['rtf_error']:.1f}%")
        
        if scalability_results:
            max_robots_tested = max(scalability_results.keys())
            max_rtf = scalability_results[max_robots_tested]['actual_rtf']
            print(f"📊 Max robots tested: {max_robots_tested}")
            print(f"   RTF with {max_robots_tested} robots: {max_rtf:.3f}x")
        
        print(f"\n💡 Recommendations:")
        print(f"   - Use optimized visualizer for better performance")
        print(f"   - Enable batch rendering for multiple robots")
        print(f"   - Performance level for real-time applications")
        print(f"   - Balanced level for general use")
        
    except KeyboardInterrupt:
        print(f"\n⏹️ Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test suite failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()