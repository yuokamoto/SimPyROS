#!/usr/bin/env python3
"""
Backend Comparison Demo - 3つのバックエンドの性能比較デモ
"""

import sys
import os
import time
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import simpyros


def create_circle_controller(robot_id: int, radius: float = 2.0):
    """円運動コントローラー作成"""
    def controller(dt):
        t = time.time() * 0.5  # ゆっくり回転
        angle = t + robot_id * (2 * math.pi / 10)  # 10台で円を作る
        
        # 円運動の速度計算
        angular_velocity = 0.5
        linear_velocity = radius * angular_velocity
        
        velocity = simpyros.Velocity(
            linear_x=linear_velocity * math.cos(angle + math.pi/2),
            linear_y=linear_velocity * math.sin(angle + math.pi/2),
            angular_z=angular_velocity
        )
        
        return velocity
    
    return controller


def demo_backend(backend: simpyros.SimulationBackend, 
                num_robots: int = 20, 
                duration: float = 5.0,
                visualization: bool = False):
    """単一バックエンドのデモ"""
    
    print(f"\n🧪 {backend.value.upper()} Demo")
    print("-" * 50)
    
    # バックエンドに応じた最適設定
    if backend == simpyros.SimulationBackend.SIMPLE_WHILE_LOOP:
        config = simpyros.create_high_performance_config(visualization=visualization)
        config.update_rate = 60.0 if not visualization else 30.0
    elif backend == simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP:
        config = simpyros.create_balanced_config(visualization=visualization)
        config.update_rate = 30.0
    else:  # SIMPY_PURE
        config = simpyros.create_feature_rich_config(visualization=visualization)
        config.update_rate = 20.0
    
    config.backend = backend
    config.real_time_factor = 0.0  # 最高速度で比較
    
    sim = simpyros.create_simulation_manager(config)
    
    try:
        creation_start = time.time()
        
        # ロボット作成
        for i in range(num_robots):
            # 円形配置
            angle = i * 2 * math.pi / num_robots
            radius = 5.0
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            robot = sim.add_robot_from_urdf(
                name=f"robot_{i}",
                urdf_path="examples/robots/mobile_robot.urdf",
                initial_pose=simpyros.Pose(x=x, y=y, z=0),
                joint_update_rate=10.0
            )
            
            # 円運動コントローラー
            controller = create_circle_controller(i)
            
            def make_callback(ctrl, robot_name):
                def callback(dt):
                    vel = ctrl(dt)
                    sim.set_robot_velocity(robot_name, vel)
                return callback
            
            sim.set_robot_control_callback(
                f"robot_{i}", 
                make_callback(controller, f"robot_{i}"),
                frequency=10.0
            )
        
        creation_time = time.time() - creation_start
        print(f"   Robot creation: {creation_time:.3f}s")
        
        # 実行
        print(f"   Running simulation for {duration}s...")
        sim.run(duration=duration)
        
        # 結果取得
        stats = sim.get_performance_stats()
        
        print(f"   ✅ Completed")
        print(f"   RTF: {stats['rtf']:.3f}x")
        print(f"   Frame rate: {stats['avg_fps']:.1f} Hz")
        print(f"   Callbacks/sec: {stats['callbacks_per_sec']:.1f}")
        
        return stats
        
    except Exception as e:
        print(f"   ❌ Failed: {e}")
        return None
        
    finally:
        sim.shutdown()


def comprehensive_comparison():
    """包括的な性能比較"""
    
    print("🎯 SimPyROS Backend Comprehensive Comparison")
    print("=" * 70)
    
    test_configs = [
        (10, "小規模 (10台)", 3.0),
        (50, "中規模 (50台)", 3.0),
        (100, "大規模 (100台)", 2.0),
    ]
    
    backends = [
        simpyros.SimulationBackend.SIMPLE_WHILE_LOOP,
        simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP,
        # simpyros.SimulationBackend.SIMPY_PURE  # 時間節約のため省略
    ]
    
    results = []
    
    for num_robots, scale_desc, duration in test_configs:
        print(f"\n📊 {scale_desc}テスト")
        print("=" * 40)
        
        scale_results = {}
        
        for backend in backends:
            stats = demo_backend(
                backend=backend,
                num_robots=num_robots,
                duration=duration,
                visualization=False  # 性能比較のためヘッドレス
            )
            
            if stats:
                scale_results[backend.value] = stats
                
            time.sleep(1.0)  # システム安定化
        
        results.append((scale_desc, scale_results))
    
    # 総合分析
    print(f"\n🏁 Comprehensive Analysis Results")
    print("=" * 70)
    print(f"{'Scale':<15} {'Backend':<20} {'RTF':<10} {'CB/sec':<12} {'Rating'}")
    print("-" * 70)
    
    for scale_desc, scale_results in results:
        for backend_name, stats in scale_results.items():
            rtf = stats['rtf']
            cb_per_sec = stats['callbacks_per_sec']
            
            if rtf >= 1.0:
                rating = "🚀 ULTRA"
            elif rtf >= 0.5:
                rating = "⚡ FAST"
            elif rtf >= 0.1:
                rating = "✅ GOOD"
            elif rtf >= 0.05:
                rating = "⚠️ FAIR"
            else:
                rating = "❌ SLOW"
            
            print(f"{scale_desc:<15} {backend_name:<20} {rtf:<10.3f} {cb_per_sec:<12.1f} {rating}")
    
    # 推奨事項
    print(f"\n💡 Backend Selection Recommendations:")
    print("=" * 50)
    print("📈 Performance Priority:")
    print("   - 100+ robots → Simple While Loop (RTF ~1.0x)")
    print("   - 50+ robots + 可視化 → Simple While Loop")
    print("   - 最高速度重視 → Simple While Loop")
    
    print("\n⚖️ Balance (Performance + Features):")
    print("   - 10-50 robots → SimPy FrequencyGroup (RTF ~0.1-0.5x)")
    print("   - リアルタイム制御 → SimPy FrequencyGroup")
    print("   - 可視化 + 制御 → SimPy FrequencyGroup")
    
    print("\n🎭 Feature Priority:")
    print("   - 複雑なイベント処理 → Pure SimPy")
    print("   - リソース管理 → Pure SimPy")
    print("   - 非同期通信 → Pure SimPy")
    
    return results


def interactive_demo():
    """インタラクティブなデモ"""
    
    print("🎮 Interactive Backend Demo")
    print("=" * 40)
    
    # バックエンド選択
    print("\n📋 Available Backends:")
    backends = [
        (simpyros.SimulationBackend.SIMPLE_WHILE_LOOP, "Simple While Loop - 最高性能"),
        (simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP, "SimPy FrequencyGroup - バランス"),
        (simpyros.SimulationBackend.SIMPY_PURE, "Pure SimPy - 高機能")
    ]
    
    for i, (backend, description) in enumerate(backends, 1):
        print(f"   {i}. {description}")
    
    try:
        choice = int(input("\nSelect backend (1-3): ")) - 1
        if 0 <= choice < len(backends):
            selected_backend, description = backends[choice]
            
            num_robots = int(input("Number of robots (10-100): "))
            duration = float(input("Duration in seconds (1-10): "))
            
            print(f"\n🚀 Running {description}")
            print(f"   Robots: {num_robots}")
            print(f"   Duration: {duration}s")
            
            stats = demo_backend(
                backend=selected_backend,
                num_robots=num_robots,
                duration=duration,
                visualization=False
            )
            
            if stats:
                print(f"\n🎯 Your Results:")
                print(f"   Backend: {selected_backend.value}")
                print(f"   RTF: {stats['rtf']:.3f}x")
                print(f"   Performance: {'🚀 EXCELLENT' if stats['rtf'] >= 1.0 else '✅ GOOD' if stats['rtf'] >= 0.1 else '⚠️ NEEDS IMPROVEMENT'}")
        else:
            print("Invalid choice")
            
    except (ValueError, KeyboardInterrupt):
        print("\nDemo cancelled")


def main():
    """メインデモ実行"""
    
    print("🎯 SimPyROS - Unified Robotics Simulation Framework")
    print("Backend Comparison and Performance Analysis")
    print("=" * 80)
    
    try:
        # 1. 簡単なクイック比較
        print("\n1️⃣ Quick Backend Comparison")
        simpyros.benchmark_all_backends(num_robots=20, duration=2.0)
        
        time.sleep(2.0)
        
        # 2. 包括的比較（時間がある場合）
        response = input("\n🤔 Run comprehensive comparison? (y/n): ").lower().strip()
        if response.startswith('y'):
            comprehensive_comparison()
        
        # 3. インタラクティブデモ
        response = input("\n🎮 Run interactive demo? (y/n): ").lower().strip()
        if response.startswith('y'):
            interactive_demo()
            
        print(f"\n✅ Demo completed successfully!")
        print(f"💡 For more examples, see: simpyros.print_usage_examples()")
        
    except KeyboardInterrupt:
        print(f"\n⏹️ Demo interrupted")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()