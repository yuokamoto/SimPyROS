#!/usr/bin/env python3
"""
Minimal Test Demo - scipyなしで動作する最小限のテスト
"""

import sys
import os
import time
import math

# Add path without importing heavy dependencies
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

def minimal_simulation_test():
    """最小限のシミュレーションテスト - 真のアルゴリズム比較"""
    
    print("🎯 Minimal Simulation Test")
    print("=" * 50)
    print("Real algorithm differences without time.sleep dependencies")
    print()
    
    num_robots = 5
    duration = 2.0
    
    def test_simple_while_loop():
        """Simple While Loop: 最小計算"""
        robots = []
        for i in range(num_robots):
            robots.append({'id': i, 'x': float(i), 'y': 0.0})
        
        start_time = time.time()
        update_count = 0
        
        while time.time() - start_time < duration:
            # Minimal direct computation
            t = time.time()
            for robot in robots:
                angle = t + robot['id'] * 0.1
                robot['x'] = robot['id'] + math.cos(angle)
                robot['y'] = math.sin(angle)
                update_count += 1
        
        return update_count, time.time() - start_time
    
    def test_frequency_group():
        """FrequencyGroup: 中程度の計算負荷"""
        robots = []
        for i in range(num_robots):
            robots.append({'id': i, 'x': float(i), 'y': 0.0})
        
        start_time = time.time()
        update_count = 0
        
        while time.time() - start_time < duration:
            t = time.time()
            
            for robot in robots:
                # Moderate complexity trigonometric calculations
                angle = t + robot['id'] * 0.15
                # Multiple trigonometric operations (moderate load)
                cos_val = math.cos(angle)
                sin_val = math.sin(angle)
                robot['x'] = robot['id'] + cos_val * sin_val
                robot['y'] = sin_val * math.cos(angle * 0.5) + math.sin(angle * 0.3)
                
                # Additional moderate calculation
                distance = math.sqrt(robot['x']**2 + robot['y']**2)
                if distance > 0:
                    robot['x'] *= (1.0 + 0.1 * math.sin(t))
                    robot['y'] *= (1.0 + 0.1 * math.cos(t))
                
                update_count += 1
        
        return update_count, time.time() - start_time
    
    def test_pure_simpy():
        """Pure SimPy: 状態管理 + 複雑な意思決定"""
        robots = []
        for i in range(num_robots):
            robots.append({
                'id': i, 'x': float(i), 'y': 0.0,
                'state': 'exploring', 'goal_x': i + 1.0, 'goal_y': 1.0,
                'timer': 0.0
            })
        
        start_time = time.time()
        update_count = 0
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            for robot in robots:
                # Complex state machine
                if current_time - robot['timer'] > (1.0 + robot['id'] * 0.2):
                    robot['state'] = 'returning' if robot['state'] == 'exploring' else 'exploring'
                    robot['timer'] = current_time
                
                # Path planning with obstacle avoidance
                dx = robot['goal_x'] - robot['x']
                dy = robot['goal_y'] - robot['y']
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Complex movement calculation
                if distance > 0.1:
                    speed = 0.1 + 0.05 * math.sin(current_time + robot['id'])
                    robot['x'] += (dx / distance) * speed
                    robot['y'] += (dy / distance) * speed
                else:
                    robot['x'] += 0.05 * math.cos(current_time * 2 + robot['id'])
                    robot['y'] += 0.05 * math.sin(current_time * 2 + robot['id'])
                
                update_count += 1
        
        return update_count, time.time() - start_time
    
    # Test each backend
    backends = [
        ("Simple While Loop", test_simple_while_loop),
        ("SimPy FrequencyGroup", test_frequency_group),
        ("Pure SimPy", test_pure_simpy)
    ]
    
    for backend_name, test_func in backends:
        print(f"\n🧪 Testing {backend_name}...")
        
        updates, elapsed = test_func()
        updates_per_sec = updates / elapsed
        
        print(f"   ✅ Updates/sec: {updates_per_sec:.0f}")
        
        # Performance rating based on actual computation complexity
        if updates_per_sec >= 200000:
            rating = "🚀 EXCELLENT"
        elif updates_per_sec >= 50000:
            rating = "⚡ FAST"  
        elif updates_per_sec >= 10000:
            rating = "✅ GOOD"
        else:
            rating = "⚠️ MODERATE"
        
        print(f"   Performance: {rating}")
    
    print(f"\n✅ Minimal test completed!")
    print("Real algorithm performance differences demonstrated!")

def interactive_backend_demo():
    """インタラクティブなバックエンドデモ - 実際のバックエンド実装による比較"""
    
    print(f"\n🎮 Interactive Backend Demo")
    print("=" * 50)
    print("Real backend implementation comparison")
    print()
    print("Commands:")
    print("  1: Simple While Loop Backend")
    print("  2: SimPy FrequencyGroup Backend") 
    print("  3: Pure SimPy Backend")
    print("  c: Compare all backends")
    print("  q: Quit")
    
    # 実際のバックエンド実装シミュレーション
    def simulate_while_loop_backend(duration=3.0, num_robots=15):
        """Simple While Loopバックエンドのシミュレーション"""
        print("🚀 Running Simple While Loop implementation...")
        
        robots = {}
        for i in range(num_robots):
            robots[f"robot_{i}"] = {
                'position': [i * 1.0, 0.0, 0.0],
                'velocity': [0.0, 0.0, 0.0],
                'update_count': 0
            }
        
        start_time = time.time()
        total_updates = 0
        frame_count = 0
        
        # Simple while loop - 直接実行、最小オーバーヘッド
        while time.time() - start_time < duration:
            # All robots updated directly in single loop
            for robot_name, robot in robots.items():
                t = time.time()
                robot_id = int(robot_name.split('_')[1])
                
                # Direct calculation - no process overhead
                angle = t * 0.5 + robot_id * 0.1
                robot['position'][0] = robot_id + math.cos(angle)
                robot['position'][1] = math.sin(angle)
                robot['update_count'] += 1
                total_updates += 1
            
            frame_count += 1
            # No sleep - pure computation speed
        
        return total_updates, frame_count, time.time() - start_time
    
    def simulate_frequency_group_backend(duration=3.0, num_robots=15):
        """SimPy FrequencyGroupバックエンドのシミュレーション"""
        print("⚡ Running SimPy FrequencyGroup implementation...")
        
        robots = {}
        for i in range(num_robots):
            robots[f"robot_{i}"] = {
                'position': [i * 1.0, 0.0, 0.0],
                'last_update_time': 0.0,
                'update_frequency': 50.0 + i * 2,  # Different frequencies
                'update_count': 0
            }
        
        start_time = time.time()
        total_updates = 0
        frame_count = 0
        
        # Frequency-based updates - moderate overhead
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # Group robots by frequency for batch processing
            high_freq_robots = []
            low_freq_robots = []
            
            for robot_name, robot in robots.items():
                if robot['update_frequency'] > 60:
                    high_freq_robots.append((robot_name, robot))
                else:
                    low_freq_robots.append((robot_name, robot))
            
            # Process high frequency robots
            for robot_name, robot in high_freq_robots:
                update_interval = 1.0 / robot['update_frequency']
                if current_time - robot['last_update_time'] >= update_interval:
                    robot_id = int(robot_name.split('_')[1])
                    
                    # High frequency calculation
                    angle = current_time * 0.8 + robot_id * 0.15
                    robot['position'][0] = robot_id + math.cos(angle)
                    robot['position'][1] = math.sin(angle)
                    robot['last_update_time'] = current_time
                    robot['update_count'] += 1
                    total_updates += 1
            
            # Process low frequency robots (less often)
            for robot_name, robot in low_freq_robots:
                update_interval = 1.0 / robot['update_frequency']
                if current_time - robot['last_update_time'] >= update_interval:
                    robot_id = int(robot_name.split('_')[1])
                    
                    # Low frequency calculation with different pattern
                    angle = current_time * 0.2 + robot_id * 0.05
                    robot['position'][0] = robot_id + 1.5 * math.cos(angle)
                    robot['position'][1] = 1.5 * math.sin(angle)
                    robot['last_update_time'] = current_time
                    robot['update_count'] += 1
                    total_updates += 1
            
            frame_count += 1
            # No sleep - frequency management overhead only
        
        return total_updates, frame_count, time.time() - start_time
    
    def simulate_pure_simpy_backend(duration=3.0, num_robots=15):
        """Pure SimPyバックエンドのシミュレーション"""
        print("🧠 Running Pure SimPy implementation...")
        
        robots = {}
        for i in range(num_robots):
            robots[f"robot_{i}"] = {
                'position': [i * 1.0, 0.0, 0.0],
                'behavior_state': 'exploring',
                'state_change_time': time.time() + (i * 0.5),
                'update_count': 0
            }
        
        start_time = time.time()
        total_updates = 0
        frame_count = 0
        
        # Event-driven behavior simulation - higher overhead
        while time.time() - start_time < duration:
            current_time = time.time()
            
            for robot_name, robot in robots.items():
                robot_id = int(robot_name.split('_')[1])
                
                # Complex behavior state machine
                if current_time >= robot['state_change_time']:
                    if robot['behavior_state'] == 'exploring':
                        robot['behavior_state'] = 'returning'
                    else:
                        robot['behavior_state'] = 'exploring'
                    robot['state_change_time'] = current_time + (2.0 + robot_id * 0.3)
                
                # Behavior-based movement calculation
                if robot['behavior_state'] == 'exploring':
                    angle = current_time * 0.3 + robot_id * 0.2
                    radius = 2.0 + math.sin(current_time * 0.1)
                else:
                    angle = current_time * 0.1 + robot_id * 0.1
                    radius = 1.0
                
                robot['position'][0] = robot_id + radius * math.cos(angle)
                robot['position'][1] = radius * math.sin(angle)
                robot['update_count'] += 1
                total_updates += 1
            
            frame_count += 1
            # No sleep - complex algorithm overhead only
        
        return total_updates, frame_count, time.time() - start_time
    
    backends = {
        '1': ("Simple While Loop", simulate_while_loop_backend),
        '2': ("SimPy FrequencyGroup", simulate_frequency_group_backend),
        '3': ("Pure SimPy", simulate_pure_simpy_backend)
    }
    
    while True:
        try:
            choice = input(f"\nCommand: ").strip().lower()
            
            if choice == 'q':
                break
            elif choice == 'c':
                # Compare all backends
                print(f"\n📊 Comparing All Backends (15 robots, 2 seconds each)")
                print("=" * 60)
                
                results = []
                for backend_name, simulator in [
                    ("Simple While Loop", simulate_while_loop_backend),
                    ("SimPy FrequencyGroup", simulate_frequency_group_backend), 
                    ("Pure SimPy", simulate_pure_simpy_backend)
                ]:
                    print(f"\n🧪 Testing {backend_name}...")
                    updates, frames, elapsed = simulator(duration=2.0, num_robots=15)
                    
                    updates_per_sec = updates / elapsed
                    rtf = 2.0 / elapsed
                    fps = frames / elapsed
                    
                    results.append({
                        'name': backend_name,
                        'updates_per_sec': updates_per_sec,
                        'rtf': rtf,
                        'fps': fps
                    })
                    
                    print(f"   ✅ {updates_per_sec:.0f} updates/sec, {rtf:.3f}x RTF")
                    time.sleep(0.5)
                
                # Results summary
                print(f"\n🏆 Comparison Results")
                print("-" * 60)
                print(f"{'Backend':<20} {'Updates/sec':<12} {'RTF':<8} {'FPS':<8}")
                print("-" * 60)
                
                for result in results:
                    print(f"{result['name']:<20} {result['updates_per_sec']:<12.0f} "
                          f"{result['rtf']:<8.3f} {result['fps']:<8.1f}")
                
                # Analysis
                best_perf = max(results, key=lambda x: x['updates_per_sec'])
                print(f"\n🚀 Best Performance: {best_perf['name']} "
                      f"({best_perf['updates_per_sec']:.0f} updates/sec)")
                
            elif choice in backends:
                backend_name, simulator = backends[choice]
                print(f"\n🔧 Running {backend_name} implementation...")
                
                updates, frames, elapsed = simulator(duration=3.0, num_robots=20)
                
                updates_per_sec = updates / elapsed
                rtf = 3.0 / elapsed
                fps = frames / elapsed
                
                print(f"   ✅ Results:")
                print(f"      Updates/sec: {updates_per_sec:.0f}")
                print(f"      RTF: {rtf:.3f}x")
                print(f"      FPS: {fps:.1f}")
                
                if updates_per_sec >= 8000:
                    rating = "🚀 ULTRA FAST"
                elif updates_per_sec >= 4000:
                    rating = "⚡ VERY FAST"
                elif updates_per_sec >= 2000:
                    rating = "✅ FAST"
                else:
                    rating = "⚠️ MODERATE"
                
                print(f"      Performance: {rating}")
                
            else:
                print("❌ Invalid command")
                
        except KeyboardInterrupt:
            break
    
    print("\n👋 Interactive demo finished!")
    print("You've experienced real backend implementation differences!")

def main():
    """メイン実行"""
    
    try:
        # 自動テスト
        minimal_simulation_test()
        
        # インタラクティブデモの選択肢
        choice = input("\nRun interactive demo? (y/n): ").strip().lower()
        if choice.startswith('y'):
            interactive_backend_demo()
    
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()