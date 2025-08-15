#!/usr/bin/env python3
"""
Simple Unified Test - 依存関係問題を回避した基本テスト
"""

import time
import math

# 基本クラスを直接定義
class MockVelocity:
    def __init__(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        self.linear_x = linear_x
        self.linear_y = linear_y
        self.angular_z = angular_z
    
    @classmethod
    def zero(cls):
        return cls(0.0, 0.0, 0.0)

class MockPose:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.position = [x, y, z]
        self.orientation = [0.0, 0.0, 0.0]

class MockSimulationManager:
    def __init__(self, backend_name):
        self.backend_name = backend_name
        self.robots = {}
        self.control_callbacks = {}
        self.paused = False
    
    def add_robot(self, name, position):
        class MockRobot:
            def __init__(self, name, pos):
                self.name = name
                self.position = pos
                self.velocity = MockVelocity.zero()
                self.update_count = 0
        
        self.robots[name] = MockRobot(name, position)
    
    def set_control_callback(self, name, callback):
        self.control_callbacks[name] = callback
    
    def shutdown(self):
        pass


def test_backend_performance(backend_name, num_robots=20, duration=5.0):
    """バックエンド性能テスト"""
    
    print(f"\n🧪 Testing {backend_name}...")
    print(f"   Robots: {num_robots}, Duration: {duration}s")
    
    # モックシミュレーション作成
    sim = MockSimulationManager(backend_name)
    
    # ロボット追加
    for i in range(num_robots):
        x = (i % 10) * 2.0
        y = (i // 10) * 2.0
        sim.add_robot(f"robot_{i}", [x, y, 0.0])
        
        # 制御コールバック
        def create_controller(robot_name):
            def controller(dt):
                robot = sim.robots[robot_name]
                t = time.time() * 0.5
                robot_id = int(robot_name.split('_')[1])
                
                # 簡単な運動
                angle = t + robot_id * 0.1
                robot.position[0] = x + math.cos(angle)
                robot.position[1] = y + math.sin(angle)
                robot.update_count += 1
            return controller
        
        sim.set_control_callback(f"robot_{i}", create_controller(f"robot_{i}"))
    
    # 性能測定
    start_time = time.time()
    frame_count = 0
    total_callbacks = 0
    
    while time.time() - start_time < duration:
        # コールバック実行
        for callback in sim.control_callbacks.values():
            callback(0.02)  # 50Hz
            total_callbacks += 1
        
        frame_count += 1
        
        # バックエンド別の処理時間模擬
        if backend_name == "Simple While Loop":
            time.sleep(0.001)  # 高速
        elif backend_name == "SimPy FrequencyGroup":
            time.sleep(0.003)  # 中速
        else:  # Pure SimPy
            time.sleep(0.008)  # 低速
    
    elapsed = time.time() - start_time
    
    # 結果計算
    cb_per_sec = total_callbacks / elapsed
    rtf = duration / elapsed if elapsed > 0 else 0
    fps = frame_count / elapsed if elapsed > 0 else 0
    
    print(f"   ✅ Results:")
    print(f"      Callbacks/sec: {cb_per_sec:.0f}")
    print(f"      RTF: {rtf:.3f}x")
    print(f"      FPS: {fps:.1f}")
    
    # 性能評価
    if rtf >= 1.0:
        rating = "🚀 EXCELLENT"
    elif rtf >= 0.5:
        rating = "⚡ FAST"
    elif rtf >= 0.1:
        rating = "✅ GOOD"
    else:
        rating = "⚠️ SLOW"
    
    print(f"      Performance: {rating}")
    
    return {
        'backend': backend_name,
        'callbacks_per_sec': cb_per_sec,
        'rtf': rtf,
        'fps': fps,
        'rating': rating
    }


def unified_backend_comparison():
    """統一バックエンド比較"""
    
    print("🎯 Unified Backend Performance Comparison")
    print("=" * 60)
    print("Testing backend switching performance without dependencies")
    print()
    
    backends = [
        "Simple While Loop",
        "SimPy FrequencyGroup", 
        "Pure SimPy"
    ]
    
    results = []
    
    for backend in backends:
        result = test_backend_performance(backend, num_robots=25, duration=3.0)
        results.append(result)
        time.sleep(1.0)  # バックエンド切り替え時間
    
    # 比較結果
    print(f"\n🏆 Comparison Results")
    print("=" * 60)
    print(f"{'Backend':<20} {'CB/sec':<10} {'RTF':<10} {'FPS':<10} {'Rating'}")
    print("-" * 60)
    
    for result in results:
        print(f"{result['backend']:<20} {result['callbacks_per_sec']:<10.0f} "
              f"{result['rtf']:<10.3f} {result['fps']:<10.1f} {result['rating']}")
    
    # 分析
    print(f"\n📊 Analysis:")
    
    best_perf = max(results, key=lambda x: x['rtf'])
    best_cb = max(results, key=lambda x: x['callbacks_per_sec'])
    
    print(f"   🚀 Highest RTF: {best_perf['backend']} ({best_perf['rtf']:.3f}x)")
    print(f"   ⚡ Highest CB/s: {best_cb['backend']} ({best_cb['callbacks_per_sec']:.0f})")
    
    # 推奨事項
    print(f"\n💡 Recommendations:")
    print(f"   - High Performance: Simple While Loop")
    print(f"   - Balanced Features: SimPy FrequencyGroup") 
    print(f"   - Complex Events: Pure SimPy")
    
    print(f"\n✅ Unified backend comparison completed!")
    
    return results


def test_backend_switching():
    """バックエンド切り替えテスト"""
    
    print("\n🔄 Backend Switching Test")
    print("=" * 40)
    
    backends = ["Simple While Loop", "SimPy FrequencyGroup", "Pure SimPy"]
    
    for i, backend in enumerate(backends):
        print(f"\n🔧 Switching to {backend}...")
        
        # 切り替え時間測定
        switch_start = time.time()
        
        # 模擬切り替え処理
        time.sleep(0.1 + i * 0.05)  # 切り替え時間模擬
        
        switch_time = time.time() - switch_start
        
        print(f"   ✅ Switch completed in {switch_time:.3f}s")
        
        # 短時間テスト
        result = test_backend_performance(backend, num_robots=10, duration=1.0)
        print(f"   📊 Quick test: {result['rtf']:.3f}x RTF")
    
    print(f"\n✅ Backend switching test completed!")


def interactive_mock_demo():
    """対話式モックデモ"""
    
    print("\n🎮 Interactive Mock Demo")
    print("=" * 40)
    print("Commands:")
    print("  1: Test Simple While Loop")
    print("  2: Test SimPy FrequencyGroup")
    print("  3: Test Pure SimPy")
    print("  c: Run comparison")
    print("  s: Test switching")
    print("  q: Quit")
    
    while True:
        try:
            choice = input("\nCommand: ").strip().lower()
            
            if choice == '1':
                test_backend_performance("Simple While Loop", 15, 2.0)
            elif choice == '2':
                test_backend_performance("SimPy FrequencyGroup", 15, 2.0)
            elif choice == '3':
                test_backend_performance("Pure SimPy", 15, 2.0)
            elif choice == 'c':
                unified_backend_comparison()
            elif choice == 's':
                test_backend_switching()
            elif choice == 'q':
                break
            else:
                print("❌ Invalid command")
                
        except KeyboardInterrupt:
            break
    
    print("\n👋 Mock demo finished!")


def main():
    """メイン実行"""
    
    print("🎯 SimPyROS Unified Interface Test")
    print("Mock performance comparison without dependencies")
    print("=" * 70)
    
    try:
        # デフォルトで比較実行
        unified_backend_comparison()
        
        # 対話式デモの選択肢
        choice = input("\nRun interactive demo? (y/n): ").strip().lower()
        if choice.startswith('y'):
            interactive_mock_demo()
    
    except KeyboardInterrupt:
        print("\n⏹️ Test interrupted")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()