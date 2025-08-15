#!/usr/bin/env python3
"""
SimPyROS - 統一シミュレーションインターフェース
Simple While Loop / SimPy FrequencyGroup / Pure SimPy を切り替え可能
"""

# メインクラスのエクスポート
from core.base_simulation_manager import (
    BaseSimulationManager,
    UnifiedSimulationConfig,
    SimulationBackend,
    create_simulation_manager,
    quick_simulation,
    compare_backends
)

from core.simulation_object import Velocity, Pose

# 便利なプリセット設定
def create_high_performance_config(visualization: bool = False, 
                                  num_robots: int = 100) -> UnifiedSimulationConfig:
    """高性能優先設定"""
    return UnifiedSimulationConfig(
        backend=SimulationBackend.SIMPLE_WHILE_LOOP,
        visualization=visualization,
        update_rate=50.0 if not visualization else 30.0,
        real_time_factor=0.0,  # 最高速度
        verbose=False,
        performance_monitoring=True
    )

def create_balanced_config(visualization: bool = True) -> UnifiedSimulationConfig:
    """バランス設定（機能と性能のトレードオフ）"""
    return UnifiedSimulationConfig(
        backend=SimulationBackend.SIMPY_FREQUENCY_GROUP,
        visualization=visualization,
        update_rate=30.0,
        real_time_factor=1.0,
        enable_frequency_grouping=True,
        verbose=True,
        performance_monitoring=True
    )

def create_feature_rich_config(visualization: bool = True) -> UnifiedSimulationConfig:
    """高機能優先設定（複雑なイベント処理対応）"""
    return UnifiedSimulationConfig(
        backend=SimulationBackend.SIMPY_PURE,
        visualization=visualization,
        update_rate=20.0,
        real_time_factor=1.0,
        enable_frequency_grouping=False,
        verbose=True,
        performance_monitoring=True
    )

# 簡単な実行用関数
def run_quick_demo(num_robots: int = 10, 
                  backend: SimulationBackend = SimulationBackend.SIMPLE_WHILE_LOOP,
                  duration: float = 5.0,
                  visualization: bool = False):
    """クイックデモ実行"""
    
    print(f"🎯 SimPyROS Quick Demo")
    print(f"Backend: {backend.value}")
    print(f"Robots: {num_robots}")
    print(f"Visualization: {'ON' if visualization else 'OFF'}")
    print("=" * 50)
    
    stats = quick_simulation(
        num_robots=num_robots,
        backend=backend,
        visualization=visualization,
        duration=duration
    )
    
    print(f"\n📊 Demo Results:")
    print(f"RTF: {stats['rtf']:.3f}x")
    print(f"Callbacks/sec: {stats['callbacks_per_sec']:.1f}")
    print(f"Performance: {'🚀 EXCELLENT' if stats['rtf'] >= 1.0 else '✅ GOOD' if stats['rtf'] >= 0.1 else '⚠️ FAIR'}")
    
    return stats

def benchmark_all_backends(num_robots: int = 20, duration: float = 3.0):
    """全バックエンドのベンチマーク"""
    
    print(f"🏁 SimPyROS Backend Benchmark")
    print(f"Testing with {num_robots} robots for {duration}s each")
    print("=" * 60)
    
    results = compare_backends(num_robots=num_robots, duration=duration)
    
    print(f"\n🏆 Benchmark Results:")
    print(f"{'Backend':<25} {'RTF':<10} {'CB/sec':<12} {'Rating'}")
    print("-" * 55)
    
    for backend, stats in results.items():
        if stats:
            rtf = stats['rtf']
            cb_per_sec = stats['callbacks_per_sec']
            
            if rtf >= 1.0:
                rating = "🚀 ULTRA"
            elif rtf >= 0.5:
                rating = "⚡ FAST"  
            elif rtf >= 0.1:
                rating = "✅ GOOD"
            else:
                rating = "⚠️ SLOW"
            
            print(f"{backend:<25} {rtf:<10.3f} {cb_per_sec:<12.1f} {rating}")
        else:
            print(f"{backend:<25} {'FAILED':<10} {'N/A':<12} ❌")
    
    return results

# 使用例とヘルプ
def print_usage_examples():
    """使用例表示"""
    
    print("""
🎯 SimPyROS - 統一ロボットシミュレーションフレームワーク

## 基本的な使用例:

### 1. 高性能シミュレーション (Simple While Loop)
```python
import simpyros

# 高性能設定
config = simpyros.create_high_performance_config(visualization=False)
sim = simpyros.create_simulation_manager(config)

# ロボット追加
robot = sim.add_robot_from_urdf("robot1", "path/to/robot.urdf")
sim.set_robot_control_callback("robot1", my_controller, frequency=10.0)

# 実行
sim.run(duration=10.0)
sim.print_performance_summary()
```

### 2. バランス型シミュレーション (SimPy FrequencyGroup)
```python
config = simpyros.create_balanced_config(visualization=True)
sim = simpyros.create_simulation_manager(config)
# ... (same as above)
```

### 3. 高機能シミュレーション (Pure SimPy)
```python
config = simpyros.create_feature_rich_config()
sim = simpyros.create_simulation_manager(config)
# ... (same as above)
```

### 4. クイック実行
```python
# 10台ロボット、Simple While Loop、5秒実行
stats = simpyros.quick_simulation(
    num_robots=10, 
    backend=simpyros.SimulationBackend.SIMPLE_WHILE_LOOP,
    duration=5.0
)
```

### 5. バックエンド比較
```python
results = simpyros.compare_backends(num_robots=50)
```

## バックエンド選択指針:

- **Simple While Loop**: 最高性能、シンプル、100+台ロボット (RTF ~1.0x)
- **SimPy FrequencyGroup**: バランス、中規模、リアルタイム制御 (RTF ~0.1-0.5x)  
- **Pure SimPy**: 高機能、複雑なイベント処理、小規模 (RTF ~0.05x)

## パフォーマンスガイド:

- 100+台ロボット → Simple While Loop
- 10-50台ロボット + 可視化 → SimPy FrequencyGroup
- 複雑なイベント処理 → Pure SimPy
""")

if __name__ == "__main__":
    print_usage_examples()
    
    # 簡単なデモ実行
    print("\n" + "="*50)
    print("Running quick demo...")
    run_quick_demo(num_robots=5, duration=2.0)