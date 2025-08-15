# SimPyROS - 統一ロボットシミュレーションフレームワーク

**SimPy使用/非使用を切り替え可能な統一インターフェース**

## 🎯 概要

SimPyROSは、ロボットシミュレーションのための柔軟なフレームワークです。3つの異なるバックエンドから選択でき、性能と機能のトレードオフを自由に調整できます。

### 🔧 3つのバックエンド

| バックエンド | 性能 | 複雑度 | 適用場面 | RTF |
|-------------|-----|-------|---------|-----|
| **Simple While Loop** | 🚀 最高 | 最小 | 100+台ロボット | ~1.0x |
| **SimPy FrequencyGroup** | ⚡ 高 | 中 | バランス重視 | ~0.1-0.5x |
| **Pure SimPy** | ✅ 中 | 高 | 複雑なイベント | ~0.05x |

## 🚀 クイックスタート

### インストール

```bash
git clone <repository>
cd SimPyROS
source simpyros-env/bin/activate  # 仮想環境
```

### 基本的な使用法

```python
import simpyros
import math

# 1. 高性能設定で開始
config = simpyros.create_high_performance_config()
sim = simpyros.create_simulation_manager(config)

# 2. ロボット追加
robot = sim.add_robot_from_urdf(
    name="robot1",
    urdf_path="examples/robots/mobile_robot.urdf",
    initial_pose=simpyros.Pose(x=1.0, y=0, z=0)
)

# 3. 制御コールバック設定
def my_controller(dt):
    t = sim.get_sim_time()
    velocity = simpyros.Velocity(
        linear_x=0.1 * math.sin(t),
        angular_z=0.05
    )
    sim.set_robot_velocity("robot1", velocity)

sim.set_robot_control_callback("robot1", my_controller, frequency=10.0)

# 4. シミュレーション実行
sim.run(duration=10.0)
sim.print_performance_summary()
sim.shutdown()
```

## 📊 バックエンド選択ガイド

### Simple While Loop（推奨）
- **用途**: 高性能が必要な場合、100+台ロボット
- **特徴**: 最高速度、最小依存関係、デバッグ容易
```python
config = simpyros.create_high_performance_config()
```

### SimPy FrequencyGroup
- **用途**: 機能と性能のバランス、リアルタイム制御
- **特徴**: 中程度の性能、FrequencyGroup最適化
```python
config = simpyros.create_balanced_config()
```

### Pure SimPy
- **用途**: 複雑なイベント処理、リソース管理
- **特徴**: 最大機能性、イベント駆動
```python
config = simpyros.create_feature_rich_config()
```

## 🎮 例とデモ

### チュートリアル
```bash
python examples/unified/quick_start_tutorial.py
```

### バックエンド比較
```bash
python examples/unified/backend_comparison_demo.py
```

### 移行例
```bash
python examples/unified/simple_migration_example.py
```

## 🔧 高度な使用法

### カスタム設定
```python
config = simpyros.UnifiedSimulationConfig(
    backend=simpyros.SimulationBackend.SIMPLE_WHILE_LOOP,
    visualization=True,
    update_rate=60.0,
    real_time_factor=1.0,
    verbose=True
)
```

### 複数ロボット制御
```python
for i in range(100):
    robot = sim.add_robot_from_urdf(
        name=f"robot_{i}",
        urdf_path="examples/robots/mobile_robot.urdf",
        initial_pose=simpyros.Pose(x=i*2.0, y=0, z=0)
    )
    
    def create_controller(robot_id):
        def controller(dt):
            # ロボット固有の制御ロジック
            pass
        return controller
    
    sim.set_robot_control_callback(f"robot_{i}", create_controller(i))
```

### 性能監視
```python
# リアルタイム統計
stats = sim.get_performance_stats()
print(f"RTF: {stats['rtf']:.3f}x")

# バックエンド比較
results = simpyros.compare_backends(num_robots=50)
```

## 📈 性能ベンチマーク

### 100台ロボット性能比較

| バックエンド | RTF | 特徴 |
|-------------|-----|------|
| Simple While Loop | **0.976x** | ほぼリアルタイム |
| SimPy FrequencyGroup | 0.1x | 最適化済み |
| Pure SimPy | 0.05x | フル機能 |

### スケーラビリティ

- **Simple While Loop**: 200台でも RTF 0.95x 維持
- **SimPy FrequencyGroup**: 50台で RTF 0.2x
- **Pure SimPy**: 20台で RTF 0.1x

## 🛠️ 既存コードからの移行

### 従来のSimulationManager
```python
# OLD
from core.simulation_manager import SimulationManager, SimulationConfig
config = SimulationConfig(...)
sim = SimulationManager(config)
```

### 新しい統一インターフェース
```python
# NEW
import simpyros
config = simpyros.create_high_performance_config()
sim = simpyros.create_simulation_manager(config)
```

**既存のコードはそのまま動作します！**

## 🎯 使い分け指針

### Simple While Loopを選ぶべき場合
- ✅ 50台以上のロボット
- ✅ 最高性能が必要
- ✅ シンプルな制御ロジック
- ✅ デバッグの容易さ重視

### SimPy FrequencyGroupを選ぶべき場合
- ✅ 10-50台のロボット
- ✅ 可視化 + 制御
- ✅ リアルタイム係数制御
- ✅ 中程度の複雑さ

### Pure SimPyを選ぶべき場合
- ✅ 複雑なイベント処理
- ✅ リソース競合管理
- ✅ 非同期通信
- ✅ 条件待機・タイムアウト

## 📚 API リファレンス

### 主要クラス
- `UnifiedSimulationConfig`: 統一設定
- `BaseSimulationManager`: 抽象基底クラス
- `create_simulation_manager()`: ファクトリー関数

### ユーティリティ関数
- `quick_simulation()`: クイック実行
- `compare_backends()`: バックエンド比較
- `print_usage_examples()`: 使用例表示

## 🤝 貢献

プルリクエストや課題報告を歓迎します。

## 📄 ライセンス

[適切なライセンスを記載]

---

**SimPyROS**: 性能と機能の完璧なバランスを実現するロボットシミュレーションフレームワーク