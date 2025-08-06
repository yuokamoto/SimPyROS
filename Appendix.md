# SimPyROS Appendix

## SimPy.rt を使わない理由について

### 概要

SimPyROSでは通常の`simpy.Environment`を使用し、`simpy.rt.RealtimeEnvironment`は使用していません。以下にその理由と技術的背景を説明します。

### SimPy.rt（リアルタイム環境）とは

SimPyのリアルタイム環境は、シミュレーション時間と実際の時間（壁時計時間）を同期させる実行環境です。

```python
# 通常のSimPy環境
env = simpy.Environment()  # 最大速度で実行

# リアルタイム環境
env = simpy.rt.RealtimeEnvironment(factor=1.0, strict=True)  # 実時間と同期
```

### 主要な違い

#### 1. 時間同期
- **通常環境**: 計算能力の限界まで高速実行
- **リアルタイム環境**: 実時間との同期によりオーバーヘッドが発生

#### 2. 設定パラメータ
```python
simpy.rt.RealtimeEnvironment(
    initial_time=0,
    factor=1.0,      # 実時間倍率（1.0=等倍、0.1=10倍速、60=1/60倍速）
    strict=True      # 計算時間超過時にエラー発生
)
```

#### 3. エラーハンドリング
- `strict=True`: 計算時間が実時間を超過するとRuntimeError
- `strict=False`: エラーを無視して継続実行

### SimPyROSでsimpy.rtを使わない理由

#### 1. パフォーマンス重視
```python
# 現在の実装：最大速度実行
env = simpy.Environment()
env.run(until=simulation_time)  # 最速実行

# リアルタイム同期のオーバーヘッドを回避
```

#### 2. プラットフォーム依存性の問題
- **Windows**: 比較的良好な同期精度
- **Linux/Unix**: 同期精度が劣る
- **原因**: Pythonの時間処理がプラットフォーム非依存ではない

#### 3. 時間精度の制限
```python
# simpy.rtの制約
# - 時間精度は約100ms程度が限界
# - 高精度シミュレーションには不適切
```

#### 4. 柔軟な制御の実現
現在の実装では手動でリアルタイム制御を行っています：

```python
# examples/headless_realtime_demo.py より
start_time = time.time()
while True:
    current_time = time.time()
    sim_time = current_time - start_time
    
    if sim_time >= duration:
        break
        
    # シミュレーション更新
    update_motion(sim_time)
    
    # フレームレート制御
    time.sleep(0.1)  # 10 FPS
```

この方式の利点：
- **必要時のみリアルタイム制御**
- **フレームレート調整可能**
- **ヘッドレス環境対応**
- **プラットフォーム非依存**

#### 5. 用途適合性

**simpy.rt が適している場面:**
- ハードウェア連携（Hardware-in-the-Loop）
- 人間とのリアルタイムインタラクション
- デモンストレーション用途
- 分散最適化アルゴリズムの実時間解析

**現在の実装が適している場面:**
- 高速シミュレーション実行
- バッチ処理・大量計算
- 研究・分析用途
- 可視化とファイル出力

### パフォーマンス比較

```python
# 速度比較例（概算）
# 通常環境: 1000ステップ/秒
# リアルタイム環境: factor=1.0 → 1ステップ/秒（1000倍遅い）

# メモリ使用量
# 通常環境: 基本SimPyのみ
# リアルタイム環境: 同期処理のオーバーヘッド
```

### 技術的制約

#### Python固有の制限
```python
# Pythonの制約
# - リアルタイム用途に設計されていない
# - 時間精度は100ms程度が限界
# - 非同期実装ではないため、実行中はブロッキング
```

#### スレッド使用時の考慮点
```python
# より複雑な操作をsimpy外でモデル化し、
# リアルタイム部分を小さく高速に保つためにスレッドが有効
```

### 実装上の判断

SimPyROSでは以下の判断により通常環境を採用：

1. **高速シミュレーション**: 研究・開発での反復実行
2. **クロスプラットフォーム**: Linux/Unix環境での安定動作
3. **柔軟性**: 用途に応じたリアルタイム制御
4. **分析重視**: 大量データ処理とバッチ実行

### 両方式の実装を提供

SimPyROSでは学習と比較のため、両方の実装を提供しています：

#### 1. 手動制御方式（推奨）
**ファイル:** `examples/headless_realtime_demo.py`, `examples/simple_realtime_demo.py`

```python
# 手動制御の実装例
start_time = time.time()
while True:
    current_time = time.time()
    sim_time = current_time - start_time
    
    # シミュレーション更新
    update_motion(sim_time)
    
    # フレームレート制御
    time.sleep(0.1)  # 10 FPS
```

**特徴:**
- 高いパフォーマンス
- プラットフォーム非依存
- 柔軟な制御
- 研究・本格運用に適している

#### 2. SimPy.rt方式（代替実装）
**ファイル:** `examples/simpy_rt_demo.py`

```python
# SimPy.rt実装例
class RealtimeSimulationRunner:
    def __init__(self, time_scale=1.0):
        self.env = simpy.rt.RealtimeEnvironment(
            factor=time_scale,
            strict=False
        )
    
    def time_scale_controller(self):
        # 動的時間スケール調整
        yield self.env.timeout(2.0)
        self.env.factor = self.time_scale
```

**特徴:**
- SimPy組み込み機能
- シンプルな実装
- プラットフォーム依存性あり
- 教育・デモに適している

#### 実装選択のガイドライン

```python
# 用途別推奨実装
use_cases = {
    "研究・分析": "手動制御方式",
    "大量データ処理": "手動制御方式", 
    "本格運用": "手動制御方式",
    "教育・学習": "両方を体験",
    "プロトタイプ": "SimPy.rt方式",
    "Windows環境デモ": "SimPy.rt方式",
    "Linux/Unix環境": "手動制御方式"
}
```

### 将来的な拡張可能性

統合実装も可能：

```python
# 統合実装例
class FlexibleSimulationRunner:
    def __init__(self, realtime_method='manual', factor=1.0):
        self.method = realtime_method
        if realtime_method == 'simpy_rt':
            self.env = simpy.rt.RealtimeEnvironment(factor=factor)
        else:
            self.env = simpy.Environment()
            self.manual_factor = factor
```

### 参考文献

- [SimPy Real-time simulations](https://simpy.readthedocs.io/en/latest/topical_guides/real-time-simulations.html)
- [simpy.rt API Reference](https://simpy.readthedocs.io/en/latest/api_reference/simpy.rt.html)
- [CBE 41622/61622 Laboratory: Scheduling Real-Time Events with Simpy](https://jckantor.github.io/cbe61622/notebooks/00/A.04-Scheduling-Real-Time-Events-with-Simpy.html)

### 実例による比較

SimPyROSでは両方式を実装しているため、実際に体験して比較できます：

#### パフォーマンス比較実験
```bash
# 手動制御方式（高速）
time python examples/headless_realtime_demo.py quick
# → 約5秒で完了

# SimPy.rt方式（リアルタイム同期）
time python examples/simpy_rt_demo.py 1
# → factor=1.0の場合、約30秒で完了
```

#### プラットフォーム互換性テスト
```bash
# Linux環境での精度比較
python examples/simple_realtime_demo.py     # 安定
python examples/simpy_rt_demo.py 1          # 同期精度に課題あり

# Windows環境での精度比較  
python examples/simple_realtime_demo.py     # 安定
python examples/simpy_rt_demo.py 1          # 比較的良好な同期
```

### 学習のための推奨順序

1. **基本理解**: `examples/simple_realtime_demo.py` で手動制御方式を理解
2. **比較体験**: `examples/simpy_rt_demo.py 1` でSimPy.rt方式を体験  
3. **実用検討**: 用途に応じて適切な方式を選択

### まとめ

SimPyROSでは**パフォーマンス、柔軟性、プラットフォーム互換性を重視し、手動制御方式を推奨**しています。ただし、教育目的や特定用途でのSimPy.rt方式も理解できるよう、両方の実装を提供しています。

**推奨指針:**
- **一般用途・本格運用**: 手動制御方式
- **学習・比較**: 両方を体験
- **Windows環境でのシンプルなデモ**: SimPy.rt方式も選択肢