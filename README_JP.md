# SimPyROS - 中央集権型離散イベントシミュレーションフレームワーク

**SimPyの中央集権アーキテクチャ**と**統合時間管理**を基盤とした強力なロボティクスシミュレーションフレームワーク。インタラクティブ3D可視化、URDFロボット支援、リアルタイム制御による高性能シミュレーションを提供します。

## 🚀 革新的な中央集権アーキテクチャ

SimPyROSは最適なパフォーマンスのために**中央集権更新管理**と単一シミュレーションループを活用しています：

```python
# 中央集権更新フロー（最新アーキテクチャ）
SimulationManager._simulation_process_loop():
    advance_sim_time() → current_sim_time
    for robot in robots:
        robot.update_joints_if_needed(current_sim_time)  # 100Hzジョイント制御
        robot.update_if_needed(current_sim_time)         # ベース動作更新
    for obj in objects:
        obj.update_if_needed(current_sim_time)           # カスタム更新頻度
    for callback in control_callbacks:
        callback.call_if_needed(current_sim_time)        # ユーザー制御ロジック
    yield env.timeout(time_step)  # 最適パフォーマンスのための単一yield
```

これにより**優れたパフォーマンス**（ヘッドレス1500+ Hz）、**簡素化されたデバッグ**、**精密なタイミング制御**を実現します。

## ✨ 主要機能

- 🤖 **URDFロボット対応**: 適切な運動学を持つ複雑なロボットの読み込み
- 🎮 **インタラクティブ3D可視化**: コントロール付きリアルタイムPyVistaレンダリング
- ⚡ **中央集権アーキテクチャ**: 最適パフォーマンスの単一ループ設計
- 🔗 **マルチロボット対応**: 独立した更新頻度を持つ複数ロボット
- 🎯 **簡素化インターフェース**: 完全なシミュレーションが〜20行
- 📊 **リアルタイム制御**: 動的速度、一時停止/再開、リセット機能
- ⏱️ **統合時間管理**: 全コンポーネントでの中央集権sim_timeアクセス
- 📈 **リアルタイムモニター**: ライブ更新付きシミュレーション統計ウィンドウ
- 🧪 **高性能**: ヘッドレス1500+ Hz、可視化付き60+ Hz
- 🔧 **処理時間補償**: 正確なリアルタイム同期
- 🛡️ **X11エラーフリー**: 全プラットフォームで堅牢なディスプレイ処理
- 📝 **包括的ログ機能**: 設定可能レベルと日本語対応の構造化ログ

## 🚀 クイックスタート

### 1. 環境セットアップ

```bash
# クローンとセットアップ
git clone <repository-url>
cd SimPyROS

# 仮想環境作成（推奨）
python -m venv simpyros-env
source simpyros-env/bin/activate  # Linux/macOS
# OR: simpyros-env\Scripts\activate  # Windows

# 依存関係のインストール
pip install --upgrade pip
pip install -r requirements.txt
```

### 2. 初回シミュレーション実行

```bash
# 環境アクティベート
source simpyros-env/bin/activate

# 基本から始める
python examples/beginner/basic_simulation.py --vis

# リアルタイム統計用モニターウィンドウ付き
python examples/beginner/basic_simulation.py --vis --enable-monitor

# 高度なマルチロボット体験
python examples/advanced/all_features_demo.py --vis
```

### 3. シンプルなコード例

```python
from core.simulation_manager import SimulationManager
import math

# 1. シミュレーション作成
sim = SimulationManager()

# 2. ロボット追加
robot = sim.add_robot_from_urdf("my_robot", "robots/articulated_arm_robot.urdf")

# 3. 制御定義
def control(dt):
    sim_time = sim.get_sim_time()
    for joint_name in robot.get_joint_names():
        position = math.sin(sim_time)
        sim.set_robot_joint_position("my_robot", joint_name, position)

# 4. 実行
sim.set_robot_control_callback("my_robot", control)
sim.run(duration=10.0, visualization=True)
```

**これだけです！** 完全なシミュレーションが〜15行で実現できます。

## 📝 ログ設定

SimPyROSには設定可能な出力レベルを持つ包括的なログシステムが含まれています：

### 環境変数
```bash
# ログの詳細度制御
export SIMPYROS_DEBUG=0  # 警告とエラーのみ
export SIMPYROS_DEBUG=1  # 通常出力（デフォルト）
export SIMPYROS_DEBUG=2  # 詳細デバッグ出力

# Python内での設定
import os
os.environ['SIMPYROS_DEBUG'] = '2'
```

### プログラム制御
```python
from core.logger import set_log_level, enable_debug, suppress_verbose

# 特定レベル設定
set_log_level('DEBUG')    # 詳細デバッグ
set_log_level('INFO')     # 通常情報
set_log_level('WARNING')  # 警告とエラーのみ

# 便利関数
enable_debug()           # デバッグログ有効化
suppress_verbose()       # 警告/エラーのみ表示
```

### コード内でのログ使用
```python
from core.logger import get_logger, log_success, log_warning, log_error

logger = get_logger('your_module')
log_success(logger, "操作が正常に完了しました")
log_warning(logger, "警告メッセージ")
log_error(logger, "エラーが発生しました")
```

## 🏗️ アーキテクチャ比較

### 前：独立プロセスアーキテクチャ（レガシー）
```python
# 複数独立SimPyプロセス（レガシーアプローチ）
robot.start_joint_control_process()    # 100Hzジョイント制御
robot.start_sensor_process()          # 30Hzセンサー処理
robot.start_navigation_process()      # 10Hz自律ナビゲーション
robot.start_base_motion_process()     # 100Hzベース動作

# 問題: プロセス同期の複雑さ、低パフォーマンス
```

### 後：中央集権更新管理（現在）
```python
# 直接メソッド呼び出しによる単一中央集権ループ
SimulationManager._simulation_process_loop():
    for robot in robots: robot.update_joints_if_needed(sim_time)
    for obj in objects: obj.update_if_needed(sim_time)
    yield env.timeout(time_step)  # システム全体で唯一のyield

# 利点: 優れたパフォーマンス、簡素化されたデバッグ、精密なタイミング
```

## 📁 例の構造

例は難易度別に整理されています：

### 🟢 [初心者](examples/beginner/) - ここから始める
- **[basic_simulation.py](examples/beginner/basic_simulation.py)** - 必須の最初の例
  - 動作する単一ロボット
  - インタラクティブ可視化制御
  - 中央集権アーキテクチャへの完璧な導入

### 🟡 [中級者](examples/intermediate/) - スキルアップ  
- **[link_connections.py](examples/intermediate/link_connections.py)** - オブジェクト接続
- **[mesh_robots.py](examples/intermediate/mesh_robots.py)** - TurtleBot3、UR5統合
- **[pyvista/](examples/intermediate/pyvista/)** - 直接可視化制御

### 🔴 [上級者](examples/advanced/) - フレームワークマスター
- **[all_features_demo.py](examples/advanced/all_features_demo.py)** ⭐ **注目**
  - **全機能ショーケース** 全可視化バックエンド付き
  - **複数ロボット協調** デモンストレーション
  - **パフォーマンス比較** バックエンド間
- **[advanced_simpy_demo.py](examples/advanced/advanced_simpy_demo.py)** - 高度なSimPyパターン

## 🎯 学習パス

1. **[basic_simulation.py](examples/beginner/basic_simulation.py)** - 中央集権アーキテクチャの基礎を学ぶ
2. **[link_connections.py](examples/intermediate/link_connections.py)** - オブジェクト関係  
3. **[mesh_robots.py](examples/intermediate/mesh_robots.py)** - 外部ロボットモデル
4. **[all_features_demo.py](examples/advanced/all_features_demo.py)** - 完全機能ショーケース

## 🔧 システム要件

- **Python 3.8+** （推奨：3.9+）
- **プラットフォーム**: Linux、macOS、Windows
- **依存関係**: SimPy、PyVista、NumPy、SciPy

## 🆘 トラブルシューティング

### ディスプレイ問題
```bash
# ディスプレイ確認
echo $DISPLAY

# ヘッドレスサーバー用
export DISPLAY=:0

# モニターウィンドウ問題（X11 RENDERエラー）
# --enable-monitorフラグは一部システムで慎重に使用
# エラーが発生する場合はモニターなしで実行: (--enable-monitorを除外)
```

### インストール問題
```bash
# クリーンインストール
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt --no-cache-dir

# 代替: conda
conda install -c conda-forge vtk pyvista
```

## 📚 ドキュメント

- **[例ガイド](examples/README.md)** - 完全な使用例
- **[CLAUDE.md](CLAUDE.md)** - 開発履歴と詳細アーキテクチャ
- **[レガシーコード](legacy/)** - 参考用の以前の実装

## 📄 ライセンス

[ライセンス情報をここに追加してください]

---

**🎯 SimPyROSは、統合時間制御を持つ中央集権更新管理が複雑なマルチプロセス設計よりも優れていることを実証しています。パフォーマンスの違いを体験してください！**

*最新更新: 2025年8月 - 統合時間管理と処理時間補償による革新的中央集権アーキテクチャ*