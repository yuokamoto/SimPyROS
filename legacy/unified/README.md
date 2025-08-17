# 統一バックエンドデモ集

**SimPyROS統一インターフェース**を使用した高度なデモ集です。3つの異なるバックエンド（Simple While Loop、SimPy FrequencyGroup、Pure SimPy）を統一APIで切り替え可能なデモを提供します。

## 🎯 デモ概要

### 🟢 初心者向け - 基本的な統一機能

#### `minimal_test_demo.py` ⭐ **推奨開始点**
```bash
python examples/unified/minimal_test_demo.py
```
- **目的**: 依存関係なしで統一バックエンドの概念を理解
- **特徴**: scipy/PyVistaなしで動作、すべての環境で実行可能
- **学習内容**: 3つのバックエンドの性能差を体感
- **実行時間**: 約30秒

#### `quick_start_tutorial.py`
```bash  
python examples/unified/quick_start_tutorial.py
```
- **目的**: 統一APIの基本的な使用方法
- **特徴**: ステップバイステップのチュートリアル
- **学習内容**: 設定作成からシミュレーション実行まで
- **依存関係**: 基本ライブラリのみ

#### `simple_migration_example.py`
```bash
python examples/unified/simple_migration_example.py
```
- **目的**: 既存コードの統一API移行方法
- **特徴**: Before/Afterコード比較
- **学習内容**: レガシーコードの現代化手法
- **適用対象**: 既存SimPyプロジェクトの更新

### 🟡 中級者向け - 3D可視化とUI統合

#### `simple_unified_demo.py`
```bash
python examples/unified/simple_unified_demo.py
```
- **目的**: PyVista 3D可視化でのバックエンド切り替え
- **特徴**: リアルタイム3Dロボット表示、色分け表示
- **学習内容**: 可視化付きバックエンド比較
- **依存関係**: PyVista
- **制御**: キーボード（1,2,3でバックエンド切り替え）

#### `enhanced_pyvista_demo.py` ⭐ **高機能版**
```bash
python examples/unified/enhanced_pyvista_demo.py
```
- **目的**: フル機能PyVista統合デモ
- **特徴**: 高度なUIコントロール、統計表示、インタラクティブ操作
- **学習内容**: 本格的な可視化システム構築
- **依存関係**: PyVista + 既存SimPyROSライブラリ
- **制御**: マウス+キーボード+UI要素

#### `console_unified_demo.py`
```bash
python examples/unified/console_unified_demo.py
```
- **目的**: ヘッドレス環境でのバックエンド切り替え
- **特徴**: コンソールベースの操作、サーバー環境対応
- **学習内容**: GUI なしでの統一システム活用
- **依存関係**: 最小限（SimPy のみ）

### 🔴 上級者向け - カスタムUI と Webインターフェース

#### `interactive_ui_demo.py`
```bash
python examples/unified/interactive_ui_demo.py
```
- **目的**: Tkinter GUIによるリアルタイム制御
- **特徴**: グラフィカルコントロールパネル、リアルタイムグラフ
- **学習内容**: デスクトップアプリケーション統合
- **依存関係**: tkinter, matplotlib

#### `web_dashboard_demo.py`
```bash
python examples/unified/web_dashboard_demo.py
```
- **目的**: ブラウザベースのリアルタイムダッシュボード
- **特徴**: WebSocket通信、リアルタイム更新、マルチユーザー対応
- **学習内容**: Web技術との統合
- **依存関係**: flask, flask-socketio, eventlet
- **アクセス**: http://localhost:5000

#### `backend_comparison_demo.py`
```bash
python examples/unified/backend_comparison_demo.py
```
- **目的**: 詳細な性能比較とベンチマーク
- **特徴**: 複数条件での性能測定、統計分析
- **学習内容**: 科学的な性能評価手法
- **出力**: CSV結果、パフォーマンスレポート

#### `visual_comparison_demo.py`
```bash
python examples/unified/visual_comparison_demo.py
```
- **目的**: サイドバイサイドの視覚的比較
- **特徴**: 複数ウィンドウでの同時表示
- **学習内容**: 複数バックエンドの並列動作
- **依存関係**: PyVista（複数インスタンス）

### 🎯 統合デモ

#### `unified_pyvista_demo.py`
```bash
python examples/unified/unified_pyvista_demo.py
```
- **目的**: 統一システムの全機能デモンストレーション
- **特徴**: すべての機能を1つのデモに統合
- **学習内容**: システム全体の理解
- **対象**: 完全なシステム理解を求める上級者

## 🚀 実行順序の推奨

### 初回学習時
1. **`minimal_test_demo.py`** - 基本概念理解
2. **`quick_start_tutorial.py`** - API習得
3. **`simple_unified_demo.py`** - 3D可視化体験
4. **`enhanced_pyvista_demo.py`** - フル機能体験

### 特定目的別
- **ヘッドレス環境**: `console_unified_demo.py`
- **Web統合**: `web_dashboard_demo.py`
- **デスクトップアプリ**: `interactive_ui_demo.py`
- **性能分析**: `backend_comparison_demo.py`

## 🔧 依存関係

### 必須 (全デモ共通)
```bash
pip install simpy numpy
```

### オプション (機能別)
```bash
# 3D可視化用
pip install pyvista

# Web dashboard用  
pip install flask flask-socketio eventlet

# グラフィカルUI用
pip install matplotlib

# tkinterは通常Python標準ライブラリに含まれます
```

### 依存関係チェック
```bash
# デモランチャーで自動チェック
python ../../run_visual_demos.py

# 個別確認
python -c "import pyvista; print('PyVista: OK')"
python -c "import flask; print('Flask: OK')"
python -c "import matplotlib; print('Matplotlib: OK')"
```

## 🏆 性能比較結果例

```
バックエンド              コールバック/秒    RTF        特徴
Simple While Loop        21,000+           1.000x     最高性能
SimPy FrequencyGroup     7,000+            1.000x     バランス型  
Pure SimPy               3,000+            1.000x     高機能
```

## 🎮 制御方法

### キーボード制御 (PyVistaデモ)
- **1**: Simple While Loop バックエンド
- **2**: SimPy FrequencyGroup バックエンド  
- **3**: Pure SimPy バックエンド
- **Space**: 一時停止/再開
- **R**: カメラリセット
- **Q**: 終了

### コンソール制御
- **1/2/3**: バックエンド選択
- **p**: 一時停止切り替え
- **s**: 統計表示
- **q**: 終了

## 🆘 トラブルシューティング

### PyVistaが使えない場合
```bash
# 代替案: コンソール版使用
python console_unified_demo.py

# または: 最小限版使用  
python minimal_test_demo.py
```

### scipy/numpy互換性問題
```bash
# 仮想環境での解決
python -m venv clean-env
source clean-env/bin/activate
pip install --upgrade pip
pip install simpy numpy pyvista
```

### Webデモでポート問題
```bash
# ポート変更 (web_dashboard_demo.py内)
app.run(port=5001)  # 5000 → 5001
```

## 📊 学習効果

各デモで習得できるスキル：

1. **アーキテクチャ設計**: 異なるアプローチの比較分析
2. **性能最適化**: ボトルネックの特定と解決
3. **3D可視化**: インタラクティブな表示システム
4. **Web技術統合**: リアルタイム通信とUI
5. **ベンチマーク手法**: 科学的な性能評価

## 🎯 次のステップ

- **カスタムバックエンド**: 独自アーキテクチャの実装
- **ROS 2統合**: ロボット制御システムとの連携
- **分散シミュレーション**: クラウド環境での大規模実行
- **AI統合**: 機械学習アルゴリズムとの連携

---

**🎓 これらのデモを通じて、統一アーキテクチャの設計思想と実装技術を段階的に習得できます。目的と学習レベルに応じて適切なデモを選択してください！**