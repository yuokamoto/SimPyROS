# SimPyROS 開発状況 - Claude Code セッションメモ

## 最新状況（2025-08-06完了分）

### 完了した主要機能

#### 1. 基本シミュレーションフレームワーク
- **simulation_object.py**: SimPyベースの3Dオブジェクトシミュレーション
  - 双方向接続システム（親子関係なし）
  - クォータニオンベース3D回転（数値精度向上）
  - 静的オブジェクト制約（静的オブジェクトに接続されたオブジェクトは移動不可）
  - SimPy Environment統合

#### 2. 3D視覚化システム
- **visualizer.py**: matplotlib 3D可視化
  - リアルタイムアニメーション
  - 軌跡表示・座標フレーム表示
  - **マウス視点制御の永続化**（本日最後の改善）
  - ヘッドレス環境対応（自動PNG出力）
  - Ctrl+C対応のグレースフル終了

#### 3. プロジェクト構造整理
- **examples/フォルダ統合完了**
- **demos/ → examples/への統合**
- **詳細ドキュメント完備**

### ファイル構成

```
SimPyROS/
├── simulation_object.py      # メインフレームワーク
├── visualizer.py            # 3D視覚化（視点永続化対応）
├── requirements.txt         # 依存関係
├── .gitignore              # Git設定
├── Appendix.md             # 技術詳細・SimPy.rt比較
├── examples/
│   ├── README.md           # 全ファイル詳細説明
│   ├── example.py          # 基本機能デモ
│   ├── example_connected.py # 双方向接続デモ
│   ├── example_visualization.py # 3D視覚化デモ
│   ├── headless_realtime_demo.py # 手動制御リアルタイム（推奨）
│   ├── simple_realtime_demo.py # 手動制御基礎
│   ├── simpy_rt_demo.py    # SimPy.rt代替実装
│   ├── interactive_visualization_demo.py # インタラクティブ制御
│   └── view_control_demo.py # 視点制御テスト
├── tests/
│   ├── test_rotation_accuracy.py
│   ├── test_static_constraint.py
│   ├── test_visualization_static.py
│   ├── test_ctrl_c.py
│   └── test_realtime.py
└── output/                 # 生成PNG保存先
    ├── frame_XXX.png       # アニメーションフレーム
    └── *.png              # 各種出力画像
```

### 重要な設計判断

#### リアルタイム実装の2方式提供
1. **手動制御方式（推奨）**: `time.sleep()`ベース、高性能・クロスプラットフォーム
2. **SimPy.rt方式（代替）**: 教育・学習目的、プラットフォーム依存性あり

詳細は `Appendix.md` に技術的背景を記載済み。

### 最新の技術的改善

#### 1. 視点制御の永続化（本日最後の修正）
**問題**: アニメーション更新で手動視点変更がリセット
**解決**: 
```python
# visualizer.py内
def _update_plot(self, frame):
    # クリア前に視点を保存
    self._current_elev = self.ax.elev
    self._current_azim = self.ax.azim
    
    self.ax.clear()
    self._setup_plot()  # 保存視点で復元
```

#### 2. マウス操作サポート
- **左ドラッグ**: 視点回転（永続化対応）
- **右ドラッグ**: パン
- **マウスホイール**: ズーム
- **ダブルクリック**: 座標表示（一部デモ）

#### 3. キーボードショートカット
- `'p'`: 一時停止/再開
- `'r'`: 視点リセット
- `'c'`: 軌跡クリア
- `'t'`: 接続線表示切り替え
- `'q'`: 終了

### 動作確認済み機能

```bash
# 基本機能
python examples/example.py                    # ✅
python examples/example_connected.py         # ✅
python examples/test_static_constraint.py    # ✅

# 視覚化（視点制御改善済み）
python examples/example_visualization.py     # ✅
python examples/view_control_demo.py         # ✅

# リアルタイム実行
python examples/headless_realtime_demo.py quick  # ✅
python examples/simpy_rt_demo.py 1               # ✅
```

### 次回作業の推奨開始方法

#### 1. 状況確認
```bash
cd /home/rr/SimPyROS
python examples/example.py                    # 基本動作確認
python examples/view_control_demo.py          # 最新機能確認
```

#### 2. 必要に応じて依存関係確認
```bash
pip install -r requirements.txt
```

#### 3. ドキュメント参照
- `examples/README.md`: 実行方法と機能説明
- `Appendix.md`: 技術的背景
- このファイル（CLAUDE.md）: 開発履歴

### 今後の開発候補

#### 近い将来
- ロボットモデル読み込み（URDF/SDF対応強化）
- 物理エンジン統合検討
- パフォーマンス最適化

#### 中長期
- ROS2連携
- 複雑なシナリオテンプレート
- 機械学習連携

### 開発環境

- **OS**: Linux (Ubuntu系)
- **Python**: 3.x
- **主要依存**: simpy, scipy, matplotlib, numpy
- **推奨実行**: examples/フォルダから各デモを実行

---

## 作業再開時のチェックリスト

□ `cd /home/rr/SimPyROS`で作業ディレクトリに移動  
□ `python examples/example.py`で基本動作確認  
□ `python examples/view_control_demo.py`で最新機能確認  
□ 必要に応じて`pip install -r requirements.txt`  
□ 新機能開発前にCLAUDE.mdの内容を確認

**このファイル（CLAUDE.md）を参照すれば、前回までの作業内容と現在の状況が把握できます。**