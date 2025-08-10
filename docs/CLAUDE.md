# SimPyROS 開発状況 - Claude Code セッションメモ

## 最新状況（2025-08-07完了分）

### 今日完了した主要改善

#### 1. プロジェクト構成の大幅整理
- **examplesフォルダ統合**: 17個のファイル → 7個の構造化デモ
- **重複ファイル削除**: 機能が重複していた古いデモファイルを統合・削除
- **英語統一**: プログラム内コメント・出力メッセージ・ドキュメントを英語に統一

#### 2. Pose表示の改善
- **数値表示対応**: `__str__`メソッド追加
- **Before**: `<simulation_object.Pose object at 0x...>`
- **After**: `Pose(pos=(2.000, 1.000, 0.000), rot=(0.0°, 0.0°, 45.0°))`

#### 3. visualization_demo.pyの重要な修正
- **Real-time factor追加**: アニメーション速度を制御可能
  - `python visualization_demo.py 0.5` (半分の速度)
  - `python visualization_demo.py 2.0` (倍速)
- **センサー設定修正**: STATIC → DYNAMIC（robotの動作を可能に）
- **コード構造改善**: 重複するwhileループを関数化
- **Phase 4座標系修正**: world座標とlocal座標の混同を修正

#### 4. Phase 4ターゲット接近の根本的修正
**問題**: Robotがtargetに向かわない（逆方向に移動）
**原因**: 速度をロボットのローカル座標で指定するのに、方向をワールド座標で計算
**解決**: ワールド座標からローカル座標への適切な変換

```python
# ❌ 修正前（間違い）
world_direction = target_pos - robot_pos
robot.set_velocity(Velocity(linear_x=world_direction[0], ...))

# ✅ 修正後（正しい）
world_direction = (target_pos - robot_pos) / distance
local_direction = robot.pose.rotation.inv().apply(world_direction)
robot.set_velocity(Velocity(linear_x=local_direction[0], ...))
```

### 完了した主要機能

#### 1. 基本シミュレーションフレームワーク
- **simulation_object.py**: SimPyベースの3Dオブジェクトシミュレーション
  - 双方向接続システム（親子関係なし）
  - クォータニオンベース3D回転（数値精度向上）
  - 静的オブジェクト制約（静的オブジェクトに接続されたオブジェクトは移動不可）
  - **改善されたPose表示**: 数値が読みやすい形式
  - SimPy Environment統合

#### 2. 3D視覚化システム（大幅改善済み）
- **visualizer.py**: matplotlib 3D可視化
- **visualization_demo.py**: リアルタイム制御対応
  - **Real-time factorによる速度制御**
  - **正確なタイミング同期**: 瞬時実行→スムーズアニメーション
  - **Phase-based simulation**: 構造化された4段階フェーズ
  - **Dynamic target approach**: 距離監視による適応的停止
  - **Coordinate system fix**: ワールド・ローカル座標の適切な変換

#### 3. PyVista高品質3D視覚化
- **pyvista_simple_demo.py**: VTKベース高品質レンダリング
- 3Dロボットメッシュ生成と表示
- ヘッドレス環境対応（X11エラー時の自動フォールバック）

### ファイル構成（PyVista主体に整理完了 - 2025-08-09）

```
SimPyROS/
├── simulation_object.py      # メインフレームワーク（Pose表示改善済み）
├── requirements.txt         # 依存関係
├── .gitignore              # Git設定
├── Appendix.md             # 技術詳細・SimPy.rt比較
├── CLAUDE.md               # 開発履歴（このファイル）
├── examples/               # PyVistaメイン（3ファイル）
│   ├── README.md           # PyVista主体の説明書
│   ├── pyvista_robot_demo.py # インタラクティブ3Dデモ（メイン）
│   ├── pyvista_simple_demo.py # 画像生成・テスト用
│   └── realtime_demo.py    # マルチロボット・データ出力
├── legacy/                 # レガシーコード保管庫
│   ├── README.md           # レガシー説明
│   ├── visualizer.py       # matplotlib視覚化（旧メイン）
│   ├── examples/           # matplotlib系デモ
│   ├── backends/           # 旧バックエンド群
│   └── tests/              # 旧テストファイル
├── tests/                  # 現行テストファイル
└── output/                 # 生成ファイル保存先
    ├── pyvista_*.png      # PyVista画像出力
    └── *.json             # データ出力
```

### 重要な技術的修正

#### 1. Real-time Factor Implementation
```python
# 瞬時実行を防止し、指定速度でアニメーション実行
def run_simulation_phase(end_time, phase_setup=None, dynamic_check=None):
    while env.now < end_time:
        env.run(until=env.now + dt)
        
        # Real-time synchronization
        elapsed_real_time = time.time() - sim_start_time
        expected_sim_time = elapsed_real_time * real_time_factor
        
        if env.now > expected_sim_time:
            sleep_time = (env.now - expected_sim_time) / real_time_factor
            time.sleep(sleep_time)
```

#### 2. 座標系変換の正確な実装
```python
# ワールド座標の方向をロボットのローカル座標に変換
world_direction = (target_pos - robot_pos) / distance
local_direction = robot.pose.rotation.inv().apply(world_direction)
robot.set_velocity(Velocity(linear_x=local_direction[0] * speed, ...))
```

#### 3. 動的フェーズ制御
```python
# 距離に基づく早期終了機能
def check_target_distance():
    current_distance = np.linalg.norm(target.pose.position - robot.pose.position)
    if current_distance < 1.0:
        robot.stop()
        return False  # Stop the phase
    return True
```

### 動作確認済み機能（PyVista主体 - 2025-08-09更新）

```bash
# メインPyVistaデモ
python examples/pyvista_robot_demo.py 5          # ✅ インタラクティブ3Dウィンドウ
python examples/pyvista_robot_demo.py 10         # ✅ 長時間デモ
python examples/pyvista_simple_demo.py           # ✅ 画像生成・ヘッドレス対応
python examples/realtime_demo.py                 # ✅ マルチロボット・データ出力

# レガシーデモ（互換性維持）
python legacy/examples/visualization_demo.py     # ✅ matplotlib基本デモ
python legacy/examples/basic_demo.py             # ✅ 基本操作学習
python legacy/examples/realtime_demo_simple.py   # ✅ 軽量リアルタイム
```

### コード品質向上

#### 1. 英語統一完了
- **Comments**: 全て英語
- **Docstrings**: 英語で統一
- **Output messages**: 英語で統一
- **Documentation**: 英語で統一

#### 2. 構造化改善
- **関数の再利用**: 重複コード削除（約40行削減）
- **データ駆動設計**: フェーズ定義をデータ構造化
- **Error handling**: 堅牢なフォールバック機能

#### 3. 数値表示改善
- **Pose objects**: 人が読める数値表示
- **Debug output**: 詳細な状態情報
- **Performance metrics**: FPS、距離、時間の正確な測定

### 次回作業の推奨開始方法

#### 1. 状況確認
```bash
cd /home/rr/SimPyROS
python examples/basic_demo.py                    # 基本動作確認
python examples/visualization_demo.py 0.5        # 改良されたアニメーション確認
```

#### 2. 新機能テスト
```bash
# Real-time factor機能テスト
python examples/visualization_demo.py 0.3        # ゆっくり観察
python examples/visualization_demo.py 5.0        # 高速実行

# データ出力テスト
python examples/realtime_demo_simple.py          # JSON出力確認
ls output/                                        # 生成ファイル確認
```

#### 3. ドキュメント参照
- `examples/README.md`: 統合された使用方法（英語）
- `Appendix.md`: 技術的背景
- このファイル（CLAUDE.md）: 最新開発履歴

### 今後の開発候補

#### 高優先度
- **物理演算統合**: PyBulletとの連携
- **URDF/SDF読み込み強化**: 実ロボットモデル対応
- **追加センサーモデル**: LiDAR, カメラ等

#### 中優先度  
- **Multi-robot scenarios**: 複数ロボット協調
- **Path planning integration**: 経路計画アルゴリズム
- **Performance optimization**: 大規模シミュレーション対応

#### 低優先度
- **ROS2連携**: ROS 2ノードとしての実行
- **Web UI**: ブラウザベース制御インターフェース
- **機械学習連携**: 強化学習環境対応

### 開発環境

- **OS**: Linux (Ubuntu系)
- **Python**: 3.x
- **主要依存**: simpy, scipy, matplotlib, numpy, pyvista
- **推奨実行**: examples/フォルダから各デモを実行

---

## 作業再開時のチェックリスト

□ `cd /home/rr/SimPyROS`で作業ディレクトリに移動  
□ `python examples/basic_demo.py`で基本動作確認  
□ `python examples/visualization_demo.py 0.5`で改良アニメーション確認  
□ `ls output/`で生成ファイル確認  
□ 必要に応じて`pip install -r requirements.txt`  
□ 新機能開発前にCLAUDE.mdの内容を確認

**このファイル（CLAUDE.md）を参照すれば、前回までの作業内容と現在の状況が把握できます。**

## 最新更新（2025-08-09）

### プロジェクト構造の大幅整理完了

#### 1. PyVistaメイン化
- **PyVistaを主要バックエンド**に変更（リアルタイム3D描画対応）
- **インタラクティブ3Dウィンドウ**の実装完了
- **ヘッドレス環境対応**強化（X11エラー解決）

#### 2. ファイル構造整理
- **legacy/フォルダ**作成：matplotlib系コードを移動
- **examples/**を3ファイルに集約：PyVistaデモのみ
- **重複・非効率コード**の除去

#### 3. 主要デモの確立
- **pyvista_robot_demo.py**: リアルタイムインタラクティブ3D（メイン）
- **pyvista_simple_demo.py**: 画像生成・テスト用
- **realtime_demo.py**: マルチロボット・データ出力

#### 4. ドキュメント更新
- **examples/README.md**: PyVista主体の新説明書
- **legacy/README.md**: レガシーコード説明追加
- **CLAUDE.md**: 最新状況反映（このファイル）

### 動作確認済み機能
```bash
python examples/pyvista_robot_demo.py 5    # ✅ 3Dインタラクティブウィンドウ
python examples/pyvista_simple_demo.py     # ✅ ヘッドレス画像生成
python examples/realtime_demo.py           # ✅ マルチロボットデモ
```

## 追加更新（2025-08-09午後）

### examples/構造の再整理と最適化

#### 1. カテゴリ別フォルダ構成の導入
- **examples/basic/** - 基礎学習用デモ（basic_demo.py）
- **examples/pyvista/** - インタラクティブ3Dデモ（PyVistaメイン）
  - pyvista_robot_demo.py（インタラクティブリアルタイム）
  - pyvista_simple_demo.py（画像生成・テスト）

#### 2. 重複・複雑デモの整理
- **realtime_demo.py削除**: 複雑すぎるマルチロボットデモを legacy/ に移動
- **matplotlib系の整理**: visualization_demo.py を legacy/ に戻し依存関係を整理
- **alternative系の整理**: simpy_rt_demo.py も legacy/ に移動（visualizer依存のため）

#### 3. requirements.txt の最適化
**現在のexamples/実行に必要な最小依存関係:**
```
simpy>=4.0.0      # 基本シミュレーション
numpy>=1.20.0     # 数値計算
scipy>=1.7.0      # 空間変換
pyvista>=0.40.0   # 3D視覚化（メイン）
vtk>=9.0.0        # PyVistaバックエンド
```

#### 4. シンプルで明確な学習パス
1. `examples/basic/basic_demo.py` - 基礎概念理解
2. `examples/pyvista/pyvista_simple_demo.py` - 3D基本
3. `examples/pyvista/pyvista_robot_demo.py` - インタラクティブ体験

#### 5. 動作確認済み最終構成
```bash
# メインデモ（全て動作確認済み）
python examples/basic/basic_demo.py                  # ✅ 基礎学習
python examples/pyvista/pyvista_simple_demo.py       # ✅ 3D画像生成
python examples/pyvista/pyvista_robot_demo.py 5      # ✅ インタラクティブ3D

# レガシーデモ（互換性維持）
python legacy/examples/visualization_demo.py         # ✅ matplotlib基本
python legacy/examples/realtime_demo.py              # ✅ 複雑マルチロボット
```

### 完成した最終構造

```
SimPyROS/
├── simulation_object.py      # メインフレームワーク
├── requirements.txt         # 最適化済み依存関係
├── CLAUDE.md               # 開発履歴（このファイル）
├── examples/               # シンプル化された学習用デモ
│   ├── README.md           # 包括的な説明書
│   ├── basic/              # 基礎学習
│   │   └── basic_demo.py   
│   └── pyvista/            # インタラクティブ3D（メイン）
│       ├── pyvista_robot_demo.py    # インタラクティブリアルタイム
│       └── pyvista_simple_demo.py   # 画像生成・テスト
├── legacy/                 # 旧版・複雑版デモ保管
│   ├── README.md           # レガシー説明
│   ├── visualizer.py       # matplotlib視覚化
│   ├── examples/           # 旧デモ群
│   ├── backends/           # 旧バックエンド
│   └── tests/              # 旧テスト
├── tests/                  # 現行テスト
└── output/                 # 生成ファイル
```

**今日の最終成果**: PyVistaをメインとしつつ、学習しやすさを重視した構造に最適化。重複を除去し、依存関係を最小限にして、明確な学習パスを提供する実用的なロボットシミュレーション環境が完成しました。

---

## 最新セッション更新（2025-08-10）

### プロジェクト整理とURDFシステム最適化完了

#### 1. 完全なプロジェクトファイル整理
**実施した大規模整理:**
- **docs/フォルダ作成** - 全ドキュメントの体系化
  - `CLAUDE.md`, `Appendix.md`, `visualization_comparison.md`, `pyvista_performance_notes.md`
  - 各フォルダに包括的README.md作成
- **legacy/細分化** - 非推奨コードの適切な分類
  - `legacy/loaders/` - 旧URDFローダー（urdf_loader.py, simple_urdf_loader.py）
  - `legacy/debug/` - 開発デバッグスクリプト
- **unused/フォルダ** - 非使用ファイルの管理
- **メインREADME.md完全更新** - 最新機能とproject structure反映

#### 2. advanced_urdf_loader.pyの大幅簡素化
**問題**: 複数ライブラリ対応を謳いながら、実際はyourdfpyでのみテスト
**解決**: yourdfpy専用への簡素化実施

**削除した未実装コード（約200行）:**
- urchin, urdf_parser_py, urdfpy (original) の空実装関数
- 条件分岐が複雑な多重fallbackシステム
- 使用されていないメソッド（load_mesh_files等）

**yourdfpy API修正:**
```python
# 修正前（エラー）
for link in self.urdf_object.links:

# 修正後（正常動作）  
for link in self.urdf_object.robot.links:
```

**座標変換の手動実装:**
```python
# Pose.compose()が存在しないため手動実装
transformed_pos = parent_pose.position + parent_pose.rotation.apply(joint.origin_pos)
combined_rot = parent_pose.rotation * joint.origin_rot
child_pose = Pose.from_position_rotation(transformed_pos, combined_rot)
```

#### 3. ヘッドレスモードとスクリーンショット保存の分離
**ユーザー要求**: ヘッドレス実行時にスクリーンショット保存が不要な場合が多い
**実装した独立パラメータ:**

```bash
# 新しい実行オプション
python urdf_robot_demo.py 10 --headless              # ヘッドレス、screenshot無し
python urdf_robot_demo.py 10 --headless --screenshots # ヘッドレス、screenshot有り  
python urdf_robot_demo.py 10 my_robot.urdf --screenshots # インタラクティブ、screenshot有り
```

**内部実装:**
- `force_headless` と `save_screenshots` の独立制御
- 自動判定モード維持（DISPLAY環境変数ベース）
- VTKスクリーンショットエラーの適切なハンドリング

#### 4. 最適化されたプロジェクト構造

```
SimPyROS/
├── 📄 Core Files (4個のメインファイル)
│   ├── simulation_object.py       # メインフレームワーク
│   ├── pyvista_visualizer.py      # 3D視覚化システム  
│   ├── advanced_urdf_loader.py    # yourdfpy専用URDFローダー
│   └── requirements.txt           # 最小依存関係
├── 📚 docs/                       # 体系化されたドキュメント
│   ├── CLAUDE.md                  # 完全開発履歴
│   ├── Appendix.md                # 技術詳細
│   ├── visualization_comparison.md # バックエンド比較
│   └── pyvista_performance_notes.md # パフォーマンス分析
├── 🎮 examples/                   # 学習用デモ
│   ├── basic/basic_demo.py        # 基礎概念
│   ├── pyvista/                   # 3D可視化（メイン）
│   │   ├── pyvista_robot_demo.py  # インタラクティブ
│   │   ├── pyvista_simple_demo.py # 画像生成
│   │   └── urdf_robot_demo.py     # 高機能URDFデモ ⭐最新
│   └── robots/                    # URDF robotモデル
│       ├── simple_robot.urdf      # 3-link arm
│       ├── mobile_robot.urdf      # 車輪ロボット
│       └── rotation_test.urdf     # 4色テストロボット
├── 🗂️ legacy/                     # 非推奨・参考用（適切に分類）
│   ├── loaders/                   # 旧URDFローダー
│   ├── debug/                     # 開発デバッグスクリプト  
│   ├── examples/                  # matplotlib系デモ
│   └── backends/                  # 代替視覚化バックエンド
├── 🧪 tests/                      # テストスイート
├── 📁 output/                     # 生成ファイル
└── 🗃️ unused/                     # 非使用ファイル（明確な管理）
```

#### 5. 完全動作確認済みの機能
**URDF Robot Visualization System:**
- ✅ yourdfpy integration - 正確なAPI使用
- ✅ Material color extraction - 4色個別レンダリング確認
- ✅ Individual link positioning - 正確な座標変換  
- ✅ Real-time movement - 個別colored linksでの移動確認
- ✅ Headless & Interactive modes - 両モード完全対応
- ✅ Screenshot control - 独立したon/off制御

**動作確認済みコマンド例:**
```bash
# 基本動作確認
python examples/basic/basic_demo.py                  # ✅ 基礎概念
python examples/pyvista/urdf_robot_demo.py 10 examples/robots/simple_robot.urdf # ✅ 3色URDF robot

# ヘッドレスモード確認  
python examples/pyvista/urdf_robot_demo.py 5 --headless                    # ✅ screenshot無し
python examples/pyvista/urdf_robot_demo.py 5 --headless --screenshots      # ✅ screenshot有り
```

#### 6. 技術的改善点

**コードの簡素化:**
- advanced_urdf_loader.py: 約200行のデッドコード削除
- yourdfpy専用化による保守性向上  
- 明確なエラーメッセージとfallback処理

**プロジェクト構造の最適化:**
- 役割別フォルダ分類による可読性向上
- 各フォルダの目的を明確にするREADME.md
- legacy codeの適切な保管と参照性確保

**ユーザビリティ向上:**
- 独立したheadless/screenshot制御
- 包括的なusage examples in README.md
- 段階的学習パス（basic → pyvista → urdf_robot_demo）

### 開発完了状況

**完全実装・テスト済み機能:**
- 🤖 **URDF Robot Loading** - yourdfpyベース、material colors対応
- 🎮 **Interactive 3D Visualization** - PyVistaによる高品質レンダリング  
- ⚙️ **Individual Link Rendering** - 各パーツの独立color制御
- 🔄 **Real-time Movement** - 正確な座標変換による移動
- 📸 **Flexible Screenshot Control** - headless modeから独立
- 📚 **Comprehensive Documentation** - 体系化された文書構造

**プロジェクト成熟度**: Production-ready
- 明確な学習パス提供
- 保守しやすいコード構造  
- 包括的ドキュメント
- 適切なlegacy code管理

---

## 次回作業開始時の推奨手順

1. **基本動作確認**
   ```bash
   cd /home/rr/SimPyROS
   python examples/basic/basic_demo.py
   python examples/pyvista/urdf_robot_demo.py 10 examples/robots/rotation_test.urdf
   ```

2. **新機能開発時**
   - 最新のproject structure確認（README.md）
   - このCLAUDE.md履歴参照
   - examples/pyvista/urdf_robot_demo.py をベースに拡張

3. **ドキュメント更新**
   - 主要変更はCLAUDE.mdに記録
   - 技術詳細はAppendix.mdに追加
   - メインREADME.mdのroadmap更新

**現在の開発環境**: 高度なURDFロボット可視化システムを備えた、教育・研究用途に最適なロボットシミュレーション環境として完成。