# SimPyROS Examples

このフォルダには、SimPyROSシミュレーションフレームワークの使用例が含まれています。

## ファイル一覧

### 基本例

#### `example.py`
SimPyROSの基本機能を学習するためのシンプルな例
- **目的**: 基本的なオブジェクト作成、接続、移動の仕組みを理解する
- **内容**: 
  - 親オブジェクト（ロボット）と子オブジェクト（センサー）の作成
  - オブジェクト間の接続（attach/detach機能）
  - 回転、移動、テレポート操作のテスト
- **実行方法**: `python examples/example.py`
- **学習ポイント**: オブジェクトの基本操作と座標変換の理解

#### `example_connected.py` 
複数オブジェクト間の双方向接続システムのテスト例
- **目的**: 複雑な接続関係での動作確認
- **内容**:
  - 3つのオブジェクト（A、B、C）のチェーン接続
  - 双方向接続による相互影響の確認
  - 接続/切断操作のテスト
- **実行方法**: `python examples/example_connected.py`
- **学習ポイント**: 双方向接続システムの仕組みと効果

### 視覚化例

#### `example_visualization.py`
3D視覚化機能を使用したインタラクティブなシミュレーション
- **目的**: リアルタイム3D視覚化の体験
- **内容**:
  - 複数のオブジェクト（ロボット、センサー、ターゲット、障害物）
  - リアルタイムアニメーション
  - 軌跡表示と座標フレーム表示
  - 複数のモーションパターン（円運動、8の字、螺旋など）
- **実行方法**: 
  - アニメーション版: `python examples/example_visualization.py`
  - 静的版: `python examples/example_visualization.py static`
- **学習ポイント**: 3D視覚化システムの活用方法
- **注意**: matplotlib が必要（`pip install matplotlib`）

### リアルタイムデモ

SimPyROSでは、リアルタイム実行のために2つの異なるアプローチを提供しています：

#### 手動制御方式（推奨）

##### `headless_realtime_demo.py`
GUI環境がない場合でも動作するリアルタイムデモ（PNG出力対応）
- **方式**: 手動制御（`time.sleep()`使用）
- **目的**: ヘッドレス環境での視覚化とフレーム保存
- **内容**:
  - リアルタイム物理シミュレーション
  - 定期的なPNGフレーム保存（output/フォルダ）
  - 複数のモーションパターン（円運動→8の字→螺旋）
  - 複数実行モード（quick/detailed/analysis）
- **実行方法**:
  - 標準: `python examples/headless_realtime_demo.py`
  - 高速: `python examples/headless_realtime_demo.py quick`
  - 詳細: `python examples/headless_realtime_demo.py detailed`
  - 解析用: `python examples/headless_realtime_demo.py analysis`
- **出力**: output/frame_XXX.png ファイル
- **学習ポイント**: ヘッドレス環境でのシミュレーション実行

##### `simple_realtime_demo.py`
シンプルなリアルタイムシミュレーション例
- **方式**: 手動制御（`time.sleep()`使用）
- **目的**: リアルタイム実行の基本パターンの理解
- **内容**: 基本的なリアルタイム動作とタイミング制御
- **実行方法**: `python examples/simple_realtime_demo.py`
- **学習ポイント**: リアルタイム制御の基礎

#### SimPy.rt方式（代替実装）

##### `simpy_rt_demo.py`
SimPy.rtを使用したリアルタイムシミュレーション例
- **方式**: SimPy.rt（`simpy.rt.RealtimeEnvironment`使用）
- **目的**: SimPy組み込みリアルタイム機能の体験
- **内容**:
  - 円運動デモ、複数ロボット相互作用、インタラクティブパターン
  - 自動時間スケール調整（高速起動→リアルタイム移行）
  - 複数実行モード（3つのデモから選択）
- **実行方法**:
  - 円運動: `python examples/simpy_rt_demo.py 1`
  - 複数ロボット: `python examples/simpy_rt_demo.py 2`
  - インタラクティブ: `python examples/simpy_rt_demo.py 3`
- **学習ポイント**: SimPy.rtの機能と制限の理解
- **注意**: Appendix.mdを参照してください（プラットフォーム依存性あり）

### 2つのリアルタイム方式の比較

| 項目 | 手動制御方式 | SimPy.rt方式 |
|------|-------------|-------------|
| **パフォーマンス** | 高速 | 同期オーバーヘッドあり |
| **プラットフォーム互換性** | 良好 | Windows > Linux |
| **制御の柔軟性** | 高い | 制限あり |
| **実装の複雑さ** | 中程度 | シンプル |
| **用途** | 研究・分析・本格運用 | 教育・デモ・プロトタイプ |

**推奨**: 一般的な用途では手動制御方式をお勧めします。詳細はAppendix.mdを参照してください。

## 実行順序の推奨

### 基本学習コース
1. **`example.py`** - 基本操作の理解
2. **`example_connected.py`** - 接続システムの理解  
3. **`example_visualization.py static`** - 静的視覚化の体験
4. **`example_visualization.py`** - リアルタイム視覚化の体験
5. **`headless_realtime_demo.py quick`** - ヘッドレス環境での実行体験（推奨リアルタイム方式）

### リアルタイム実装比較コース
6. **`simple_realtime_demo.py`** - 手動制御方式の基礎
7. **`simpy_rt_demo.py 1`** - SimPy.rt方式の体験
8. 両方式の特徴比較（Appendix.md参照）

## 必要な依存関係

```bash
pip install simpy scipy matplotlib numpy
```

## トラブルシューティング

- **ImportError**: 親ディレクトリの`simulation_object.py`と`visualizer.py`が必要です
- **Display関連エラー**: ヘッドレス環境では自動的にPNG出力モードに切り替わります
- **matplotlib関連エラー**: `pip install matplotlib`で解決できます

## 出力ファイル

視覚化例やデモの実行により、以下のファイルが`output/`フォルダに保存されます：
- `frame_XXX.png`: アニメーションフレーム
- `visualization_output.png`: 視覚化結果
- その他の分析用画像ファイル