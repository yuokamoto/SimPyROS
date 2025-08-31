# Process-Separated PyVista 可視化 高速化・スケーリング計画

最終更新: 2025-08-31
対象ブランチ: feature/basic_sim_quick_wins

## 目的
100 台規模ロボット (各 10〜30 リンク想定) を Real-Time Factor (RTF) >= 10（もしくは rtf=0 最大速度）で安定表示し、制御ループの遅延を 50ms 未満 (理想 20ms) に抑える。

## 背景 / 現状
現行 `process_separated_pyvista` バックエンドは以下を実装済:
- 幾何 (Mesh) 一括転送: `multiprocessing.Queue` による一度きりロード
- 姿勢更新: 共有メモリ (SharedMemory) でゼロコピー配列をプロセス間伝達
- 差分更新: per-link diff + dirty flag で無駄描画削減
- バックグラウンド pose thread + Event による低 CPU レンダーループ
- ロボット数上限 (`max_robots`) 拡張 / 警告抑制

課題:
- 大量ロボット時 shutdown 安定性 (core dump 134) 未対応
- 共有メモリ書き込みは Python ループでシリアライズ (NumPy ベクトル化余地)
- レンダー頻度制御は単純 (RTF/負荷適応なし)
- リンク数別の可視化負荷計測が未整備
- IPC (Queue) の初期幾何転送が順次処理で並列化していない

## KPI 指標
| KPI | 目標 | 備考 |
|-----|------|------|
| 100 台初期スポーン時間 | <= 5 s | “ready_event” 発火まで |
| 共有メモリ pose 書込み時間/フレーム | <= 3 ms | 100 台時合計 |
| 可視化プロセス CPU 使用率 | <= 120% (2 コア相当) | 100 台継続時 |
| 1 秒あたりレンダリング回数 | 動的適応 10〜30 fps | 無変化時は <5 fps |
| シャットダウン待機 | <= 2 s | リソースリークなし |

## フェーズ別計画

### Phase 0 (Quick Wins / 低リスク)
1. Pose 書込みパス NumPy 化: `pose_data` リスト構築 → 事前確保 ndarray へ直接代入
2. `_pose_epsilon` 自動調整: ロボット静止時間が長いほど閾値緩和 (微小ノイズ抑制)
3. レンダー Event coalescing: 連続 pose 更新 (<=5ms 間隔) は 1 回の render にまとめる
4. 計測フック: 共有メモリ書込み時間, diff でスキップしたリンク数, 1 フレームあたり更新リンク数
5. 安全 shutdown: 子プロセス内で `atexit` + SIGTERM handler 追加し core dump 134 防止

### Phase 1 (データパス & CPU 最適化)
1. 共有メモリレイアウト SoA 検討: (x[], y[], z[], qw[], qx[], qy[], qz[]) に分離しキャッシュ効率向上
2. 書込み側: 連続 doubles pack (`struct.pack`) から `np.ndarray` の `memoryview` 直接代入に変更
3. Dirty flag 粒度拡張: ロボット全体 → 32/64 ビットビットマスク (最大 64 リンク) にして部分的チェック
4. Link order ハッシュ監視: 変更ない限り再構築禁止
5. 更新スレッド優先度調整 (Linux: `os.nice(+5)`) で過度な CPU 奪取抑制

### Phase 2 (レンダリング制御)
1. Adaptive FPS: 更新リンク数 / CPU 使用率に応じて目標レンダリング周期変更
2. フレームスキップ: 物理/制御ステップ N>1 回進んでもリンク差分小なら描画間引き
3. バッチ orientation 適用: Euler 変換 (`Rotation.from_quat`) を配列一括 → 角度配列生成
4. LOD (Level of Detail): 距離閾値でメッシュ簡易化・ワイヤフレーム化
5. 初期ロード非同期化: 幾何ロードを少量タスクに分割し表示遅延短縮

### Phase 3 (I/O & 並列化)
1. 幾何転送圧縮オプション (pickle→lz4 / zstd) + CRC チェック (軽量化 vs CPU トレードオフ評価)
2. 複数 shared memory セグメント分割: 1000 台以上視野で可用性確保 (リングバッファ構造)
3. 複数 Pose Writer スレッド (ロボットシャーディング) + lock-free index (atomic counter)
4. 視覚プロセスでの GPU メモリプール化 (PyVista / VTK Reuse)
5. 遠距離ロボット更新サンプリング (1/2, 1/4 レート)

### Phase 4 (高度最適化 / 拡張)
1. C 拡張 / Cython / Numba JIT: pose 書込み & diff 検出ループネイティブ化
2. Vulkan / EGL headless backend (VTK Offscreen 最適化) 評価
3. Mesh Instancing: 同一形状の大量複製ロボットで GPU インスタンシング (VTK 拡張検討)
4. GPU サイド姿勢更新: UBO/SSBO へ quaternion + translation 転送 (要 VTK カスタム)
5. 未来 timestamp 補間 (レンダー遅延削減 / ネットワーク同期準備)

## PyBullet ハイブリッド統合方針 (短期試験)
| ステップ | 手順 | 備考 |
|----------|------|------|
| 1 | PyBullet DIRECT 初期化 | GUI なし, 最低限重力設定 |
| 2 | URDF 一括ロード → bodyUniqueId 配列保持 | ロボット名 → id マップ |
| 3 | SimPy イベント内で関節コマンド蓄積 | setJointMotorControlArray まとめ発行 |
| 4 | N 内部ステップ (batch) 実行 | stepSimulation を連続呼びで Python 往復減 |
| 5 | getLinkStates (batched) | 共有メモリ layout に直接書込み |
| 6 | 計測 & 比較 (KPI テーブル) | step 時間, RTF, CPU 使用率 |

## メトリクス取得手順 (テンプレート)
1. `export SIMPYROS_VIS_VERBOSE=0`
2. 例: `python examples/beginner/basic_simulation.py --example performance --num-robots 100 --duration 30 --rtf 0 --vis --vb process_separated_pyvista`
3. ログ/計測コードが出力する JSON (未実装なら Phase0 で追加) を `logs/perf_<timestamp>.json` に保存
4. 指標: 初期ロード秒数, 平均 pose 書込み ms, 平均/最大 render 間隔, skipped_frames, diff更新比
5. スプレッドシート or pandas 集計で推移グラフ化

## リスク & 対策
| リスク | 影響 | 緩和策 |
|--------|------|--------|
| 共有メモリ破損 | 全ロボット姿勢無効 | バージョン/マジック番号 + 整合性チェック追加 |
| Shutdown ハング | プロセス残留 | タイムアウト + SIGKILL フォールバック (実装済) + atexit |
| 過度な最適化で複雑化 | 保守性低下 | フェーズごと PR 分割 / KPI 達成で打ち切り線引き |
| PyBullet 併用で API 分岐膨張 | コード肥大 | Adapter 層 (PhysicsProvider Interface) 追加 |

## 次アクション (優先順)
1. Phase 0-1 の Quick Wins 実装 (pose 書込み NumPy 化 + Event coalescing + 計測フック)
2. Shutdown 安定化 (core dump 解析 / atexit 追加)
3. KPI ログ出力 (JSON) インフラ整備
4. PyBullet ハイブリッド最小 PoC (10 台) & RTF ベンチ比較
5. Adaptive FPS (更新リンク閾値ベース) 実装

---
このファイルをベースに進捗ごとにセクションへ実測値を追記し、不要になった仮説は strike-through 化して履歴を残す。
