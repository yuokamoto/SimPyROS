# SimPyROS 改善設計書

## 概要
memo.txtの改善項目6,7,8に基づくSimPyROSの設計改善仕様書です。各項目の課題分析と実装方針を明確化します。

---

## 6. Visualizer改善

### 6.1 Visualization対象の適正化
**課題**: 現状、visualizerがrobotのみupdateしてsimulation_objectをupdateしていない可能性

**設計方針**:
- `PyVistaVisualizer`と`URDFRobotVisualizer`で全ての`SimulationObject`をupdateするように統一
- `AnimationController`に汎用オブジェクト更新機能を追加
- robot以外のsimulation_object（box, sphere等）も可視化対象に含める

### 6.2 Material透明度の修正
**課題**: URDF loaderで読み込んだmaterialが半透明になっている

**設計方針**:
- `urdf_loader.py`の`_extract_visual_origin()`でmaterial opacity処理を修正
- デフォルトopacity=1.0（不透明）に変更
- URDF内でexplicitに透明度が指定されている場合のみ透明化

### 6.3 Interactive UI Controls
**課題**: ウィンドウ内にコントロールボタンが必要

**設計方針**:
```python
class VisualizerControls:
    - axis_display_toggle: bool
    - realtime_factor: float (0.1 - 5.0)
    - collision_display_toggle: bool
    - robot_wireframe_toggle: bool
```
- PyVistaのwidget機能を使用してUI実装
- リアルタイム更新対応

### 6.4 RobotMeshFactory統合
**課題**: `RobotMeshFactory`と`URDFRobotVisualizer`の重複

**設計方針**:
- `RobotMeshFactory`の機能を`URDFRobotVisualizer`に統合
- URDF mesh作成ロジックを`URDFRobotVisualizer._create_mesh_from_urdf()`に移動
- factory pattern維持しつつcode duplication解消

### 6.5 Robot情報活用による描画最適化
**課題**: `URDFRobotVisualizer`と`Robot`が両方でload_urdfを実行している冗長性

**設計方針**:
```python
# 改善後の描画フロー
URDFRobotVisualizer.load_robot(robot_instance):
    # robot.urdf_loaderから情報を取得
    urdf_info = robot_instance.urdf_loader
    links = robot_instance.links
    joints = robot_instance.joints
    # 描画処理のみ実行（URDF再読み込み不要）
```

---

## 7. Robot Class改善

### 7.1 プログラマティックロボット作成
**課題**: 現状はURDFファイルからのみロボット作成可能

**設計方針**:
```python
class Robot:
    def add_link(self, name: str, geometry_type: str, 
                geometry_params: Dict, color: Tuple[float, float, float],
                pose: Pose = None) -> bool
    
    def add_joint(self, name: str, joint_type: JointType,
                 parent_link: str, child_link: str,
                 origin_pos: np.ndarray, origin_rot: Rotation,
                 axis: np.ndarray = None, limits: Dict = None) -> bool
    
    def finalize_robot(self) -> bool  # 構築完了後の整合性チェック
```

### 7.2 Dynamic Robot構築ワークフロー
```python
# 使用例
robot = Robot(env, base_parameters)
robot.add_link("base", "box", {"size": [0.3, 0.3, 0.1]}, (0.8, 0.8, 0.8))
robot.add_link("arm1", "cylinder", {"radius": 0.05, "length": 0.35}, (1.0, 0.5, 0.0))
robot.add_joint("joint1", JointType.REVOLUTE, "base", "arm1", [0,0,0.1], Rotation.identity())
robot.finalize_robot()
```

---

## 8. SimulationManager改善

### 8.1 Thread使用の必要性分析
**課題**: SimPy環境でthread使用の適切性

**現状分析**:
- **simulation thread**: SimPyの環境でprocess実行
- **visualization thread**: PyVistaの描画更新

**設計判断**:
```python
# Option A: SimPy Pure (推奨)
class SimulationManager:
    def __init__(self):
        self.env = simpy.Environment()
        # visualization processをSimPy内で実行
        self.viz_process = self.env.process(self._visualization_loop())
    
    def _visualization_loop(self):
        while True:
            self.update_visualization()  # Non-blocking update
            yield self.env.timeout(1/60)  # 60 FPS

# Option B: Hybrid (現状維持が必要な場合)
# PyVista interactiveモードでblocking callが必要な場合のみthread使用
```

**結論**: SimPy純粋環境を優先。PyVista interactive必須の場合のみhybrid approach

### 8.2 Real-time Factor制御
```python
class SimulationConfig:
    realtime_factor: float = 1.0  # リアルタイム係数
    max_step_size: float = 0.01   # 最大ステップサイズ
    visualization_fps: float = 60.0  # 描画フレームレート
```

---

## Implementation Priority

### Phase 1: Critical fixes
1. Material透明度修正 (6.2)
2. Thread architecture review (8.1)
3. Visualization対象統一 (6.1)

### Phase 2: Architecture improvements  
1. RobotMeshFactory統合 (6.4)
2. Robot描画最適化 (6.5)
3. Programmatic robot creation (7.1)

### Phase 3: UI enhancements
1. Interactive controls (6.3)
2. Real-time factor control (8.2)

---

## Testing Strategy
- 各改善後にbasic_simulation.pyで動作確認
- Performance regression test (89+ Hz維持)
- Multi-robot scenario test
- Headless mode compatibility test