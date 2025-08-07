# 3D Robotics Visualization ライブラリ比較

## 現状分析

現在のmatplotlibベース実装では、ロボットのCGモデル（3Dメッシュ）表示に制限があります。

### matplotlib実装の制限
```python
# 現在できること
self.ax.scatter()      # 点・マーカー表示のみ
self.ax.plot()         # 線・軌跡表示
coordinate_frames()    # 座標軸表示

# できないこと  
❌ STL/OBJ/DAE メッシュ読み込み
❌ URDFメッシュリンク表示
❌ テクスチャマッピング
❌ リアルな3Dロボット表示
```

## 推奨代替案

### 1. PyVista（最推奨）

**特徴:**
- VTKベースで高性能
- matplotlib類似の直感的API
- 豊富なメッシュフォーマット対応
- インタラクティブ表示

**実装例:**
```python
import pyvista as pv

class PyVistaRobotVisualizer:
    def __init__(self):
        self.plotter = pv.Plotter()
        self.meshes = {}
    
    def load_robot_mesh(self, name, mesh_path):
        """STL/OBJ/PLYメッシュを読み込み"""
        mesh = pv.read(mesh_path)
        self.meshes[name] = mesh
        self.plotter.add_mesh(mesh, name=name)
    
    def update_robot_pose(self, name, pose):
        """ロボットの姿勢を更新"""
        if name in self.meshes:
            # 変換行列適用
            transform = pose.to_transformation_matrix()
            transformed = self.meshes[name].transform(transform)
            self.plotter.update_mesh(transformed, name=name)
```

**メリット:**
- ✅ URDF meshes 直接対応
- ✅ リアルタイムアニメーション
- ✅ 高品質レンダリング
- ✅ Jupyter対応

### 2. Open3D

**特徴:**
- C++ベース高速処理
- 点群処理に特化
- 機械学習統合(Open3D-ML)

**実装例:**
```python
import open3d as o3d

class Open3DRobotVisualizer:
    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        
    def add_robot_mesh(self, mesh_path, pose):
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        mesh.transform(pose.to_transformation_matrix())
        self.vis.add_geometry(mesh)
```

**メリット:**
- ✅ 高速パフォーマンス
- ✅ 大規模点群対応
- ✅ ML統合
- ❌ API が複雑

### 3. RViz2/RViz (ROS統合)

**特徴:**
- ROSネイティブ対応
- URDF完全サポート
- ロボティクス特化

**実装例:**
```python
# ROS2 + rclpy
import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

class ROS2Visualizer:
    def publish_robot_marker(self, pose, mesh_path):
        marker = Marker()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = f"file://{mesh_path}"
        marker.pose = pose
        self.marker_pub.publish(marker)
```

### 4. ハイブリッド実装（推奨）

**戦略:** 用途に応じて選択可能

```python
class FlexibleRobotVisualizer:
    def __init__(self, backend='matplotlib'):
        if backend == 'pyvista':
            self.viz = PyVistaVisualizer()
        elif backend == 'open3d':
            self.viz = Open3DVisualizer()
        elif backend == 'matplotlib':
            self.viz = MatplotlibVisualizer()  # 現在の実装
        else:
            raise ValueError("Unsupported backend")
    
    def add_robot(self, name, mesh_path=None, **kwargs):
        if mesh_path and hasattr(self.viz, 'load_mesh'):
            self.viz.load_robot_mesh(name, mesh_path)
        else:
            self.viz.add_object(name, **kwargs)  # フォールバック
```

## 実装優先順位

### Phase 1: PyVista拡張（推奨）
```python
# examples/pyvista_robot_demo.py 作成
# - STL/OBJメッシュ読み込み
# - リアルタイムアニメーション  
# - 現在のSimPyシミュレーションと統合
```

### Phase 2: Open3D統合
```python
# 高速処理が必要な場合
# 大規模点群データ対応
```

### Phase 3: ROS2連携
```python
# 将来的なROSエコシステム統合
```

## 互換性維持

現在のmatplotlib実装は残し、選択可能に：

```python
# 後方互換性
python examples/example_visualization.py              # matplotlib
python examples/pyvista_robot_visualization.py       # PyVista
python examples/open3d_robot_visualization.py        # Open3D
```

## 必要な依存関係

```bash
# PyVista
pip install pyvista[all]

# Open3D  
pip install open3d

# ROS2 (別途インストール)
# Ubuntu: apt install ros-humble-desktop
```

## 結論

**即座に改善するなら:** PyVista導入
**長期的な発展:** ハイブリッド実装で選択可能
**現在の実装:** 軽量用途として維持