#!/usr/bin/env python3
"""
Unified PyVista Visualizer - 統一バックエンド対応の拡張可視化
既存のPyVistaVisualizerを拡張してSimPyROS統一インターフェースに対応
"""

import time
import math
import threading
from typing import Dict, List, Optional, Callable, Any

from .pyvista_visualizer import PyVistaVisualizer, SceneBuilder
# Legacy classes replaced with modern architecture
class SimulationBackend:
    SIMPLE_WHILE_LOOP = "simple_while_loop"
    SIMPY_FREQUENCY_GROUP = "simpy_frequency_group"  
    SIMPY_PURE = "simpy_pure"

class BaseSimulationManager:
    pass
from .simulation_object import Velocity, Pose

try:
    import pyvista as pv
    PYVISTA_AVAILABLE = True
except ImportError:
    PYVISTA_AVAILABLE = False


class UnifiedPyVistaVisualizer(PyVistaVisualizer):
    """統一バックエンド対応PyVistaVisualizer"""
    
    def __init__(self, interactive: bool = True, window_size=(1400, 900)):
        super().__init__(interactive, window_size)
        
        # 統一インターフェース用の追加状態
        self.simulation_manager = None
        self.current_backend = None
        self.backend_switch_callbacks = []
        
        # UI要素
        self.backend_text_actor = None
        self.stats_text_actor = None
        self.control_text_actor = None
        
        # ロボット管理
        self.robot_visualization_data = {}
        self.update_thread = None
        self.running = False
        
        # 統計情報
        self.visualization_stats = {
            'start_time': 0.0,
            'frame_count': 0,
            'last_update_time': 0.0
        }
    
    def setup_unified_scene(self):
        """統一シーン設定"""
        
        if not self.available:
            return False
        
        plotter = self.plotter
        
        # 背景とライティング
        plotter.set_background('darkslateblue')
        self._setup_lighting(plotter)
        
        # 地面
        SceneBuilder.add_ground_plane(
            plotter, self.pv,
            center=(0, 0, -0.5),
            size=40.0,
            color='dimgray',
            opacity=0.4
        )
        
        # 座標軸
        SceneBuilder.add_coordinate_axes(
            plotter, self.pv,
            length=3.0,
            origin=(0, 0, 0)
        )
        
        # カメラ位置
        plotter.camera_position = [(0, -30, 20), (0, 0, 0), (0, 0, 1)]
        
        # UI要素追加
        self._setup_unified_ui()
        
        print("📺 Unified PyVista scene setup complete")
        return True
    
    def _setup_lighting(self, plotter):
        """ライティング設定"""
        try:
            # PyVistaのデフォルトライティングを使用
            plotter.enable_depth_peeling()
            plotter.add_light(self.pv.Light(
                position=(10, 10, 10),
                focal_point=(0, 0, 0),
                color='white'
            ))
        except Exception as e:
            print(f"⚠️ Lighting setup warning: {e}")
            # ライティングエラーは無視してデフォルトを使用
    
    def _setup_unified_ui(self):
        """統一UI要素設定"""
        plotter = self.plotter
        
        # メインタイトル
        plotter.add_text(
            "🎯 SimPyROS Unified Backend Visualization",
            position='upper_edge',
            font_size=16,
            color='white'
        )
        
        # バックエンド情報
        self.backend_text_actor = plotter.add_text(
            "Backend: None Selected",
            position='upper_left',
            font_size=12,
            color='yellow'
        )
        
        # 統計情報
        self.stats_text_actor = plotter.add_text(
            "Statistics:\nRTF: 0.000x\nCallbacks/sec: 0\nRobots: 0\nFrames: 0",
            position='lower_left',
            font_size=10,
            color='cyan'
        )
        
        # 制御ガイド
        self.control_text_actor = plotter.add_text(
            "🎮 Controls:\n"
            "1️⃣ Simple While Loop (Red)\n"
            "2️⃣ SimPy FrequencyGroup (Blue)\n"
            "3️⃣ Pure SimPy (Green)\n"
            "Space: Pause/Resume\n"
            "R: Reset Camera\n"
            "Q: Quit",
            position='lower_right',
            font_size=9,
            color='lightgreen'
        )
        
        # キーボードイベント設定
        self._setup_keyboard_controls()
    
    def _setup_keyboard_controls(self):
        """キーボード制御設定"""
        
        def key_press_handler(key):
            if key == '1':
                self._request_backend_switch(SimulationBackend.SIMPLE_WHILE_LOOP)
            elif key == '2':
                self._request_backend_switch(SimulationBackend.SIMPY_FREQUENCY_GROUP)
            elif key == '3':
                self._request_backend_switch(SimulationBackend.SIMPY_PURE)
            elif key == ' ':
                self._toggle_pause()
            elif key.lower() == 'r':
                self.plotter.reset_camera()
            elif key.lower() == 'q':
                self.plotter.close()
        
        # キーイベント登録
        try:
            self.plotter.iren.add_observer(
                'KeyPressEvent',
                lambda obj, event: key_press_handler(self.plotter.iren.GetKeySym())
            )
        except Exception as e:
            print(f"⚠️ Keyboard control setup warning: {e}")
    
    def attach_simulation_manager(self, sim_manager: BaseSimulationManager):
        """シミュレーション管理クラスをアタッチ"""
        
        self.simulation_manager = sim_manager
        
        # バックエンド情報取得
        backend_info = sim_manager.get_backend_info()
        self.current_backend = backend_info['backend']
        
        # UI更新
        self._update_backend_display()
        
        print(f"🔗 Attached simulation manager: {self.current_backend.value}")
    
    def start_visualization_loop(self):
        """可視化ループ開始"""
        
        if not self.available or not self.simulation_manager:
            return False
        
        self.running = True
        self.visualization_stats['start_time'] = time.time()
        
        # 更新スレッド開始
        self.update_thread = threading.Thread(
            target=self._visualization_update_loop,
            daemon=True
        )
        self.update_thread.start()
        
        print("🎬 Visualization update loop started")
        return True
    
    def stop_visualization_loop(self):
        """可視化ループ停止"""
        self.running = False
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        print("🎬 Visualization update loop stopped")
    
    def _visualization_update_loop(self):
        """可視化更新ループ"""
        
        last_robot_sync = 0.0
        
        while self.running:
            try:
                current_time = time.time()
                
                # ロボット可視化更新（30Hz）
                if current_time - last_robot_sync >= 0.033:
                    self._update_robot_visualization()
                    last_robot_sync = current_time
                
                # 統計更新（10Hz）
                if current_time - self.visualization_stats['last_update_time'] >= 0.1:
                    self._update_statistics_display()
                    self.visualization_stats['last_update_time'] = current_time
                
                self.visualization_stats['frame_count'] += 1
                
                # 更新レート制御
                time.sleep(0.01)  # 100Hz基本レート
                
            except Exception as e:
                print(f"⚠️ Visualization update error: {e}")
                break
    
    def _update_robot_visualization(self):
        """ロボット可視化更新"""
        
        if not self.simulation_manager or not hasattr(self.simulation_manager, 'robots'):
            return
        
        robots = self.simulation_manager.robots
        
        # 新しいロボットの追加
        for robot_name, robot in robots.items():
            if robot_name not in self.robot_visualization_data:
                self._add_robot_to_visualization(robot_name, robot)
        
        # 既存ロボットの位置更新
        for robot_name, viz_data in self.robot_visualization_data.items():
            if robot_name in robots:
                self._update_robot_position(robot_name, robots[robot_name], viz_data)
    
    def _add_robot_to_visualization(self, robot_name: str, robot: Any):
        """ロボットを可視化に追加"""
        
        try:
            # ロボットメッシュ作成
            robot_mesh = self._create_unified_robot_mesh()
            
            # バックエンドに応じた色
            color = self._get_backend_color()
            
            # 初期位置取得
            if hasattr(robot, 'position'):
                position = robot.position[:3] if len(robot.position) >= 3 else [0, 0, 0]
            elif hasattr(robot, 'pose') and hasattr(robot.pose, 'position'):
                position = robot.pose.position[:3]
            else:
                position = [0, 0, 0]
            
            # 位置適用
            positioned_mesh = robot_mesh.translate(position, inplace=False)
            
            # アクター追加
            actor = self.plotter.add_mesh(
                positioned_mesh,
                color=color,
                opacity=0.8,
                name=robot_name
            )
            
            # 可視化データ保存
            self.robot_visualization_data[robot_name] = {
                'actor': actor,
                'last_position': position.copy(),
                'mesh': robot_mesh,
                'update_count': 0
            }
            
        except Exception as e:
            print(f"⚠️ Failed to add robot {robot_name}: {e}")
    
    def _update_robot_position(self, robot_name: str, robot: Any, viz_data: Dict):
        """ロボット位置更新"""
        
        try:
            # 現在位置取得
            if hasattr(robot, 'position'):
                new_position = robot.position[:3] if len(robot.position) >= 3 else [0, 0, 0]
            elif hasattr(robot, 'pose') and hasattr(robot.pose, 'position'):
                new_position = robot.pose.position[:3]
            else:
                return
            
            # 位置変更チェック
            last_pos = viz_data['last_position']
            if self._position_changed(new_position, last_pos):
                
                # 新しいメッシュ作成
                new_mesh = viz_data['mesh'].copy()
                positioned_mesh = new_mesh.translate(new_position, inplace=False)
                
                # 向き適用（もしあれば）
                if hasattr(robot, 'orientation') and len(robot.orientation) >= 3:
                    yaw = robot.orientation[2]
                    positioned_mesh = positioned_mesh.rotate_z(math.degrees(yaw), inplace=False)
                
                # アクター更新
                self.plotter.remove_actor(robot_name)
                
                color = self._get_backend_color()
                actor = self.plotter.add_mesh(
                    positioned_mesh,
                    color=color,
                    opacity=0.8,
                    name=robot_name
                )
                
                # データ更新
                viz_data['actor'] = actor
                viz_data['last_position'] = new_position.copy()
                viz_data['update_count'] += 1
        
        except Exception as e:
            # 可視化エラーは無視（シミュレーションを止めない）
            pass
    
    def _create_unified_robot_mesh(self):
        """統一ロボットメッシュ作成"""
        
        # ベース（円柱）
        base = self.pv.Cylinder(
            center=[0, 0, 0.1],
            direction=[0, 0, 1],
            radius=0.4,
            height=0.2
        )
        
        # 上部センサー（球）
        sensor = self.pv.Sphere(
            center=[0, 0, 0.3],
            radius=0.15
        )
        
        # 方向指示器（矢印）
        direction_arrow = self.pv.Arrow(
            start=[0, 0, 0.2],
            direction=[1, 0, 0],
            scale=0.5
        )
        
        # ホイール（左右）
        wheel_left = self.pv.Cylinder(
            center=[-0.3, 0.4, 0],
            direction=[0, 1, 0],
            radius=0.1,
            height=0.05
        )
        
        wheel_right = self.pv.Cylinder(
            center=[-0.3, -0.4, 0],
            direction=[0, 1, 0],
            radius=0.1,
            height=0.05
        )
        
        # 全て合成
        robot_mesh = base + sensor + direction_arrow + wheel_left + wheel_right
        
        return robot_mesh
    
    def _get_backend_color(self) -> str:
        """バックエンドに応じた色取得"""
        if not self.current_backend:
            return 'gray'
        
        color_map = {
            SimulationBackend.SIMPLE_WHILE_LOOP: 'red',
            SimulationBackend.SIMPY_FREQUENCY_GROUP: 'blue',
            SimulationBackend.SIMPY_PURE: 'green'
        }
        
        return color_map.get(self.current_backend, 'gray')
    
    def _position_changed(self, pos1: List[float], pos2: List[float], threshold: float = 0.01) -> bool:
        """位置変更判定"""
        return any(abs(a - b) > threshold for a, b in zip(pos1, pos2))
    
    def _update_statistics_display(self):
        """統計表示更新"""
        
        if not self.simulation_manager:
            return
        
        try:
            # シミュレーション統計取得
            sim_stats = self.simulation_manager.get_performance_stats()
            
            # 可視化統計計算
            viz_elapsed = time.time() - self.visualization_stats['start_time']
            viz_fps = self.visualization_stats['frame_count'] / viz_elapsed if viz_elapsed > 0 else 0
            
            # 統計テキスト作成
            stats_text = (
                f"📊 Statistics:\n"
                f"RTF: {sim_stats.get('rtf', 0):.3f}x\n"
                f"CB/sec: {sim_stats.get('callbacks_per_sec', 0):.0f}\n"
                f"Robots: {len(self.robot_visualization_data)}\n"
                f"Viz FPS: {viz_fps:.1f}\n"
                f"Sim Frames: {sim_stats.get('frame_count', 0)}\n"
                f"Runtime: {viz_elapsed:.1f}s"
            )
            
            self.stats_text_actor.SetText(3, stats_text)
            
        except Exception as e:
            print(f"⚠️ Statistics update error: {e}")
    
    def _update_backend_display(self):
        """バックエンド表示更新"""
        
        if not self.current_backend:
            return
        
        backend_text = f"🔧 Backend: {self.current_backend.value}"
        
        # 性能ティア表示
        if self.simulation_manager:
            tier = self.simulation_manager._get_performance_tier()
            backend_text += f" ({tier})"
        
        self.backend_text_actor.SetText(3, backend_text)
        
        # 既存ロボットの色更新
        self._update_robot_colors()
    
    def _update_robot_colors(self):
        """ロボット色更新"""
        
        new_color = self._get_backend_color()
        
        for robot_name, viz_data in self.robot_visualization_data.items():
            try:
                # アクターの色を更新
                viz_data['actor'].GetProperty().SetColor(
                    *self.pv.colors.get_color(new_color)
                )
            except Exception as e:
                print(f"⚠️ Color update error for {robot_name}: {e}")
    
    def _request_backend_switch(self, new_backend: SimulationBackend):
        """バックエンド切り替え要求"""
        
        if self.current_backend == new_backend:
            print(f"⚠️ Already using {new_backend.value}")
            return
        
        print(f"🔄 Requesting switch to {new_backend.value}...")
        
        # コールバック実行
        for callback in self.backend_switch_callbacks:
            try:
                callback(new_backend)
            except Exception as e:
                print(f"⚠️ Backend switch callback error: {e}")
    
    def _toggle_pause(self):
        """一時停止切り替え"""
        
        if hasattr(self.simulation_manager, 'paused'):
            self.simulation_manager.paused = not getattr(self.simulation_manager, 'paused', False)
            status = "Paused" if self.simulation_manager.paused else "Running"
            print(f"🎮 Simulation {status}")
        else:
            print("⚠️ Pause not supported by current backend")
    
    def add_backend_switch_callback(self, callback: Callable[[SimulationBackend], None]):
        """バックエンド切り替えコールバック追加"""
        self.backend_switch_callbacks.append(callback)
    
    def clear_robots(self):
        """全ロボットクリア"""
        
        for robot_name in list(self.robot_visualization_data.keys()):
            try:
                self.plotter.remove_actor(robot_name)
            except:
                pass
        
        self.robot_visualization_data.clear()
        print("📺 Cleared all robot visualizations")
    
    def show_unified(self):
        """統一可視化表示"""
        
        if not self.available:
            print("❌ PyVista not available")
            return
        
        try:
            print("🖥️ Opening unified PyVista visualization...")
            print("   Use keyboard controls to switch backends")
            
            # ウィンドウ表示（ブロッキング）
            self.plotter.show()
            
        except Exception as e:
            print(f"❌ Visualization error: {e}")
        
        finally:
            self.stop_visualization_loop()


def create_unified_visualizer(interactive: bool = True) -> Optional[UnifiedPyVistaVisualizer]:
    """統一ビジュアライザー作成"""
    
    if not PYVISTA_AVAILABLE:
        print("⚠️ PyVista not available for unified visualization")
        return None
    
    visualizer = UnifiedPyVistaVisualizer(interactive=interactive)
    
    if not visualizer.available:
        print("❌ Failed to create unified visualizer")
        return None
    
    return visualizer