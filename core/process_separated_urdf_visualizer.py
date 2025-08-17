#!/usr/bin/env python3
"""
Process-Separated URDF Robot Visualizer

URDFRobotVisualizerインターフェースに準拠したプロセス分離PyVistaビジュアライザー
"""

import numpy as np
from typing import Dict, List, Optional, Any
import warnings

# Add parent directory to path
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from core.process_separated_pyvista import ProcessSeparatedPyVistaVisualizer, SharedMemoryConfig
from core.simulation_object import Pose


class ProcessSeparatedURDFRobotVisualizer(ProcessSeparatedPyVistaVisualizer):
    """
    URDFRobotVisualizerインターフェース準拠のプロセス分離ビジュアライザー
    
    SimulationManagerとの互換性を提供
    """
    
    def __init__(self, config: Optional[SharedMemoryConfig] = None):
        super().__init__(config)
        
        # URDF robot tracking
        self.urdf_robots = {}  # robot_name -> urdf_data
        self.robot_link_poses = {}  # robot_name -> {link_name -> pose}
        
        # Time management
        self.time_manager = None
        self.simulation_manager = None
        
    def connect_time_manager(self, time_manager):
        """TimeManagerに接続"""
        self.time_manager = time_manager
        print("🔗 Connected to TimeManager for centralized time access")
    
    def connect_simulation_manager(self, simulation_manager):
        """SimulationManagerに接続"""
        self.simulation_manager = simulation_manager
        print("🔗 Connected to SimulationManager for real-time factor control")
    
    def load_robot(self, robot_name: str, robot_data: Any) -> bool:
        """
        ロボットをロード（SimulationManager互換インターフェース）
        
        Args:
            robot_name: ロボット名
            robot_data: Robot instance with URDF data
            
        Returns:
            True if successful
        """
        try:
            if not hasattr(robot_data, 'urdf_loader') or not robot_data.urdf_loader:
                print(f"⚠️ Robot {robot_name} has no URDF data")
                return False
            
            # URDF データを保存
            self.urdf_robots[robot_name] = robot_data
            
            # リンク数を計算
            urdf_loader = robot_data.urdf_loader
            num_links = len(urdf_loader.links)
            
            # URDFデータを抽出
            urdf_data = self._extract_urdf_data(robot_data)
            
            # プロセス分離ビジュアライザーにロボット追加（URDFデータ付き）
            success = self.add_robot_with_urdf(robot_name, urdf_data)
            
            if success:
                # リンクポーズ管理初期化
                self.robot_link_poses[robot_name] = {}
                
                print(f"🤖 Robot '{robot_name}' loaded successfully")
                print(f"   Links: {num_links}")
                print(f"   Joints: {len(urdf_loader.joints)}")
                
                # リンク情報表示
                print(f"   📦 Links:")
                for link_name in urdf_loader.links.keys():
                    print(f"     - {link_name}")
                
                return True
            else:
                print(f"❌ Failed to add robot to process-separated visualizer")
                return False
                
        except Exception as e:
            print(f"❌ Failed to load robot '{robot_name}': {e}")
            return False
    
    def update_robot_visualization(self, robot_name: str) -> bool:
        """
        ロボット可視化を更新（SimulationManager互換インターフェース）
        
        Args:
            robot_name: 更新するロボット名
            
        Returns:
            True if successful
        """
        if robot_name not in self.urdf_robots:
            return False
        
        try:
            robot = self.urdf_robots[robot_name]
            
            # 現在のリンクポーズを取得
            if hasattr(robot, 'get_link_poses'):
                link_poses = robot.get_link_poses()
                
                # リンクポーズを変換行列に変換
                transforms = []
                link_names = []
                
                urdf_loader = robot.urdf_loader
                for link_name in urdf_loader.links.keys():
                    if link_name in link_poses and link_poses[link_name] is not None:
                        pose = link_poses[link_name]
                        transform = pose.to_transformation_matrix()
                        transforms.append(transform)
                        link_names.append(link_name)
                    else:
                        # デフォルト変換行列（単位行列）
                        transforms.append(np.eye(4))
                        link_names.append(link_name)
                
                # プロセス分離ビジュアライザーに送信
                if transforms:
                    success = self.update_robot_transforms(robot_name, transforms)
                    
                    if success:
                        # ローカルキャッシュ更新
                        self.robot_link_poses[robot_name] = link_poses
                        
                    # 時間情報も更新（標準PyVistaと同等）
                    self._update_timing_from_connected_managers()
                        
                    return success
            
            # 時間情報更新（標準PyVistaと同等）
            self._update_timing_from_connected_managers()
            return True
            
        except Exception as e:
            print(f"⚠️ Error updating robot visualization '{robot_name}': {e}")
            return False
    
    def _update_timing_from_connected_managers(self):
        """Update timing information from connected time/simulation managers"""
        try:
            sim_time = 0.0
            real_time = 0.0
            
            # Priority 1: Get time from connected time manager
            if self.time_manager:
                stats = self.time_manager.get_timing_stats()
                sim_time = stats.sim_time
                real_time = stats.real_time_elapsed
            # Priority 2: Get time from connected simulation manager (legacy)
            elif self.simulation_manager:
                sim_time = self.simulation_manager.get_sim_time()
                real_time = self.simulation_manager.get_real_time()
            
            # Update shared memory with timing info
            if hasattr(self, 'shared_memory_manager') and self.shared_memory_manager:
                self.shared_memory_manager.update_timing_info(sim_time, real_time)
            
        except Exception as e:
            # Silent fail for timing updates
            pass
    
    def update_robot_pose(self, robot_name: str, pose: Pose) -> bool:
        """
        ロボットベースポーズを更新
        
        Args:
            robot_name: ロボット名
            pose: 新しいベースポーズ
            
        Returns:
            True if successful
        """
        if robot_name not in self.urdf_robots:
            return False
        
        try:
            robot = self.urdf_robots[robot_name]
            
            # ロボットのベースポーズを設定
            if hasattr(robot, 'set_base_pose'):
                robot.set_base_pose(pose)
            
            # 可視化を更新
            return self.update_robot_visualization(robot_name)
            
        except Exception as e:
            print(f"⚠️ Error updating robot pose: {e}")
            return False
    
    def update_robot_joints(self, robot_name: str, joint_positions: Dict[str, float]) -> bool:
        """
        ロボット関節位置を更新
        
        Args:
            robot_name: ロボット名
            joint_positions: 関節名 -> 位置のディクショナリ
            
        Returns:
            True if successful
        """
        if robot_name not in self.urdf_robots:
            return False
        
        try:
            robot = self.urdf_robots[robot_name]
            
            # ロボットの関節位置を設定
            if hasattr(robot, 'set_joint_positions'):
                robot.set_joint_positions(joint_positions)
            
            # 可視化を更新
            return self.update_robot_visualization(robot_name)
            
        except Exception as e:
            print(f"⚠️ Error updating robot joints: {e}")
            return False
    
    def add_simple_object(self, name: str, shape: str, pose: Pose, 
                         size: Any = 1.0, color: int = 0x888888) -> bool:
        """
        シンプルなオブジェクトを追加
        
        Note: プロセス分離PyVistaでは現在未実装
        """
        warnings.warn("add_simple_object not yet implemented for process-separated PyVista")
        return False
    
    def remove_object(self, name: str) -> bool:
        """
        オブジェクトを削除
        
        Note: プロセス分離PyVistaでは現在未実装
        """
        warnings.warn("remove_object not yet implemented for process-separated PyVista")
        return False
    
    def clear_all(self):
        """
        すべての可視化をクリア
        
        Note: プロセス分離PyVistaでは現在未実装
        """
        warnings.warn("clear_all not yet implemented for process-separated PyVista")
    
    def close(self):
        """ビジュアライザーを閉じる"""
        self.shutdown()
    
    def get_performance_stats(self) -> Dict:
        """パフォーマンス統計を取得（拡張版）"""
        stats = super().get_performance_stats()
        
        # URDF固有の統計追加
        stats['loaded_robots'] = len(self.urdf_robots)
        stats['total_links'] = sum(
            len(robot.urdf_loader.links) 
            for robot in self.urdf_robots.values()
            if hasattr(robot, 'urdf_loader') and robot.urdf_loader
        )
        stats['backend'] = 'process_separated_pyvista'
        
        return stats
    
    def print_performance_summary(self):
        """パフォーマンス概要を表示（拡張版）"""
        stats = self.get_performance_stats()
        
        print(f"\n📊 Process-Separated URDF Visualizer Performance Summary")
        print("=" * 60)
        print(f"Backend: {stats['backend']}")
        print(f"Process separation: ✅ ENABLED")
        print(f"Total time: {stats['total_time']:.1f}s")
        print(f"Update count: {stats['update_count']}")
        print(f"Loaded robots: {stats['loaded_robots']}")
        print(f"Total links: {stats['total_links']}")
        print(f"Avg update rate: {stats['avg_update_rate']:.1f} Hz")
        print(f"Avg update time: {stats['avg_update_time']:.4f}s")
        print(f"Shared memory size: {stats['shared_memory_size']} bytes")
        
        # パフォーマンス評価
        update_rate = stats['avg_update_rate']
        if update_rate > 50:
            rating = "🚀 EXCELLENT"
        elif update_rate > 30:
            rating = "✅ VERY GOOD"
        elif update_rate > 20:
            rating = "✅ GOOD"
        elif update_rate > 10:
            rating = "⚠️ FAIR"
        else:
            rating = "❌ POOR"
        
        print(f"Performance rating: {rating}")
        
        print(f"\n💡 Process Separation Benefits:")
        print(f"   ✅ Zero impact on SimPy simulation performance")
        print(f"   ✅ Crash isolation - PyVista errors don't propagate")
        print(f"   ✅ Independent OpenGL context")
        print(f"   ✅ Non-blocking visualization updates")
    
    def _extract_urdf_data(self, robot_data: Any) -> Dict:
        """URDFデータを抽出"""
        try:
            urdf_loader = robot_data.urdf_loader
            extracted_data = {
                'links': {},
                'joints': {},
                'name': getattr(robot_data, 'robot_name', 'unknown_robot')
            }
            
            # リンクデータ抽出
            for link_name, link in urdf_loader.links.items():
                link_data = {
                    'name': link_name,
                    'visuals': []
                }
                
                # Visual情報抽出
                if hasattr(link, 'visuals') and link.visuals:
                    for visual in link.visuals:
                        visual_data = {
                            'geometry': None,
                            'material': None,
                            'origin': None
                        }
                        
                        # Geometry情報
                        if hasattr(visual, 'geometry') and visual.geometry:
                            geom = visual.geometry
                            geom_type = type(geom).__name__
                            
                            geometry_data = {'type': geom_type}
                            
                            # 形状パラメータ
                            if hasattr(geom, 'radius'):
                                geometry_data['radius'] = float(geom.radius)
                            if hasattr(geom, 'length'):
                                geometry_data['length'] = float(geom.length)
                            if hasattr(geom, 'height'):
                                geometry_data['height'] = float(geom.height)
                            if hasattr(geom, 'size'):
                                if hasattr(geom.size, '__iter__'):
                                    geometry_data['size'] = list(geom.size)
                                else:
                                    geometry_data['size'] = float(geom.size)
                            
                            visual_data['geometry'] = geometry_data
                        
                        # Material情報
                        if hasattr(visual, 'material') and visual.material:
                            material = visual.material
                            material_data = {}
                            
                            if hasattr(material, 'color') and material.color is not None:
                                if hasattr(material.color, '__iter__'):
                                    material_data['color'] = list(material.color)
                                else:
                                    material_data['color'] = [0.7, 0.7, 0.7, 1.0]  # デフォルト
                            else:
                                material_data['color'] = [0.7, 0.7, 0.7, 1.0]  # デフォルト
                            
                            if hasattr(material, 'name'):
                                material_data['name'] = material.name
                                
                            visual_data['material'] = material_data
                        
                        # Origin情報
                        if hasattr(visual, 'origin') and visual.origin is not None:
                            origin = visual.origin
                            if hasattr(origin, 'xyz') and hasattr(origin, 'rpy'):
                                visual_data['origin'] = {
                                    'xyz': list(origin.xyz) if hasattr(origin.xyz, '__iter__') else [0, 0, 0],
                                    'rpy': list(origin.rpy) if hasattr(origin.rpy, '__iter__') else [0, 0, 0]
                                }
                        
                        link_data['visuals'].append(visual_data)
                
                extracted_data['links'][link_name] = link_data
            
            # ジョイントデータ抽出
            for joint_name, joint in urdf_loader.joints.items():
                joint_data = {
                    'name': joint_name,
                    'type': str(joint.joint_type),
                    'parent': getattr(joint, 'parent_link', getattr(joint, 'parent', 'unknown')),
                    'child': getattr(joint, 'child_link', getattr(joint, 'child', 'unknown')),
                    'origin': None
                }
                
                # Origin情報
                if hasattr(joint, 'origin') and joint.origin is not None:
                    origin = joint.origin
                    if hasattr(origin, 'xyz') and hasattr(origin, 'rpy'):
                        joint_data['origin'] = {
                            'xyz': list(origin.xyz) if hasattr(origin.xyz, '__iter__') else [0, 0, 0],
                            'rpy': list(origin.rpy) if hasattr(origin.rpy, '__iter__') else [0, 0, 0]
                        }
                
                extracted_data['joints'][joint_name] = joint_data
            
            return extracted_data
            
        except Exception as e:
            print(f"❌ URDF data extraction failed: {e}")
            import traceback
            traceback.print_exc()
            return {'links': {}, 'joints': {}, 'name': 'unknown_robot'}


def create_process_separated_urdf_visualizer(max_robots: int = 20, 
                                           max_links_per_robot: int = 30) -> ProcessSeparatedURDFRobotVisualizer:
    """プロセス分離URDFロボットビジュアライザーを作成"""
    config = SharedMemoryConfig(
        max_robots=max_robots,
        max_links_per_robot=max_links_per_robot,
        update_frequency=30.0
    )
    
    visualizer = ProcessSeparatedURDFRobotVisualizer(config)
    
    if visualizer.initialize():
        return visualizer
    else:
        raise RuntimeError("プロセス分離URDFビジュアライザー初期化失敗")


if __name__ == "__main__":
    print("🧪 Process-Separated URDF Robot Visualizer Test")
    print("=" * 50)
    
    try:
        # ビジュアライザー作成
        visualizer = create_process_separated_urdf_visualizer(max_robots=2, max_links_per_robot=10)
        
        print("✅ プロセス分離URDFビジュアライザー作成成功")
        
        # 基本テスト
        print("🧪 基本機能テスト...")
        
        # Mock robot for testing
        class MockRobot:
            def __init__(self):
                self.urdf_loader = MockURDFLoader()
                
            def get_link_poses(self):
                return {
                    'base_link': Pose(0, 0, 0),
                    'link1': Pose(0.1, 0, 0.1),
                    'link2': Pose(0.2, 0, 0.1)
                }
        
        class MockURDFLoader:
            def __init__(self):
                self.links = {
                    'base_link': None,
                    'link1': None, 
                    'link2': None
                }
                self.joints = {
                    'joint1': None,
                    'joint2': None
                }
        
        # テストロボット読み込み
        mock_robot = MockRobot()
        success = visualizer.load_robot("test_robot", mock_robot)
        
        if success:
            print("✅ ロボット読み込み成功")
            
            # 可視化更新テスト
            print("🔄 可視化更新テスト...")
            
            for i in range(5):
                success = visualizer.update_robot_visualization("test_robot")
                if success:
                    print(f"   更新 {i+1}/5 成功")
                else:
                    print(f"   更新 {i+1}/5 失敗")
            
            # パフォーマンス統計
            visualizer.print_performance_summary()
            
        else:
            print("❌ ロボット読み込み失敗")
        
        print("\n🎉 テスト完了！")
        
    except Exception as e:
        print(f"❌ テスト失敗: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            visualizer.close()
        except:
            pass