#!/usr/bin/env python3
"""
Process-Separated URDF Robot Visualizer

URDFRobotVisualizerインターフェースに準拠したプロセス分離PyVistaビジュアライザー
最適化されたgeometry + pose更新アーキテクチャ
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
    
    Features:
    - SimulationManagerとの完全互換性
    - 標準PyVistaVisualizerロジック使用
    - 最適化されたデータ送信(geometry一回 + pose継続)
    """
    
    def __init__(self, config: Optional[SharedMemoryConfig] = None):
        super().__init__(config)
        
        # URDF robot tracking
        self.urdf_robots = {}  # robot_name -> robot_instance
        self.robot_link_poses = {}  # robot_name -> {link_name -> pose}
        
        # Time management
        self.time_manager = None
        self.simulation_manager = None
        
        # Performance tracking
        self.update_count = 0
        self.last_update_time = 0
        
        # Compatibility attributes
        self.available = True  # For SimulationManager compatibility
        
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
            
            # 初期化が必要
            if not self.is_initialized:
                print("🚀 Initializing visualizer...")
                init_success = self.initialize()
                if not init_success:
                    print(f"❌ Failed to initialize visualizer")
                    return False
            
            # ロボットを追加（geometry送信）
            success = self.add_robot(robot_name, robot_data)
            
            if success:
                # リンクポーズ管理初期化
                self.robot_link_poses[robot_name] = {}
                for link_name in robot_data.links.keys():
                    self.robot_link_poses[robot_name][link_name] = Pose()
                
                print(f"✅ URDF Robot loaded: {robot_name} ({len(robot_data.links)} links)")
                return True
            else:
                print(f"❌ Failed to add robot to visualizer: {robot_name}")
                return False
                
        except Exception as e:
            print(f"❌ Robot loading failed: {robot_name} - {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def update_robot_visualization(self, robot_name: str, robot_data: Any = None, force_render: bool = False):
        """
        ロボットの可視化を更新（SimulationManager互換インターフェース）
        
        Args:
            robot_name: ロボット名
            robot_data: Robot instance with current poses (optional, will use stored robot)
            force_render: Force rendering (compatibility parameter)
        """
        if robot_name not in self.urdf_robots:
            return
            
        try:
            # Use provided robot_data or stored robot instance
            if robot_data is None:
                robot_data = self.urdf_robots[robot_name]
                
            # ロボットからリンクポーズを取得
            if hasattr(robot_data, 'get_link_poses'):
                link_poses = robot_data.get_link_poses()
                
                # 高速pose更新（shared memory経由）
                success = self.update_robot_poses(robot_name, link_poses)
                
                if success:
                    self.update_count += 1
                    self.last_update_time = self._get_current_time()
                    
                    # デバッグ出力（間引き）
                    if self.update_count <= 5 or self.update_count % 100 == 0:
                        print(f"🔄 Pose update #{self.update_count}: {robot_name} ({len(link_poses)} links)")
                        
            else:
                print(f"⚠️ Robot {robot_name} has no get_link_poses method")
                
        except Exception as e:
            print(f"❌ Visualization update failed for {robot_name}: {e}")
    
    def remove_robot(self, robot_name: str):
        """ロボットを削除"""
        if robot_name in self.urdf_robots:
            del self.urdf_robots[robot_name]
            
        if robot_name in self.robot_link_poses:
            del self.robot_link_poses[robot_name]
            
        if robot_name in self.robots:
            del self.robots[robot_name]
            
        print(f"🗑️ Robot removed: {robot_name}")
    
    def get_robot_names(self) -> List[str]:
        """登録されているロボット名のリストを取得"""
        return list(self.urdf_robots.keys())
    
    def is_robot_loaded(self, robot_name: str) -> bool:
        """ロボットが読み込まれているかチェック"""
        return robot_name in self.urdf_robots
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """パフォーマンス統計を取得"""
        if not self.is_initialized:
            return {}
            
        current_time = self._get_current_time()
        
        return {
            'update_count': self.update_count,
            'last_update_time': self.last_update_time,
            'total_robots': len(self.urdf_robots),
            'active_robots': len([r for r in self.urdf_robots.keys() if r in self.robots]),
            'current_time': current_time,
            'architecture': 'process_separated'
        }
    
    def print_performance_summary(self):
        """パフォーマンス統計を表示"""
        stats = self.get_performance_stats()
        
        print("📊 ProcessSeparatedURDFRobotVisualizer Performance:")
        print(f"   Total robots: {stats.get('total_robots', 0)}")
        print(f"   Active robots: {stats.get('active_robots', 0)}")
        print(f"   Update count: {stats.get('update_count', 0)}")
        print(f"   Architecture: {stats.get('architecture', 'unknown')}")
        
        if self.pose_manager:
            print(f"   Shared memory: {self.pose_manager.shm_name}")
    
    def _get_current_time(self) -> float:
        """現在時刻を取得（TimeManagerがあれば使用）"""
        if self.time_manager and hasattr(self.time_manager, 'get_sim_time'):
            return self.time_manager.get_sim_time()
        else:
            import time
            return time.time()


def create_process_separated_urdf_visualizer(config: Optional[SharedMemoryConfig] = None):
    """
    Factory function to create the URDF visualizer
    
    Args:
        config: Shared memory configuration
        
    Returns:
        ProcessSeparatedURDFRobotVisualizer instance
    """
    return ProcessSeparatedURDFRobotVisualizer(config)


if __name__ == "__main__":
    # Test the URDF visualizer
    print("🧪 Testing ProcessSeparatedURDFRobotVisualizer")
    
    config = SharedMemoryConfig(max_robots=2, max_links_per_robot=8)
    visualizer = ProcessSeparatedURDFRobotVisualizer(config)
    
    if visualizer.initialize():
        print("✅ Test passed - URDF visualizer initialized")
        
        # Print initial stats
        visualizer.print_performance_summary()
        
        import time
        time.sleep(3.0)
        
        visualizer.shutdown()
        print("✅ Test completed")
    else:
        print("❌ Test failed - initialization error")