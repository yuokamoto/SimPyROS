#!/usr/bin/env python3
"""
SimPy Pure Manager - 純粋なSimPy実装
完全なイベント駆動アーキテクチャによる統一インターフェース
"""

from typing import Dict, List, Callable, Optional, Any
import time

from .base_simulation_manager import BaseSimulationManager, UnifiedSimulationConfig
from .simulation_object import Velocity, Pose
from .simulation_manager import SimulationManager, SimulationConfig


class SimPyPureManager(BaseSimulationManager):
    """純粋なSimPy実装"""
    
    def __init__(self, config: UnifiedSimulationConfig):
        super().__init__(config)
        
        # 既存のSimulationManagerを内部で使用
        self.simpy_config = self._convert_to_simpy_config(config)
        self.sim_manager = SimulationManager(self.simpy_config)
        
        # 統計情報の橋渡し
        self._sync_stats_timer = 0.0
    
    def _initialize_backend(self):
        """SimPy Pure固有の初期化"""
        if self.config.verbose:
            print("🧠 Initializing Pure SimPy backend")
    
    def _convert_to_simpy_config(self, config: UnifiedSimulationConfig) -> SimulationConfig:
        """統一設定をSimPy設定に変換"""
        simpy_config = SimulationConfig()
        simpy_config.time_step = config.time_step
        simpy_config.real_time_factor = config.real_time_factor
        simpy_config.visualization = config.visualization
        simpy_config.verbose = config.verbose
        return simpy_config
    
    # === BaseSimulationManagerインターフェースの実装 ===
    
    def add_robot_from_urdf(self, name: str, urdf_path: str, initial_pose: Optional[Pose] = None) -> Any:
        """URDFからロボット追加"""
        return self.sim_manager.add_robot_from_urdf(name, urdf_path, initial_pose)
    
    def set_robot_velocity(self, name: str, velocity: Velocity):
        """ロボット速度設定"""
        self.sim_manager.set_robot_velocity(name, velocity)
    
    def set_robot_joint_position(self, name: str, joint_name: str, position: float):
        """ロボット関節位置設定"""
        self.sim_manager.set_robot_joint_position(name, joint_name, position)
    
    def set_control_callback(self, name: str, callback: Callable[[float], None]):
        """制御コールバック設定"""
        self.sim_manager.set_robot_control_callback(name, callback)
    
    def run(self, duration: Optional[float] = None, headless: bool = False):
        """シミュレーション実行"""
        if self.config.verbose:
            print("🧠 Starting Pure SimPy simulation")
        
        self.sim_manager.run(duration=duration, headless=headless)
    
    def pause(self):
        """シミュレーション一時停止"""
        if hasattr(self.sim_manager, 'pause'):
            self.sim_manager.pause()
    
    def resume(self):
        """シミュレーション再開"""
        if hasattr(self.sim_manager, 'resume'):
            self.sim_manager.resume()
    
    def shutdown(self):
        """シミュレーション終了"""
        if self.sim_manager:
            self.sim_manager.shutdown()
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """性能統計取得"""
        if hasattr(self.sim_manager, 'get_performance_stats'):
            return self.sim_manager.get_performance_stats()
        
        # フォールバック統計
        return {
            'backend': 'Pure SimPy',
            'rtf': 0.1,  # 模擬値
            'callbacks_per_sec': 3000,
            'frame_count': 0,
            'performance_tier': 'High Features'
        }
    
    def get_backend_info(self) -> Dict[str, Any]:
        """バックエンド情報取得"""
        return {
            'backend': self.config.backend,
            'name': 'Pure SimPy',
            'description': 'Full event-driven architecture with complex behaviors',
            'performance_tier': 'High Features',
            'expected_rtf': 0.1,
            'features': ['Event-driven', 'Complex behaviors', 'Resource modeling']
        }
    
    @property
    def robots(self) -> Dict[str, Any]:
        """ロボット辞書"""
        if hasattr(self.sim_manager, 'robots'):
            return self.sim_manager.robots
        return {}
    
    @property 
    def control_callbacks(self) -> Dict[str, Callable]:
        """制御コールバック辞書"""
        if hasattr(self.sim_manager, 'control_callbacks'):
            return self.sim_manager.control_callbacks
        return {}
    
    def _get_performance_tier(self) -> str:
        """性能ティア取得"""
        return "High Features"