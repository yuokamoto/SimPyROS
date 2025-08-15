#!/usr/bin/env python3
"""
SimPy FrequencyGroup Manager - FrequencyGroup最適化版SimPy実装
既存のFrequencyGroup機能をラップした統一インターフェース
"""

from typing import Dict, List, Callable, Optional, Any
import time

from .base_simulation_manager import BaseSimulationManager, UnifiedSimulationConfig
from .simulation_object import Velocity, Pose
from .simulation_manager import SimulationManager, SimulationConfig


class SimPyFrequencyGroupManager(BaseSimulationManager):
    """SimPy FrequencyGroup最適化実装"""
    
    def __init__(self, config: UnifiedSimulationConfig):
        super().__init__(config)
        
        # 既存のSimulationManagerを内部で使用
        self.simpy_config = self._convert_to_simpy_config(config)
        self.sim_manager = SimulationManager(self.simpy_config)
        
        # 統計情報の橋渡し
        self._sync_stats_timer = 0.0
    
    def _initialize_backend(self):
        """SimPy FrequencyGroup固有の初期化"""
        if self.config.verbose:
            print(f"🔄 SimPy FrequencyGroup Manager initialized")
            print(f"   FrequencyGrouping: {self.simpy_config.enable_frequency_grouping}")
            print(f"   Real-time factor: {self.config.real_time_factor}")
    
    def _convert_to_simpy_config(self, unified_config: UnifiedSimulationConfig) -> SimulationConfig:
        """統一設定をSimPy設定に変換"""
        return SimulationConfig(
            visualization=unified_config.visualization,
            enable_frequency_grouping=unified_config.enable_frequency_grouping,
            update_rate=unified_config.update_rate,
            real_time_factor=unified_config.real_time_factor,
            strict=False  # 性能優先
        )
    
    # === ロボット管理 ===
    
    def add_robot_from_urdf(self, 
                           name: str, 
                           urdf_path: str,
                           initial_pose: Pose = None,
                           joint_update_rate: Optional[float] = None,
                           **kwargs) -> Any:
        """URDFからロボット追加"""
        
        if joint_update_rate is None:
            joint_update_rate = self.config.default_joint_update_rate
        
        robot = self.sim_manager.add_robot_from_urdf(
            name=name,
            urdf_path=urdf_path,
            initial_pose=initial_pose or Pose(),
            joint_update_rate=joint_update_rate,
            unified_process=True,
            frequency_grouping_managed=self.config.enable_frequency_grouping
        )
        
        # 内部状態同期
        self.robots[name] = robot
        
        if self.config.verbose:
            print(f"✅ Robot '{name}' added via SimPy ({len(self.robots)} total)")
        
        return robot
    
    def set_robot_velocity(self, robot_name: str, velocity: Velocity) -> bool:
        """ロボット速度設定"""
        result = self.sim_manager.set_robot_velocity(robot_name, velocity)
        return result
    
    def set_robot_control_callback(self, 
                                  robot_name: str, 
                                  callback: Callable, 
                                  frequency: float = 10.0) -> bool:
        """ロボット制御コールバック設定"""
        
        result = self.sim_manager.set_robot_control_callback(
            robot_name, callback, frequency
        )
        
        if result:
            self.control_callbacks[robot_name] = callback
            
            if self.config.verbose:
                print(f"✅ Control callback set for '{robot_name}' at {frequency} Hz via SimPy")
        
        return result
    
    def get_robot_pose(self, robot_name: str) -> Optional[Pose]:
        """ロボットポーズ取得"""
        if robot_name in self.robots:
            robot = self.robots[robot_name]
            return robot.get_pose()
        return None
    
    # === シミュレーション制御 ===
    
    def run(self, duration: Optional[float] = None):
        """シミュレーション実行"""
        
        if self.config.verbose:
            print(f"🔄 SimPy FrequencyGroup simulation starting")
            print(f"   Robots: {len(self.robots)}")
            print(f"   Duration: {duration}s" if duration else "   Duration: Unlimited")
            print("=" * 50)
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        try:
            # SimPy実行
            self.sim_manager.run(duration=duration)
            
            # 統計情報同期
            self._sync_stats_from_simpy()
            
        except KeyboardInterrupt:
            if self.config.verbose:
                print("\n⏹️ SimPy simulation interrupted")
        
        finally:
            self.running = False
            if self.config.performance_monitoring:
                self.print_performance_summary()
    
    def _sync_stats_from_simpy(self):
        """SimPy統計情報の同期"""
        try:
            # SimPyの統計情報を取得して同期
            simpy_stats = self.sim_manager.get_timing_stats()
            
            self.stats['sim_time'] = self.sim_manager.get_sim_time()
            self.stats['frame_count'] = getattr(self.sim_manager, 'frame_count', 0)
            
            # FrequencyGroup統計情報
            if hasattr(self.sim_manager, '_frequency_groups'):
                total_callbacks = 0
                for group_info in self.sim_manager._frequency_groups.values():
                    total_callbacks += len(group_info.get('callbacks', []))
                
                # 概算コールバック数
                elapsed = time.time() - self.stats['start_time']
                if elapsed > 0:
                    self.stats['total_callbacks'] = int(total_callbacks * elapsed * 10)  # 概算
            
        except Exception as e:
            if self.config.verbose:
                print(f"⚠️ Stats sync failed: {e}")
    
    def pause(self):
        """シミュレーション一時停止"""
        # SimPy実装では一時停止機能は限定的
        if self.config.verbose:
            print("⚠️ Pause not fully supported in SimPy backend")
    
    def resume(self):
        """シミュレーション再開"""
        if self.config.verbose:
            print("⚠️ Resume not fully supported in SimPy backend")
    
    def shutdown(self):
        """シミュレーション終了"""
        self.running = False
        
        try:
            self.sim_manager.shutdown()
        except Exception as e:
            if self.config.verbose:
                print(f"⚠️ SimPy shutdown error: {e}")
        
        if self.config.verbose:
            print("🛑 SimPy FrequencyGroup simulation shutdown")
    
    # === 時間管理 ===
    
    def get_sim_time(self) -> float:
        """シミュレーション時間取得"""
        return self.sim_manager.get_sim_time()
    
    def set_real_time_factor(self, factor: float):
        """リアルタイム係数設定"""
        self.config.real_time_factor = factor
        self.sim_manager.set_real_time_factor(factor)
        
        if self.config.verbose:
            print(f"🕐 SimPy Real-time factor set to {factor}x")


class SimPyPureManager(BaseSimulationManager):
    """純粋SimPy実装（FrequencyGroup無し）"""
    
    def __init__(self, config: UnifiedSimulationConfig):
        super().__init__(config)
        
        # 純粋SimPy設定
        self.simpy_config = SimulationConfig(
            visualization=config.visualization,
            enable_frequency_grouping=False,  # 純粋SimPy
            update_rate=config.update_rate,
            real_time_factor=config.real_time_factor,
            strict=False
        )
        
        self.sim_manager = SimulationManager(self.simpy_config)
    
    def _initialize_backend(self):
        """Pure SimPy固有の初期化"""
        if self.config.verbose:
            print(f"🎭 Pure SimPy Manager initialized")
            print(f"   FrequencyGrouping: DISABLED (pure SimPy)")
            print(f"   Maximum SimPy features enabled")
    
    # ロボット管理メソッドはSimPyFrequencyGroupManagerと同様
    # 簡潔性のため継承で実装
    
    def add_robot_from_urdf(self, 
                           name: str, 
                           urdf_path: str,
                           initial_pose: Pose = None,
                           joint_update_rate: Optional[float] = None,
                           **kwargs) -> Any:
        """URDFからロボット追加（純粋SimPy）"""
        
        if joint_update_rate is None:
            joint_update_rate = self.config.default_joint_update_rate
        
        robot = self.sim_manager.add_robot_from_urdf(
            name=name,
            urdf_path=urdf_path,
            initial_pose=initial_pose or Pose(),
            joint_update_rate=joint_update_rate,
            unified_process=False,  # 純粋SimPy：独立プロセス
            frequency_grouping_managed=False
        )
        
        self.robots[name] = robot
        
        if self.config.verbose:
            print(f"✅ Robot '{name}' added via Pure SimPy ({len(self.robots)} total)")
        
        return robot
    
    # 他のメソッドは基本的にSimPyFrequencyGroupManagerと同じ
    # （実装の簡潔性のため一部省略）
    
    def set_robot_velocity(self, robot_name: str, velocity: Velocity) -> bool:
        return self.sim_manager.set_robot_velocity(robot_name, velocity)
    
    def set_robot_control_callback(self, robot_name: str, callback: Callable, frequency: float = 10.0) -> bool:
        result = self.sim_manager.set_robot_control_callback(robot_name, callback, frequency)
        if result:
            self.control_callbacks[robot_name] = callback
        return result
    
    def get_robot_pose(self, robot_name: str) -> Optional[Pose]:
        if robot_name in self.robots:
            return self.robots[robot_name].get_pose()
        return None
    
    def run(self, duration: Optional[float] = None):
        if self.config.verbose:
            print(f"🎭 Pure SimPy simulation starting")
            print("   Maximum event-driven features enabled")
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        try:
            self.sim_manager.run(duration=duration)
            self.stats['sim_time'] = self.sim_manager.get_sim_time()
        finally:
            self.running = False
            if self.config.performance_monitoring:
                self.print_performance_summary()
    
    def pause(self):
        if self.config.verbose:
            print("⚠️ Pause not supported in Pure SimPy backend")
    
    def resume(self):
        if self.config.verbose:
            print("⚠️ Resume not supported in Pure SimPy backend")
    
    def shutdown(self):
        self.running = False
        try:
            self.sim_manager.shutdown()
        except:
            pass
        
        if self.config.verbose:
            print("🛑 Pure SimPy simulation shutdown")
    
    def get_sim_time(self) -> float:
        return self.sim_manager.get_sim_time()
    
    def set_real_time_factor(self, factor: float):
        self.config.real_time_factor = factor
        self.sim_manager.set_real_time_factor(factor)