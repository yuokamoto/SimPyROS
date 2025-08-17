#!/usr/bin/env python3
"""
Base Simulation Manager - 統一インターフェース
SimPy使用/非使用を切り替え可能な抽象基底クラス
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Callable, Optional, Any, Union
from dataclasses import dataclass
from enum import Enum
import time

from .simulation_object import Velocity, Pose


class SimulationBackend(Enum):
    """シミュレーションバックエンド選択"""
    SIMPLE_WHILE_LOOP = "simple_while_loop"  # Pure Python, 最高性能
    SIMPY_FREQUENCY_GROUP = "simpy_frequency_group"  # SimPy最適化版
    SIMPY_PURE = "simpy_pure"  # 純粋SimPy


@dataclass
class UnifiedSimulationConfig:
    """統一シミュレーション設定"""
    # Backend選択
    backend: SimulationBackend = SimulationBackend.SIMPLE_WHILE_LOOP
    
    # 基本設定
    visualization: bool = False
    update_rate: float = 30.0  # Hz
    real_time_factor: float = 1.0  # 0.0 = maximum speed
    
    # 性能設定
    enable_frequency_grouping: bool = True  # SimPy使用時の最適化
    default_joint_update_rate: float = 10.0  # デフォルトロボット更新頻度
    
    # デバッグ設定
    verbose: bool = False
    performance_monitoring: bool = True


class BaseSimulationManager(ABC):
    """統一シミュレーション管理インターフェース"""
    
    def __init__(self, config: UnifiedSimulationConfig):
        self.config = config
        self.robots: Dict[str, Any] = {}
        self.simulation_objects: Dict[str, Any] = {}
        self.control_callbacks: Dict[str, Callable] = {}
        
        # 統計情報
        self.stats = {
            'start_time': 0.0,
            'sim_time': 0.0,
            'frame_count': 0,
            'total_callbacks': 0,
            'total_updates': 0
        }
        
        self.running = False
        
        # 実装固有の初期化
        self._initialize_backend()
    
    @abstractmethod
    def _initialize_backend(self):
        """バックエンド固有の初期化"""
        pass
    
    # === ロボット管理 ===
    
    @abstractmethod
    def add_robot_from_urdf(self, 
                           name: str, 
                           urdf_path: str,
                           initial_pose: Pose = None,
                           joint_update_rate: Optional[float] = None,
                           **kwargs) -> Any:
        """URDFからロボット追加"""
        pass
    
    @abstractmethod
    def set_robot_velocity(self, robot_name: str, velocity: Velocity) -> bool:
        """ロボット速度設定"""
        pass
    
    @abstractmethod
    def set_robot_control_callback(self, 
                                  robot_name: str, 
                                  callback: Callable, 
                                  frequency: float = 10.0) -> bool:
        """ロボット制御コールバック設定"""
        pass
    
    @abstractmethod
    def get_robot_pose(self, robot_name: str) -> Optional[Pose]:
        """ロボットポーズ取得"""
        pass
    
    # === シミュレーション制御 ===
    
    @abstractmethod
    def run(self, duration: Optional[float] = None):
        """シミュレーション実行"""
        pass
    
    @abstractmethod
    def pause(self):
        """シミュレーション一時停止"""
        pass
    
    @abstractmethod
    def resume(self):
        """シミュレーション再開"""
        pass
    
    @abstractmethod
    def shutdown(self):
        """シミュレーション終了"""
        pass
    
    # === 時間管理 ===
    
    @abstractmethod
    def get_sim_time(self) -> float:
        """シミュレーション時間取得"""
        pass
    
    @abstractmethod
    def set_real_time_factor(self, factor: float):
        """リアルタイム係数設定"""
        pass
    
    # === 統計情報 ===
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """性能統計取得"""
        current_time = time.time()
        elapsed = current_time - self.stats['start_time'] if self.stats['start_time'] > 0 else 0.001
        
        return {
            'backend': self.config.backend.value,
            'elapsed_time': elapsed,
            'sim_time': self.stats['sim_time'],
            'rtf': self.stats['sim_time'] / elapsed if elapsed > 0 else 0,
            'frame_count': self.stats['frame_count'],
            'avg_fps': self.stats['frame_count'] / elapsed if elapsed > 0 else 0,
            'total_callbacks': self.stats['total_callbacks'],
            'callbacks_per_sec': self.stats['total_callbacks'] / elapsed if elapsed > 0 else 0,
            'num_robots': len(self.robots),
            'num_objects': len(self.simulation_objects)
        }
    
    def print_performance_summary(self):
        """性能サマリー表示"""
        stats = self.get_performance_stats()
        
        print(f"\n📊 {stats['backend'].upper()} Performance Summary")
        print("=" * 50)
        print(f"Backend: {stats['backend']}")
        print(f"Robots: {stats['num_robots']}")
        print(f"Real Time Factor: {stats['rtf']:.3f}x")
        print(f"Frame Rate: {stats['avg_fps']:.1f} Hz")
        print(f"Callbacks/sec: {stats['callbacks_per_sec']:.1f}")
        
        # 性能評価
        if stats['rtf'] >= 1.0:
            rating = "🚀 EXCELLENT"
        elif stats['rtf'] >= 0.5:
            rating = "✅ GOOD"
        elif stats['rtf'] >= 0.1:
            rating = "⚠️ FAIR"
        else:
            rating = "❌ POOR"
        
        print(f"Performance Rating: {rating}")
    
    # === ユーティリティ ===
    
    def get_backend_info(self) -> Dict[str, Any]:
        """バックエンド情報取得"""
        return {
            'backend': self.config.backend,
            'description': self._get_backend_description(),
            'features': self._get_backend_features(),
            'performance_tier': self._get_performance_tier()
        }
    
    def _get_backend_description(self) -> str:
        """バックエンド説明"""
        descriptions = {
            SimulationBackend.SIMPLE_WHILE_LOOP: "Pure Python while loop - 最高性能、最小複雑度",
            SimulationBackend.SIMPY_FREQUENCY_GROUP: "SimPy with FrequencyGroup optimization - 高性能+イベント処理",
            SimulationBackend.SIMPY_PURE: "Pure SimPy - 最大機能性、複雑なイベント処理対応"
        }
        return descriptions.get(self.config.backend, "Unknown backend")
    
    def _get_backend_features(self) -> List[str]:
        """バックエンド機能"""
        features = {
            SimulationBackend.SIMPLE_WHILE_LOOP: [
                "最高性能 (RTF ~1.0x)",
                "最小依存関係",
                "デバッグ容易",
                "基本的なロボット制御"
            ],
            SimulationBackend.SIMPY_FREQUENCY_GROUP: [
                "高性能 (RTF ~0.1-0.5x)",
                "プロセス最適化",
                "リアルタイム係数制御",
                "イベント処理対応"
            ],
            SimulationBackend.SIMPY_PURE: [
                "フル機能 (RTF ~0.05x)",
                "複雑なイベント処理",
                "リソース管理",
                "非同期通信"
            ]
        }
        return features.get(self.config.backend, [])
    
    def _get_performance_tier(self) -> str:
        """性能ティア"""
        tiers = {
            SimulationBackend.SIMPLE_WHILE_LOOP: "S (最高性能)",
            SimulationBackend.SIMPY_FREQUENCY_GROUP: "A (高性能)",
            SimulationBackend.SIMPY_PURE: "B (機能重視)"
        }
        return tiers.get(self.config.backend, "Unknown")


# === ファクトリー関数 ===

def create_simulation_manager(config: UnifiedSimulationConfig) -> BaseSimulationManager:
    """シミュレーション管理クラスのファクトリー"""
    
    if config.backend == SimulationBackend.SIMPLE_WHILE_LOOP:
        from .simple_while_loop_manager import SimpleWhileLoopManager
        return SimpleWhileLoopManager(config)
    
    elif config.backend == SimulationBackend.SIMPY_FREQUENCY_GROUP:
        from .simpy_frequency_group_manager import SimPyFrequencyGroupManager
        return SimPyFrequencyGroupManager(config)
    
    elif config.backend == SimulationBackend.SIMPY_PURE:
        from .simpy_pure_manager import SimPyPureManager
        return SimPyPureManager(config)
    
    else:
        raise ValueError(f"Unknown backend: {config.backend}")


# === 便利関数 ===

def quick_simulation(num_robots: int = 10, 
                    backend: SimulationBackend = SimulationBackend.SIMPLE_WHILE_LOOP,
                    visualization: bool = False,
                    duration: float = 10.0) -> Dict[str, Any]:
    """クイックシミュレーション実行"""
    
    config = UnifiedSimulationConfig(
        backend=backend,
        visualization=visualization,
        real_time_factor=0.0 if not visualization else 1.0
    )
    
    sim = create_simulation_manager(config)
    
    try:
        # ロボット追加
        for i in range(num_robots):
            x = (i % 10) * 2.0
            y = (i // 10) * 2.0
            
            robot = sim.add_robot_from_urdf(
                name=f"robot_{i}",
                urdf_path="examples/robots/mobile_robot.urdf",
                initial_pose=Pose(x=x, y=y, z=0)
            )
            
            # 簡単なコントローラ
            def create_controller(robot_id):
                def controller(dt):
                    t = sim.get_sim_time()
                    velocity = Velocity(
                        linear_x=0.1,
                        angular_z=0.1 if robot_id % 2 == 0 else -0.1
                    )
                    sim.set_robot_velocity(f"robot_{robot_id}", velocity)
                return controller
            
            sim.set_robot_control_callback(f"robot_{i}", create_controller(i))
        
        # 実行
        sim.run(duration=duration)
        
        return sim.get_performance_stats()
        
    finally:
        sim.shutdown()


def compare_backends(num_robots: int = 20, duration: float = 3.0) -> Dict[str, Dict[str, Any]]:
    """バックエンド性能比較"""
    
    print(f"🔍 Backend Performance Comparison ({num_robots} robots)")
    print("=" * 60)
    
    backends = [
        SimulationBackend.SIMPLE_WHILE_LOOP,
        SimulationBackend.SIMPY_FREQUENCY_GROUP
    ]
    
    results = {}
    
    for backend in backends:
        print(f"\n🧪 Testing {backend.value}...")
        
        try:
            stats = quick_simulation(
                num_robots=num_robots,
                backend=backend,
                visualization=False,
                duration=duration
            )
            
            results[backend.value] = stats
            print(f"   RTF: {stats['rtf']:.3f}x")
            
        except Exception as e:
            print(f"   ❌ Failed: {e}")
            results[backend.value] = None
    
    return results