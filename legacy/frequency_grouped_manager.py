#!/usr/bin/env python3
"""
Frequency-Grouped Simulation Manager - SimPyROS

Core implementation of frequency-based process grouping optimization.
Groups robots with identical update frequencies into shared SimPy processes,
dramatically reducing scheduling overhead.

Key Benefits:
- Reduces N processes to F processes (where F = unique frequencies)
- Single yield per frequency group instead of per robot
- Massive performance improvement for scenarios with repeated frequencies
- Maintains exact same behavior as individual processes
"""

import time
from collections import defaultdict
from typing import Dict, List, Callable, Any, Optional
from dataclasses import dataclass

import simpy
from core.simulation_manager import SimulationManager, SimulationConfig
from core.robot import Robot
from core.simulation_object import Pose


@dataclass
class FrequencyGroup:
    """周波数グループの情報"""
    frequency: float
    robots: List[Dict[str, Any]]
    process: Optional[Any] = None
    callback_count: int = 0
    

class FrequencyGroupedManager:
    """周波数でグループ化されたシミュレーションマネージャ"""
    
    def __init__(self, config: SimulationConfig):
        """初期化"""
        self.base_sim = SimulationManager(config)
        self.env = self.base_sim.env
        
        # 周波数グループ管理
        self.frequency_groups: Dict[float, FrequencyGroup] = {}
        self.robot_to_group: Dict[str, float] = {}
        
        # パフォーマンス統計
        self.total_callbacks = 0
        self.active_processes = 0
        
    def add_robot_from_urdf(self, name: str, urdf_path: str, 
                          initial_pose: Pose = None, **kwargs) -> Robot:
        """URDFからロボットを追加（基本的なロボット作成のみ）"""
        
        # unified_process=Falseにして個別プロセス作成を回避
        kwargs['unified_process'] = False
        
        robot = self.base_sim.add_robot_from_urdf(
            name=name, 
            urdf_path=urdf_path, 
            initial_pose=initial_pose,
            **kwargs
        )
        
        return robot
    
    def add_robot_to_frequency_group(self, robot_name: str, robot_instance: Robot,
                                   controller: Callable, frequency: float):
        """ロボットを指定周波数のグループに追加"""
        
        # 新しい周波数グループを作成
        if frequency not in self.frequency_groups:
            self.frequency_groups[frequency] = FrequencyGroup(
                frequency=frequency,
                robots=[]
            )
        
        # ロボット情報をグループに追加
        robot_info = {
            'name': robot_name,
            'instance': robot_instance,
            'controller': controller,
            'callback_count': 0
        }
        
        self.frequency_groups[frequency].robots.append(robot_info)
        self.robot_to_group[robot_name] = frequency
        
        return len(self.frequency_groups[frequency].robots)
    
    def create_frequency_group_process(self, frequency: float) -> Any:
        """指定周波数の統合プロセスを作成"""
        
        group = self.frequency_groups[frequency]
        if not group.robots:
            return None
        
        def frequency_group_process():
            """統合周波数プロセス - 同じ周波数の全ロボットを一括処理"""
            
            dt = 1.0 / frequency
            robots = group.robots
            
            while True:
                # 同じ周波数の全ロボットを同じタイミングで一括処理
                for robot_info in robots:
                    try:
                        # コントローラを実行
                        robot_info['controller'](dt)
                        
                        # 統計更新
                        robot_info['callback_count'] += 1
                        group.callback_count += 1
                        self.total_callbacks += 1
                        
                    except Exception as e:
                        print(f"❌ Controller error for {robot_info['name']}: {e}")
                
                # 重要: 一度のyieldで全ロボット処理完了
                # これによりSimPyのスケジューリングオーバーヘッドが大幅削減
                yield self.env.timeout(dt)
        
        # プロセスをSimPy環境に登録
        process = self.env.process(frequency_group_process())
        group.process = process
        
        return process
    
    def start_all_frequency_groups(self):
        """全周波数グループのプロセスを開始"""
        
        active_frequencies = list(self.frequency_groups.keys())
        self.active_processes = 0
        
        print(f"🔄 Starting frequency-grouped processes...")
        
        total_robots = 0
        for frequency in sorted(active_frequencies):
            group = self.frequency_groups[frequency]
            robot_count = len(group.robots)
            total_robots += robot_count
            
            print(f"   {frequency:6.1f} Hz: {robot_count:3d} robots")
            
            # プロセス作成・開始
            process = self.create_frequency_group_process(frequency)
            if process:
                self.active_processes += 1
        
        print(f"Process optimization achieved:")
        print(f"   Traditional approach: {total_robots} processes")
        print(f"   Frequency-grouped: {self.active_processes} processes")
        print(f"   Reduction: {total_robots - self.active_processes} processes "
              f"({(1 - self.active_processes/total_robots)*100:.1f}%)")
        
        return self.active_processes
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """パフォーマンス統計を取得"""
        
        stats = {
            'total_callbacks': self.total_callbacks,
            'active_processes': self.active_processes,
            'total_robots': sum(len(group.robots) for group in self.frequency_groups.values()),
            'frequency_groups': {},
            'process_reduction_percent': 0
        }
        
        total_robots = stats['total_robots']
        if total_robots > 0:
            stats['process_reduction_percent'] = (1 - self.active_processes / total_robots) * 100
        
        # 周波数別統計
        for frequency, group in self.frequency_groups.items():
            stats['frequency_groups'][frequency] = {
                'robot_count': len(group.robots),
                'total_callbacks': group.callback_count,
                'robots': [
                    {
                        'name': robot['name'],
                        'callback_count': robot['callback_count']
                    } for robot in group.robots
                ]
            }
        
        return stats
    
    def set_robot_velocity(self, robot_name: str, velocity):
        """ロボットの速度を設定"""
        return self.base_sim.set_robot_velocity(robot_name, velocity)
    
    def set_robot_joint_position(self, robot_name: str, joint_name: str, position: float):
        """ロボットの関節位置を設定"""
        return self.base_sim.set_robot_joint_position(robot_name, joint_name, position)
    
    def get_sim_time(self) -> float:
        """シミュレーション時間を取得"""
        return self.base_sim.get_sim_time()
    
    def run(self, duration: float = None):
        """シミュレーション実行"""
        return self.base_sim.run(duration=duration)
    
    def shutdown(self):
        """シャットダウン"""
        try:
            self.base_sim.shutdown()
        except:
            pass


class FrequencyOptimizer:
    """周波数最適化ユーティリティ"""
    
    @staticmethod
    def analyze_frequency_distribution(frequencies: List[float]) -> Dict[str, Any]:
        """周波数分布を分析して最適化効果を予測"""
        
        from collections import Counter
        freq_counts = Counter(frequencies)
        
        total_robots = len(frequencies)
        unique_frequencies = len(freq_counts)
        
        # 最適化効果の計算
        process_reduction = total_robots - unique_frequencies
        reduction_percent = (process_reduction / total_robots) * 100 if total_robots > 0 else 0
        
        # 周波数利用効率
        most_common = freq_counts.most_common(3)
        
        analysis = {
            'total_robots': total_robots,
            'unique_frequencies': unique_frequencies,
            'process_reduction': process_reduction,
            'reduction_percent': reduction_percent,
            'frequency_distribution': dict(freq_counts),
            'most_common_frequencies': most_common,
            'optimization_effectiveness': 'Excellent' if reduction_percent > 80 else
                                       'Very Good' if reduction_percent > 60 else
                                       'Good' if reduction_percent > 40 else
                                       'Moderate' if reduction_percent > 20 else
                                       'Limited'
        }
        
        return analysis
    
    @staticmethod
    def recommend_frequencies(num_robots: int, target_frequencies: int = 4) -> List[float]:
        """最適な周波数セットを推奨"""
        
        # 一般的なロボット制御周波数
        common_frequencies = [5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 50.0]
        
        # ターゲット数まで選択
        selected = common_frequencies[:min(target_frequencies, len(common_frequencies))]
        
        return selected
    
    @staticmethod
    def create_frequency_assignment(num_robots: int, frequencies: List[float]) -> List[float]:
        """ロボットに周波数を割り当て"""
        
        # 均等分散での割り当て
        assignments = []
        for i in range(num_robots):
            freq = frequencies[i % len(frequencies)]
            assignments.append(freq)
        
        return assignments


# 使いやすいヘルパー関数
def create_frequency_grouped_simulation(config: SimulationConfig) -> FrequencyGroupedManager:
    """周波数グループ化シミュレーションを作成"""
    return FrequencyGroupedManager(config)


def analyze_optimization_potential(frequencies: List[float]):
    """最適化効果の分析"""
    return FrequencyOptimizer.analyze_frequency_distribution(frequencies)


def get_recommended_frequencies(num_robots: int, num_frequency_groups: int = 4) -> List[float]:
    """推奨周波数を取得"""
    return FrequencyOptimizer.recommend_frequencies(num_robots, num_frequency_groups)


# デモ用の簡単な例
def demo_frequency_grouping():
    """周波数グループ化のデモ"""
    
    print("🎯 Frequency Grouping Demo")
    print("="*40)
    
    # 100台のロボットに4つの周波数を割り当て
    num_robots = 100
    frequencies = get_recommended_frequencies(num_robots, 4)
    assignments = FrequencyOptimizer.create_frequency_assignment(num_robots, frequencies)
    
    # 最適化効果を分析
    analysis = analyze_optimization_potential(assignments)
    
    print(f"Robots: {analysis['total_robots']}")
    print(f"Unique frequencies: {analysis['unique_frequencies']}")
    print(f"Process reduction: {analysis['process_reduction']} ({analysis['reduction_percent']:.1f}%)")
    print(f"Optimization effectiveness: {analysis['optimization_effectiveness']}")
    
    print(f"\nFrequency distribution:")
    for freq, count in sorted(analysis['frequency_distribution'].items()):
        print(f"   {freq:5.1f} Hz: {count:3d} robots")


if __name__ == "__main__":
    demo_frequency_grouping()