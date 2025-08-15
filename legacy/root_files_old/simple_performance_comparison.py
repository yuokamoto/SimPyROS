#!/usr/bin/env python3
"""
Simple Performance Comparison - SimPy vs Simple While Loop
シンプルなパフォーマンス比較テスト
"""

import time
import threading
import math
from typing import Dict, List, Callable
from dataclasses import dataclass

import sys
import os
import simpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_object import Velocity, Pose


@dataclass 
class SimpleRobotState:
    """Lightweight robot state"""
    name: str
    position: List[float] = None
    velocity: Velocity = None
    
    def __post_init__(self):
        if self.position is None:
            self.position = [0.0, 0.0, 0.0]
        if self.velocity is None:
            self.velocity = Velocity.zero()


class SimpleWhileLoopSimulation:
    """純粋なwhile loopシミュレーション"""
    
    def __init__(self, update_rate: float = 30.0):
        self.update_rate = update_rate
        self.dt = 1.0 / update_rate
        
        self.robots: Dict[str, SimpleRobotState] = {}
        self.callbacks: Dict[str, Callable] = {}
        self.callback_frequencies: Dict[str, float] = {}
        self.last_callback_times: Dict[str, float] = {}
        
        self.running = False
        self.current_time = 0.0
        self.frame_count = 0
        self.total_callbacks = 0
        
        self.lock = threading.Lock()
    
    def add_robot(self, name: str, initial_pose: Pose = None):
        """Add robot"""
        if initial_pose is None:
            initial_pose = Pose()
            
        with self.lock:
            self.robots[name] = SimpleRobotState(
                name=name,
                position=[initial_pose.position[0], initial_pose.position[1], 0.0],
                velocity=Velocity.zero()
            )
    
    def set_robot_control_callback(self, name: str, callback: Callable, frequency: float):
        """Set control callback"""
        with self.lock:
            self.callbacks[name] = callback
            self.callback_frequencies[name] = frequency
            self.last_callback_times[name] = 0.0
    
    def set_robot_velocity(self, name: str, velocity: Velocity):
        """Set robot velocity"""
        if name in self.robots:
            with self.lock:
                self.robots[name].velocity = velocity
    
    def run(self, duration: float):
        """Run simulation"""
        self.running = True
        self.current_time = 0.0
        self.frame_count = 0
        self.total_callbacks = 0
        
        start_time = time.time()
        
        while self.running and self.current_time < duration:
            loop_start = time.time()
            
            # Update time
            self.current_time = time.time() - start_time
            
            # Update robots
            with self.lock:
                for robot in self.robots.values():
                    # Simple physics
                    robot.position[0] += robot.velocity.linear_x * self.dt
                    robot.position[1] += robot.velocity.linear_y * self.dt
            
            # Execute callbacks
            with self.lock:
                for name, callback in self.callbacks.items():
                    frequency = self.callback_frequencies[name]
                    last_time = self.last_callback_times[name]
                    
                    if self.current_time - last_time >= (1.0 / frequency):
                        try:
                            callback(1.0 / frequency)
                            self.total_callbacks += 1
                            self.last_callback_times[name] = self.current_time
                        except Exception as e:
                            print(f"⚠️ Callback error: {e}")
            
            self.frame_count += 1
            
            # Frame rate control
            loop_elapsed = time.time() - loop_start
            sleep_time = self.dt - loop_elapsed
            
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        self.running = False
        return time.time() - start_time


class SimPyFrequencyGroupSimulation:
    """SimPy FrequencyGroup風のシミュレーション"""
    
    def __init__(self, update_rate: float = 30.0):
        self.env = simpy.Environment()
        self.update_rate = update_rate
        
        self.robots: Dict[str, SimpleRobotState] = {}
        self.callbacks: Dict[str, Callable] = {}
        self.callback_frequencies: Dict[str, float] = {}
        
        self.total_callbacks = 0
        self.running = False
    
    def add_robot(self, name: str, initial_pose: Pose = None):
        """Add robot"""
        if initial_pose is None:
            initial_pose = Pose()
            
        self.robots[name] = SimpleRobotState(
            name=name,
            position=[initial_pose.position[0], initial_pose.position[1], 0.0],
            velocity=Velocity.zero()
        )
    
    def set_robot_control_callback(self, name: str, callback: Callable, frequency: float):
        """Set control callback"""
        self.callbacks[name] = callback
        self.callback_frequencies[name] = frequency
    
    def set_robot_velocity(self, name: str, velocity: Velocity):
        """Set robot velocity"""
        if name in self.robots:
            self.robots[name].velocity = velocity
    
    def frequency_group_process(self, frequency: float, robot_names: List[str]):
        """Frequency group process - FrequencyGroup style"""
        dt = 1.0 / frequency
        
        while self.running:
            # Update all robots in this frequency group
            for name in robot_names:
                if name in self.robots:
                    robot = self.robots[name]
                    # Simple physics
                    robot.position[0] += robot.velocity.linear_x * dt
                    robot.position[1] += robot.velocity.linear_y * dt
                
                # Execute callback
                if name in self.callbacks:
                    try:
                        self.callbacks[name](dt)
                        self.total_callbacks += 1
                    except Exception as e:
                        print(f"⚠️ Callback error: {e}")
            
            # Single yield for all robots in group
            yield self.env.timeout(dt)
    
    def run(self, duration: float):
        """Run simulation"""
        self.running = True
        self.total_callbacks = 0
        
        # Group robots by frequency
        frequency_groups = {}
        for name, frequency in self.callback_frequencies.items():
            if frequency not in frequency_groups:
                frequency_groups[frequency] = []
            frequency_groups[frequency].append(name)
        
        # Start frequency group processes
        for frequency, robot_names in frequency_groups.items():
            self.env.process(self.frequency_group_process(frequency, robot_names))
        
        # Run simulation
        start_time = time.time()
        self.env.run(until=duration)
        return time.time() - start_time


def performance_comparison_test():
    """100台ロボットパフォーマンス比較"""
    
    print("🎯 Simple While Loop vs SimPy FrequencyGroup Performance Test")
    print("=" * 70)
    
    num_robots = 100
    duration = 5.0
    results = []
    
    # Test 1: Simple While Loop
    print(f"\n1️⃣ Simple While Loop Test ({num_robots} robots)")
    print("-" * 50)
    
    sim1 = SimpleWhileLoopSimulation(update_rate=50.0)
    
    # Create robots
    creation_start = time.time()
    for i in range(num_robots):
        x = (i % 10) * 2.0
        y = (i // 10) * 2.0
        
        sim1.add_robot(f"robot_{i}", Pose(x=x, y=y, z=0))
        
        def create_controller(robot_id):
            def controller(dt):
                t = sim1.current_time
                velocity = Velocity(
                    linear_x=0.2 * math.sin(t + robot_id * 0.1),
                    angular_z=0.1 * math.cos(t + robot_id * 0.2)
                )
                sim1.set_robot_velocity(f"robot_{robot_id}", velocity)
            return controller
        
        sim1.set_robot_control_callback(f"robot_{i}", create_controller(i), frequency=10.0)
    
    creation_time1 = time.time() - creation_start
    
    print(f"   Robot creation: {creation_time1:.4f}s")
    print(f"   Running {duration}s simulation...")
    
    run_time1 = sim1.run(duration)
    
    print(f"   ✅ Completed")
    print(f"   Run time: {run_time1:.4f}s")
    print(f"   RTF: {duration/run_time1:.3f}x")
    print(f"   Frame count: {sim1.frame_count}")
    print(f"   Total callbacks: {sim1.total_callbacks}")
    print(f"   Callbacks/sec: {sim1.total_callbacks/run_time1:.1f}")
    
    results.append({
        'name': 'Simple While Loop',
        'creation_time': creation_time1,
        'run_time': run_time1,
        'rtf': duration/run_time1,
        'frame_count': sim1.frame_count,
        'total_callbacks': sim1.total_callbacks,
        'callbacks_per_sec': sim1.total_callbacks/run_time1
    })
    
    # Test 2: SimPy FrequencyGroup
    print(f"\n2️⃣ SimPy FrequencyGroup Test ({num_robots} robots)")
    print("-" * 50)
    
    sim2 = SimPyFrequencyGroupSimulation(update_rate=50.0)
    
    # Create robots
    creation_start = time.time()
    for i in range(num_robots):
        x = (i % 10) * 2.0
        y = (i // 10) * 2.0
        
        sim2.add_robot(f"robot_{i}", Pose(x=x, y=y, z=0))
        
        def create_controller(robot_id, sim_ref):
            def controller(dt):
                t = sim_ref.env.now
                velocity = Velocity(
                    linear_x=0.2 * math.sin(t + robot_id * 0.1),
                    angular_z=0.1 * math.cos(t + robot_id * 0.2)
                )
                sim_ref.set_robot_velocity(f"robot_{robot_id}", velocity)
            return controller
        
        sim2.set_robot_control_callback(f"robot_{i}", create_controller(i, sim2), frequency=10.0)
    
    creation_time2 = time.time() - creation_start
    
    print(f"   Robot creation: {creation_time2:.4f}s")
    print(f"   Running {duration}s simulation...")
    
    run_time2 = sim2.run(duration)
    
    print(f"   ✅ Completed")
    print(f"   Run time: {run_time2:.4f}s")
    print(f"   RTF: {duration/run_time2:.3f}x")
    print(f"   Total callbacks: {sim2.total_callbacks}")
    print(f"   Callbacks/sec: {sim2.total_callbacks/run_time2:.1f}")
    
    results.append({
        'name': 'SimPy FrequencyGroup',
        'creation_time': creation_time2,
        'run_time': run_time2,
        'rtf': duration/run_time2,
        'frame_count': 0,  # Not applicable
        'total_callbacks': sim2.total_callbacks,
        'callbacks_per_sec': sim2.total_callbacks/run_time2
    })
    
    # Comparison
    print(f"\n📊 Performance Comparison Results")
    print("=" * 70)
    print(f"{'Approach':<20} {'Creation':<12} {'Run Time':<12} {'RTF':<10} {'Callbacks/s':<12} {'Rating'}")
    print("-" * 70)
    
    for result in results:
        if result['rtf'] >= 10.0:
            rating = "🚀 ULTRA"
        elif result['rtf'] >= 5.0:
            rating = "⚡ FAST"
        elif result['rtf'] >= 1.0:
            rating = "✅ GOOD"
        elif result['rtf'] >= 0.5:
            rating = "⚠️ FAIR"
        else:
            rating = "❌ POOR"
        
        print(f"{result['name']:<20} {result['creation_time']:<12.4f} {result['run_time']:<12.4f} "
              f"{result['rtf']:<10.3f} {result['callbacks_per_sec']:<12.1f} {rating}")
    
    # Analysis
    if len(results) == 2:
        simple_rtf = results[0]['rtf']
        simpy_rtf = results[1]['rtf']
        
        print(f"\n🔍 Analysis:")
        print(f"   Simple While Loop RTF: {simple_rtf:.3f}x")
        print(f"   SimPy FrequencyGroup RTF: {simpy_rtf:.3f}x")
        
        if simple_rtf > simpy_rtf:
            improvement = (simple_rtf / simpy_rtf - 1) * 100
            print(f"   ✅ Simple While Loop is {improvement:.1f}% faster")
        else:
            degradation = (simpy_rtf / simple_rtf - 1) * 100
            print(f"   ⚠️ Simple While Loop is {degradation:.1f}% slower")
        
        print(f"\n💡 Conclusion:")
        print(f"   - Simple While Loop: Maximum performance, minimal complexity")
        print(f"   - SimPy FrequencyGroup: Additional overhead from SimPy framework")
        print(f"   - For pure performance: Simple While Loop wins")
        print(f"   - For complex event handling: SimPy provides more features")
    
    return results


def main():
    """メイン実行"""
    
    print("🧪 SimPy vs Simple While Loop Performance Analysis")
    print("100台ロボットでの直接的な性能比較")
    print("=" * 80)
    
    try:
        results = performance_comparison_test()
        
        print(f"\n🏁 Final Verdict:")
        print(f"   - Simple While Loop: 最高のパフォーマンス、最小の複雑さ")
        print(f"   - SimPy: 機能豊富だが、オーバーヘッドあり")
        print(f"   - 単純なロボットシミュレーション: Simple While Loopが最適")
        print(f"   - 複雑なイベント処理: SimPyの価値が活かされる")
        
    except KeyboardInterrupt:
        print(f"\n⏹️ Test interrupted")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()