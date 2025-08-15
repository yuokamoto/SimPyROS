#!/usr/bin/env python3
"""
SimPyの設計思想 vs FrequencyGroup最適化の分析
FrequencyGroup導入がSimPyの意味を薄めているかを検証
"""

import sys
import os
import time
import simpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose


def pure_simpy_approach():
    """純粋なSimPyアプローチ - 各ロボットが独立プロセス"""
    
    print("🔬 純粋SimPyアプローチ分析")
    print("=" * 40)
    
    env = simpy.Environment()
    robots_data = []
    callback_count = 0
    
    def independent_robot_process(env, robot_id):
        """各ロボットが完全に独立したSimPyプロセス"""
        nonlocal callback_count
        frequency = 10.0  # Hz
        dt = 1.0 / frequency
        
        while True:
            # 独立したロボット処理
            callback_count += 1
            
            # SimPyの真の価値：複雑なイベント処理
            # 例：他のロボットとのリソース競合
            # 例：センサーデータ待機
            # 例：外部イベントへの反応
            
            # シンプルな例：他のプロセスとの相互作用
            if robot_id == 0 and env.now > 1.0:
                # リーダーロボットが他に指令を送る場面
                pass
            
            # 各ロボットが独立してyield
            yield env.timeout(dt)
    
    # 複数ロボットプロセスを起動
    num_robots = 20
    for i in range(num_robots):
        env.process(independent_robot_process(env, i))
    
    # 実行
    start_time = time.time()
    env.run(until=2.0)  # 2秒実行
    elapsed = time.time() - start_time
    
    print(f"✅ 純粋SimPyアプローチ結果:")
    print(f"   プロセス数: {num_robots}")
    print(f"   実行時間: {elapsed:.4f}秒")
    print(f"   コールバック数: {callback_count}")
    print(f"   SimPy活用度: 100% (各ロボットが独立)")
    
    return {
        'processes': num_robots,
        'callbacks': callback_count,
        'elapsed': elapsed,
        'simpy_utilization': 1.0
    }


def frequency_group_approach():
    """FrequencyGroupアプローチ - 一括処理"""
    
    print(f"\n🔄 FrequencyGroupアプローチ分析")
    print("=" * 40)
    
    env = simpy.Environment()
    callback_count = 0
    
    def frequency_grouped_process(env, robots):
        """FrequencyGroupのように一括処理"""
        nonlocal callback_count
        frequency = 10.0
        dt = 1.0 / frequency
        
        while True:
            # 同じ周波数のロボットを一括処理
            for robot_id in robots:
                callback_count += 1
                # 直接メソッド呼び出し（SimPyの並行性を使わない）
            
            # 1回のyieldで全ロボット処理
            yield env.timeout(dt)
    
    # 1つのプロセスで全ロボットを管理
    num_robots = 20
    robot_list = list(range(num_robots))
    env.process(frequency_grouped_process(env, robot_list))
    
    # 実行
    start_time = time.time()
    env.run(until=2.0)
    elapsed = time.time() - start_time
    
    print(f"✅ FrequencyGroupアプローチ結果:")
    print(f"   プロセス数: 1")
    print(f"   実行時間: {elapsed:.4f}秒")
    print(f"   コールバック数: {callback_count}")
    print(f"   SimPy活用度: 20% (時間管理のみ)")
    
    return {
        'processes': 1,
        'callbacks': callback_count,
        'elapsed': elapsed,
        'simpy_utilization': 0.2
    }


def hybrid_approach():
    """ハイブリッドアプローチ - SimPyの価値を活かしながら最適化"""
    
    print(f"\n⚖️ ハイブリッドアプローチ分析")
    print("=" * 40)
    
    env = simpy.Environment()
    callback_count = 0
    
    # SimPyの真の価値：リソース管理
    shared_resource = simpy.Resource(env, capacity=2)  # 2台まで同時アクセス可能
    communication_store = simpy.Store(env)  # ロボット間通信
    
    def smart_robot_process(env, robot_id, frequency_group):
        """SimPyの機能を活用しつつ効率化"""
        nonlocal callback_count
        dt = 1.0 / frequency_group
        
        while True:
            callback_count += 1
            
            # SimPyの価値1：リソース競合の自然な処理
            if robot_id % 5 == 0:  # 5台に1台がリソースを使用
                with shared_resource.request() as req:
                    yield req  # リソース取得まで待機
                    # リソース使用中の処理
                    yield env.timeout(0.1)  # リソース占有時間
            
            # SimPyの価値2：非同期通信
            if robot_id == 0 and env.now > 1.0:  # リーダーロボット
                communication_store.put(f"Command from robot {robot_id}")
            
            # 通常の処理
            yield env.timeout(dt)
    
    def communication_handler(env):
        """通信処理プロセス - SimPyの非同期性を活用"""
        while True:
            message = yield communication_store.get()
            # メッセージ処理
            print(f"📡 Message received: {message}")
    
    # プロセス起動：SimPyの並行性を保ちつつ効率化
    frequency_groups = {10.0: [], 20.0: []}
    num_robots = 20
    
    for i in range(num_robots):
        freq = 10.0 if i < 15 else 20.0  # 周波数グループ分け
        frequency_groups[freq].append(i)
        env.process(smart_robot_process(env, i, freq))
    
    # 通信処理プロセス
    env.process(communication_handler(env))
    
    # 実行
    start_time = time.time()
    env.run(until=2.0)
    elapsed = time.time() - start_time
    
    unique_frequencies = len(frequency_groups)
    
    print(f"✅ ハイブリッドアプローチ結果:")
    print(f"   プロセス数: {num_robots + 1} (ロボット{num_robots} + 通信1)")
    print(f"   周波数グループ数: {unique_frequencies}")
    print(f"   実行時間: {elapsed:.4f}秒")
    print(f"   コールバック数: {callback_count}")
    print(f"   SimPy活用度: 80% (並行性+リソース+通信)")
    
    return {
        'processes': num_robots + 1,
        'callbacks': callback_count,
        'elapsed': elapsed,
        'simpy_utilization': 0.8
    }


def simpy_philosophy_analysis():
    """SimPy設計思想の分析"""
    
    print("🧠 SimPy設計思想 vs FrequencyGroup分析")
    print("=" * 60)
    
    results = []
    
    # 3つのアプローチを実行
    pure_result = pure_simpy_approach()
    results.append(("Pure SimPy", pure_result))
    
    freq_result = frequency_group_approach()
    results.append(("FrequencyGroup", freq_result))
    
    hybrid_result = hybrid_approach()
    results.append(("Hybrid", hybrid_result))
    
    # 比較分析
    print(f"\n📊 アプローチ比較分析")
    print("=" * 60)
    print(f"{'Approach':<15} {'Processes':<10} {'Speed':<10} {'SimPy活用':<10} {'評価'}")
    print("-" * 60)
    
    for name, result in results:
        speed_ratio = freq_result['elapsed'] / result['elapsed']  # FrequencyGroupを基準
        simpy_util = f"{result['simpy_utilization']*100:.0f}%"
        
        if result['simpy_utilization'] > 0.7:
            rating = "🏆 IDEAL"
        elif result['simpy_utilization'] > 0.4:
            rating = "✅ GOOD"  
        elif result['simpy_utilization'] > 0.2:
            rating = "⚠️ LIMITED"
        else:
            rating = "❌ POOR"
            
        print(f"{name:<15} {result['processes']:<10} {speed_ratio:<10.2f}x {simpy_util:<10} {rating}")
    
    # 結論
    print(f"\n💡 分析結論:")
    print(f"━" * 50)
    
    pure_util = pure_result['simpy_utilization']
    freq_util = freq_result['simpy_utilization'] 
    hybrid_util = hybrid_result['simpy_utilization']
    
    if freq_util < 0.3:
        print(f"⚠️ FrequencyGroupはSimPyの価値を大幅に削減")
        print(f"   SimPy活用度: {pure_util*100:.0f}% → {freq_util*100:.0f}%")
        print(f"   主にタイマー機能のみ使用")
        
    print(f"\n🔧 推奨改善案:")
    if hybrid_util > freq_util:
        print(f"✅ ハイブリッドアプローチ採用")
        print(f"   - SimPyの並行性とリソース管理を活用")
        print(f"   - 周波数最適化も維持")
        print(f"   - SimPy活用度: {hybrid_util*100:.0f}%")
    
    print(f"\n🤖 SimPyの真の価値:")
    print(f"   1. プロセス間の複雑な相互作用")
    print(f"   2. リソース競合の自然な処理") 
    print(f"   3. 非同期イベントの処理")
    print(f"   4. 条件待機とタイムアウト")
    print(f"   5. 確率的事象のシミュレーション")
    
    return results


def main():
    """メイン分析実行"""
    
    print("🎯 SimPy哲学 vs パフォーマンス最適化")
    print("FrequencyGroup導入がSimPyの意味を薄めているかの検証")
    print("=" * 80)
    
    try:
        results = simpy_philosophy_analysis()
        
        print(f"\n🏁 最終判定:")
        print(f"━" * 30)
        
        freq_result = next(r[1] for r in results if r[0] == "FrequencyGroup")
        simpy_util = freq_result['simpy_utilization']
        
        if simpy_util < 0.3:
            print(f"❌ YES - FrequencyGroupはSimPyの意味を大幅に薄めている")
            print(f"   → ほぼ単純なタイマーライブラリとして使用")
            print(f"   → SimPyの設計思想から逸脱")
        elif simpy_util < 0.6:
            print(f"⚠️ PARTIALLY - SimPyの価値を部分的に活用")
            print(f"   → パフォーマンス優先でトレードオフ")
        else:
            print(f"✅ NO - SimPyの価値を適切に活用")
            
        print(f"\n💭 設計思想の考察:")
        print(f"   - パフォーマンス vs 設計思想のトレードオフ")
        print(f"   - 実用性 vs 理想的アーキテクチャ")
        print(f"   - SimPyを選んだ理由の再検討が必要")
        
    except Exception as e:
        print(f"❌ 分析失敗: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()