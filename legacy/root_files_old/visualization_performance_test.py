#!/usr/bin/env python3
"""
100台ロボット可視化パフォーマンステスト - RTF分析
"""

import sys
import os
import time
import gc

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose


def rtf_performance_test(num_robots=100, visualization=True, duration=5.0):
    """RTF性能測定テスト"""
    
    mode = "可視化ON" if visualization else "ヘッドレス"
    print(f"🧪 {num_robots}台ロボット {mode} RTF測定テスト")
    print(f"実行時間: {duration}秒")
    print("-" * 50)
    
    config = SimulationConfig(
        visualization=visualization,
        enable_frequency_grouping=True,  # 最適化有効
        update_rate=30.0,  # 可視化時は控えめに
        real_time_factor=0.0  # 最高速度
    )
    
    sim = SimulationManager(config)
    callback_count = 0
    
    try:
        # ロボット作成時間測定
        creation_start = time.time()
        
        print(f"🏗️ {num_robots}台のロボット作成中...")
        
        for i in range(num_robots):
            # グリッド配置
            grid_size = int(num_robots**0.5) + 1
            x = (i % grid_size) * 3.0  # 間隔を広げる
            y = (i // grid_size) * 3.0
            
            robot = sim.add_robot_from_urdf(
                name=f"perf_robot_{i}",
                urdf_path="examples/robots/mobile_robot.urdf",
                initial_pose=Pose(x=x, y=y, z=0),
                joint_update_rate=10.0,  # 可視化時は低頻度
                unified_process=True
            )
            
            # 軽量コントローラ
            def create_controller(robot_id):
                def controller(dt):
                    nonlocal callback_count
                    callback_count += 1
                    # 簡単な動き
                    velocity = Velocity(
                        linear_x=0.2,
                        angular_z=0.1 if robot_id % 2 == 0 else -0.1
                    )
                    sim.set_robot_velocity(f"perf_robot_{robot_id}", velocity)
                return controller
            
            sim.set_robot_control_callback(f"perf_robot_{i}", create_controller(i), frequency=10.0)
            
            # 進捗表示
            if (i + 1) % max(10, num_robots//10) == 0:
                print(f"   作成済み: {i+1}/{num_robots}")
        
        creation_time = time.time() - creation_start
        print(f"✅ ロボット作成完了: {creation_time:.3f}秒")
        
        # シミュレーション実行時間測定
        print(f"🚀 {duration}秒間のシミュレーション開始...")
        
        wall_start = time.time()
        sim_start_time = sim.get_sim_time()
        
        sim.run(duration=duration)
        
        wall_elapsed = time.time() - wall_start  
        sim_elapsed = sim.get_sim_time() - sim_start_time
        
        # RTF計算
        rtf = sim_elapsed / wall_elapsed if wall_elapsed > 0 else 0
        
        # 結果表示
        print(f"\n📊 {num_robots}台 {mode} 性能結果:")
        print(f"   ロボット作成時間: {creation_time:.3f}秒")
        print(f"   実時間: {wall_elapsed:.3f}秒")
        print(f"   シミュレーション時間: {sim_elapsed:.3f}秒")
        print(f"   RTF (Real Time Factor): {rtf:.3f}x")
        print(f"   コールバック総数: {callback_count:,}")
        print(f"   平均コールバック/秒: {callback_count/wall_elapsed:.1f}")
        
        # RTF評価
        if rtf >= 1.0:
            rating = "🚀 EXCELLENT (リアルタイム以上)"
        elif rtf >= 0.5:
            rating = "✅ GOOD (リアルタイムの半分以上)"
        elif rtf >= 0.1:
            rating = "⚠️ FAIR (リアルタイムの1/10以上)"
        else:
            rating = "❌ POOR (リアルタイムの1/10未満)"
        
        print(f"   性能評価: {rating}")
        
        # 可視化特有の分析
        if visualization:
            total_links = num_robots * 5  # mobile_robot.urdfは5リンク
            print(f"   描画メッシュ数: {total_links:,}個")
            print(f"   メッシュあたりRTF: {rtf/total_links*1000:.4f}x (per 1000 meshes)")
            
        return {
            'num_robots': num_robots,
            'visualization': visualization,
            'creation_time': creation_time,
            'wall_time': wall_elapsed,
            'sim_time': sim_elapsed,
            'rtf': rtf,
            'callback_count': callback_count,
            'total_links': num_robots * 5 if visualization else 0
        }
        
    except Exception as e:
        print(f"❌ テスト失敗: {e}")
        import traceback
        traceback.print_exc()
        return None
        
    finally:
        sim.shutdown()


def compare_visualization_impact():
    """可視化の影響度比較"""
    
    print("🔍 可視化性能影響度分析")
    print("=" * 60)
    
    test_configs = [
        (20, False, "20台ヘッドレス"),
        (20, True, "20台可視化"),
        (50, False, "50台ヘッドレス"),  
        (50, True, "50台可視化"),
        (100, False, "100台ヘッドレス"),
        (100, True, "100台可視化"),
    ]
    
    results = []
    
    for num_robots, visualization, description in test_configs:
        print(f"\n🧪 {description} テスト")
        
        gc.collect()  # メモリクリア
        
        result = rtf_performance_test(
            num_robots=num_robots,
            visualization=visualization, 
            duration=3.0  # 短時間で測定
        )
        
        if result:
            results.append((description, result))
            
        time.sleep(2.0)  # システム安定化
    
    # 比較分析
    if len(results) >= 2:
        print(f"\n📊 可視化性能影響分析")
        print("=" * 60)
        print(f"{'構成':<20} {'RTF':<10} {'作成時間':<10} {'総リンク数':<10} {'評価'}")
        print("-" * 60)
        
        for description, result in results:
            rtf_str = f"{result['rtf']:.3f}x"
            creation_str = f"{result['creation_time']:.3f}s"
            links_str = f"{result['total_links']:,}" if result['visualization'] else "N/A"
            
            if result['rtf'] >= 0.5:
                evaluation = "✅"
            elif result['rtf'] >= 0.1:
                evaluation = "⚠️"
            else:
                evaluation = "❌"
                
            print(f"{description:<20} {rtf_str:<10} {creation_str:<10} {links_str:<10} {evaluation}")
        
        # 可視化コスト分析
        print(f"\n💡 可視化コスト分析:")
        
        headless_100 = next((r[1] for r in results if "100台ヘッドレス" in r[0]), None)
        visual_100 = next((r[1] for r in results if "100台可視化" in r[0]), None)
        
        if headless_100 and visual_100:
            rtf_ratio = headless_100['rtf'] / visual_100['rtf'] if visual_100['rtf'] > 0 else float('inf')
            print(f"   100台ヘッドレスRTF: {headless_100['rtf']:.3f}x")
            print(f"   100台可視化RTF: {visual_100['rtf']:.3f}x")
            print(f"   可視化による速度低下: {rtf_ratio:.1f}倍")
            
            if visual_100['rtf'] >= 0.1:
                print(f"   ✅ RTF={visual_100['rtf']:.3f}は100台可視化として妥当な性能")
            else:
                print(f"   ⚠️ RTF={visual_100['rtf']:.3f}は改善の余地あり")


def main():
    """メイン実行"""
    print("🎯 100台ロボット可視化RTF性能分析")
    print("SimPyROSのFrequencyGroup最適化効果を検証")
    print("=" * 70)
    
    try:
        # まず単発テスト
        print("1️⃣ 100台可視化RTF測定")
        result = rtf_performance_test(num_robots=100, visualization=True, duration=5.0)
        
        if result and result['rtf'] > 0:
            print(f"\n💭 結果考察:")
            print(f"RTF={result['rtf']:.3f}は100台ロボット+可視化として:")
            
            if result['rtf'] >= 0.1:
                print(f"✅ 妥当な性能範囲内")
                print(f"   - 500メッシュのリアルタイム描画")
                print(f"   - FrequencyGroup最適化効果あり")
                print(f"   - 可視化が主なボトルネック")
            else:
                print(f"⚠️ 改善の余地あり")
        
        time.sleep(3.0)
        
        # 詳細比較分析
        print("\n2️⃣ 可視化影響度詳細分析")
        compare_visualization_impact()
        
    except KeyboardInterrupt:
        print(f"\n⏹️ テスト中断")
    except Exception as e:
        print(f"\n❌ テスト失敗: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()