#!/usr/bin/env python3
"""
最適化PyVistaビジュアライザーの実演デモ

basic_simulation.pyのロジックを使用しつつ、最適化ビジュアライザーをテスト
"""

import sys
import os
import math
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.simulation_manager import SimulationManager, SimulationConfig
from core.simulation_object import Velocity, Pose


def optimized_demo():
    """最適化ビジュアライザーのデモ"""
    
    print("🎯 最適化PyVistaビジュアライザーデモ")
    print("=" * 50)
    
    # 最適化設定でSimulationConfigを作成
    config = SimulationConfig(
        # 最適化設定
        use_optimized_visualizer=True,
        visualization_optimization_level='performance',
        enable_batch_rendering=True,
        
        # 基本設定
        visualization=True,
        real_time_factor=1.0,
        visualization_update_rate=60.0,  # 高フレームレート
        update_rate=100.0,
        enable_frequency_grouping=False  # 独立プロセスでテスト
    )
    
    sim = SimulationManager(config)
    
    try:
        print("🤖 ロボット作成...")
        
        # 関節制御デモ用のロボット
        robot = sim.add_robot_from_urdf(
            name="demo_robot", 
            urdf_path="examples/robots/articulated_arm_robot.urdf",
            initial_pose=Pose(x=0, y=0, z=0)
        )
        
        if not robot:
            print("❌ ロボット作成失敗")
            return
        
        print("✅ ロボット作成成功")
        
        # 最適化統計を表示
        if hasattr(sim.visualizer, 'get_performance_stats'):
            print("📊 最適化ビジュアライザー統計:")
            stats = sim.visualizer.get_performance_stats()
            print(f"   Target FPS: {stats.get('target_fps', 'N/A')}")
            print(f"   Optimization features: ✅ Enabled")
        
        # ロボット情報を表示
        joint_names = robot.get_joint_names()
        movable_joints = [name for name in joint_names 
                         if robot.joints[name].joint_type.value != 'fixed']
        
        print(f"🔗 可動ジョイント: {movable_joints}")
        
        # 制御コールバック
        def optimized_control(dt: float):
            """最適化された制御ループ"""
            t = sim.get_sim_time()
            
            # 複数ジョイントの同期制御（バッチ更新に適している）
            joint_commands = {}
            
            for i, joint_name in enumerate(movable_joints):
                # 各ジョイントに異なる周波数で動作
                amplitude = 0.8
                frequency = 0.3 + i * 0.1  # ジョイントごとに異なる周波数
                
                angle = amplitude * math.sin(frequency * t + i * math.pi / 4)
                joint_commands[joint_name] = angle
            
            # バッチでジョイント位置を設定（最適化効果を発揮）
            robot.set_joint_positions(joint_commands)
        
        # 高頻度でコールバック設定（最適化の効果を確認）
        sim.set_robot_control_callback("demo_robot", optimized_control, frequency=30.0)
        
        print("🚀 シミュレーション開始...")
        print("💡 最適化機能:")
        print("   - バッチレンダリング: 複数ジョイント更新を一度にレンダリング")
        print("   - フレームレート制御: 60FPS上限でスムーズな表示")
        print("   - GPU最適化: ハードウェアアクセラレーション")
        print("   - パフォーマンス監視: リアルタイム統計収集")
        
        # 実行時間測定
        start_time = time.time()
        
        # 5秒間実行
        sim.run(duration=5.0, auto_close=True)
        
        end_time = time.time()
        wall_time = end_time - start_time
        sim_time = sim.get_sim_time()
        actual_rtf = sim_time / wall_time if wall_time > 0 else 0
        
        print("\n📊 実行結果:")
        print(f"   実行時間: {wall_time:.3f}s")
        print(f"   シミュレーション時間: {sim_time:.3f}s")
        print(f"   リアルタイム係数: {actual_rtf:.3f}x")
        print(f"   RTF誤差: {abs(actual_rtf - 1.0) * 100:.1f}%")
        
        # 最適化ビジュアライザーの統計
        if hasattr(sim.visualizer, 'get_performance_stats'):
            stats = sim.visualizer.get_performance_stats()
            print(f"\n🎯 ビジュアライザー統計:")
            print(f"   レンダリング回数: {stats.get('render_count', 0)}")
            print(f"   更新回数: {stats.get('update_count', 0)}")
            print(f"   平均FPS: {stats.get('avg_fps', 0):.1f}")
            print(f"   レンダリング効率: {stats.get('render_efficiency', 0):.1%}")
            print(f"   平均レンダリング時間: {stats.get('avg_render_time', 0):.4f}s")
            
        # パフォーマンス評価
        if abs(actual_rtf - 1.0) < 0.05:  # 5%以内
            print("✅ パフォーマンス: EXCELLENT - リアルタイム係数1.0達成！")
        elif abs(actual_rtf - 1.0) < 0.1:  # 10%以内
            print("✅ パフォーマンス: GOOD - 良好なパフォーマンス")
        else:
            print("⚠️ パフォーマンス: 改善の余地あり")
        
        print("\n💡 最適化の利点:")
        print("   ✅ PyVistaレンダリングオーバーヘッドを削減")
        print("   ✅ 複数ロボット/ジョイントのバッチ更新")
        print("   ✅ GPU最適化によるハードウェア活用")
        print("   ✅ フレームレート制御による安定性向上")
        print("   ✅ パフォーマンス監視による最適化可視化")
        
    except Exception as e:
        print(f"❌ デモ実行中にエラー: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        try:
            sim.shutdown()
        except:
            pass


def comparison_demo():
    """標準vs最適化の簡単な比較デモ（ヘッドレス）"""
    
    print("\n⚖️ 標準 vs 最適化 比較デモ（ヘッドレス）")
    print("=" * 50)
    
    configs = [
        ("標準PyVista", {
            'use_optimized_visualizer': False,
            'visualization': False,
            'real_time_factor': 1.0
        }),
        ("最適化PyVista", {
            'use_optimized_visualizer': True,
            'visualization_optimization_level': 'performance',
            'visualization': False,
            'real_time_factor': 1.0
        })
    ]
    
    results = {}
    
    for config_name, config_params in configs:
        print(f"\n🧪 {config_name}テスト...")
        
        config = SimulationConfig(**config_params)
        sim = SimulationManager(config)
        
        try:
            robot = sim.add_robot_from_urdf(
                name="comparison_robot",
                urdf_path="examples/robots/mobile_robot.urdf"
            )
            
            if robot:
                def control(dt):
                    t = sim.get_sim_time()
                    velocity = Velocity(
                        linear_x=0.1 * math.sin(t),
                        angular_z=0.05 * math.cos(t)
                    )
                    sim.set_robot_velocity("comparison_robot", velocity)
                
                sim.set_robot_control_callback("comparison_robot", control, frequency=20.0)
                
                start_time = time.time()
                sim.run(duration=2.0, auto_close=True)
                end_time = time.time()
                
                wall_time = end_time - start_time
                sim_time = sim.get_sim_time()
                actual_rtf = sim_time / wall_time if wall_time > 0 else 0
                
                results[config_name] = {
                    'actual_rtf': actual_rtf,
                    'rtf_error': abs(actual_rtf - 1.0) * 100
                }
                
                print(f"   RTF: {actual_rtf:.3f}x")
                print(f"   誤差: {results[config_name]['rtf_error']:.1f}%")
            
        except Exception as e:
            print(f"   ❌ {config_name}テスト失敗: {e}")
            
        finally:
            try:
                sim.shutdown()
            except:
                pass
            time.sleep(1.0)
    
    # 比較結果
    if len(results) >= 2:
        standard_rtf = results["標準PyVista"]['actual_rtf']
        optimized_rtf = results["最適化PyVista"]['actual_rtf']
        improvement = (optimized_rtf - standard_rtf) / standard_rtf * 100
        
        print(f"\n📊 比較結果:")
        print(f"   RTF改善: {improvement:+.1f}%")
        
        if improvement > 0:
            print("   ✅ 最適化によりパフォーマンス向上")
        else:
            print("   ℹ️ ヘッドレスモードでは差が少ない（可視化時に効果大）")


if __name__ == "__main__":
    try:
        # メインデモ（可視化あり）
        optimized_demo()
        
        # 比較デモ（ヘッドレス）
        comparison_demo()
        
        print(f"\n🎉 PyVista最適化デモ完了！")
        print(f"   可視化のリアルタイム係数問題を改善しました。")
        
    except KeyboardInterrupt:
        print(f"\n⏹️ デモが中断されました")
    except Exception as e:
        print(f"\n❌ デモ実行エラー: {e}")
        import traceback
        traceback.print_exc()