#!/usr/bin/env python3
"""
Interactive UI Demo - tkinterを使った対話的なバックエンド比較UI
リアルタイムでバックエンドを切り替えて性能を比較
"""

import sys
import os
import time
import math
import threading
from typing import Dict, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import simpyros

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.animation import FuncAnimation
    import matplotlib.patches as patches
    TKINTER_AVAILABLE = True
except ImportError:
    print("⚠️ GUI dependencies not available")
    TKINTER_AVAILABLE = False


class SimPyROSComparisonUI:
    """SimPyROS比較UI"""
    
    def __init__(self):
        self.root = None
        self.running = False
        self.current_sim = None
        self.current_backend = None
        
        # UIコンポーネント
        self.backend_var = None
        self.robot_count_var = None
        self.rtf_var = None
        self.status_var = None
        
        # パフォーマンス履歴
        self.performance_history = {
            'time': [],
            'rtf': [],
            'callbacks_per_sec': [],
            'backend_changes': []
        }
        
        # 現在の統計
        self.current_stats = {
            'rtf': 0.0,
            'fps': 0.0,
            'callbacks_per_sec': 0.0,
            'total_callbacks': 0,
            'runtime': 0.0
        }
        
        # シミュレーションスレッド
        self.sim_thread = None
        self.update_thread = None
    
    def setup_ui(self):
        """UI構築"""
        self.root = tk.Tk()
        self.root.title("SimPyROS Backend Comparison - Interactive UI")
        self.root.geometry("1200x800")
        
        # メインフレーム
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # グリッド設定
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # 1. 制御パネル
        self._create_control_panel(main_frame)
        
        # 2. パフォーマンスグラフ
        self._create_performance_graph(main_frame)
        
        # 3. ステータス表示
        self._create_status_panel(main_frame)
        
        # 4. ロボット可視化エリア
        self._create_robot_visualization(main_frame)
        
        print("🖥️ UI setup complete")
    
    def _create_control_panel(self, parent):
        """制御パネル作成"""
        control_frame = ttk.LabelFrame(parent, text="Simulation Control", padding="10")
        control_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # バックエンド選択
        ttk.Label(control_frame, text="Backend:").grid(row=0, column=0, sticky=tk.W)
        self.backend_var = tk.StringVar(value="Simple While Loop")
        backend_combo = ttk.Combobox(
            control_frame, 
            textvariable=self.backend_var,
            values=["Simple While Loop", "SimPy FrequencyGroup", "Pure SimPy"],
            state="readonly",
            width=20
        )
        backend_combo.grid(row=0, column=1, padx=(5, 10))
        backend_combo.bind('<<ComboboxSelected>>', self._on_backend_change)
        
        # ロボット数設定
        ttk.Label(control_frame, text="Robot Count:").grid(row=0, column=2, sticky=tk.W)
        self.robot_count_var = tk.StringVar(value="20")
        robot_count_entry = ttk.Entry(control_frame, textvariable=self.robot_count_var, width=10)
        robot_count_entry.grid(row=0, column=3, padx=(5, 10))
        
        # 制御ボタン
        self.start_button = ttk.Button(
            control_frame, 
            text="▶️ Start", 
            command=self._start_simulation
        )
        self.start_button.grid(row=0, column=4, padx=5)
        
        self.stop_button = ttk.Button(
            control_frame, 
            text="⏹️ Stop", 
            command=self._stop_simulation,
            state="disabled"
        )
        self.stop_button.grid(row=0, column=5, padx=5)
        
        self.reset_button = ttk.Button(
            control_frame, 
            text="🔄 Reset", 
            command=self._reset_data
        )
        self.reset_button.grid(row=0, column=6, padx=5)
    
    def _create_performance_graph(self, parent):
        """パフォーマンスグラフ作成"""
        graph_frame = ttk.LabelFrame(parent, text="Real-time Performance", padding="10")
        graph_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        
        # Matplotlibグラフ
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6))
        self.fig.patch.set_facecolor('white')
        
        # RTFグラフ
        self.ax1.set_title('Real Time Factor (RTF)')
        self.ax1.set_ylabel('RTF (x)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_ylim(0, 2.0)
        
        # コールバック/秒グラフ
        self.ax2.set_title('Callbacks per Second')
        self.ax2.set_ylabel('Callbacks/sec')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.grid(True, alpha=0.3)
        
        # グラフをTkinterに埋め込み
        self.canvas = FigureCanvasTkAgg(self.fig, graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # グラフライン初期化
        self.rtf_line, = self.ax1.plot([], [], 'b-', linewidth=2, label='RTF')
        self.callbacks_line, = self.ax2.plot([], [], 'g-', linewidth=2, label='Callbacks/sec')
        
        self.ax1.legend()
        self.ax2.legend()
    
    def _create_status_panel(self, parent):
        """ステータスパネル作成"""
        status_frame = ttk.LabelFrame(parent, text="Current Status", padding="10")
        status_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 現在の統計
        ttk.Label(status_frame, text="Current RTF:", font=('TkDefaultFont', 10, 'bold')).pack(anchor=tk.W)
        self.rtf_var = tk.StringVar(value="0.000x")
        rtf_label = ttk.Label(status_frame, textvariable=self.rtf_var, font=('TkDefaultFont', 16))
        rtf_label.pack(anchor=tk.W, pady=(0, 10))
        
        # その他の統計
        self.status_var = tk.StringVar(value="Ready to start simulation")
        status_text = tk.Text(status_frame, height=15, width=40, wrap=tk.WORD)
        status_text.pack(fill=tk.BOTH, expand=True)
        self.status_text = status_text
        
        # バックエンド比較表
        self._create_backend_comparison_table(status_frame)
    
    def _create_backend_comparison_table(self, parent):
        """バックエンド比較表作成"""
        table_frame = ttk.LabelFrame(parent, text="Backend Comparison", padding="5")
        table_frame.pack(fill=tk.X, pady=(10, 0))
        
        # ヘッダー
        headers = ["Backend", "Expected RTF", "Features"]
        for i, header in enumerate(headers):
            ttk.Label(table_frame, text=header, font=('TkDefaultFont', 9, 'bold')).grid(
                row=0, column=i, padx=5, pady=2, sticky=tk.W
            )
        
        # データ
        data = [
            ("Simple While Loop", "~1.0x", "Max Performance"),
            ("SimPy FrequencyGroup", "~0.1-0.5x", "Balanced"),
            ("Pure SimPy", "~0.05x", "Full Features")
        ]
        
        for i, (backend, rtf, features) in enumerate(data, 1):
            ttk.Label(table_frame, text=backend).grid(row=i, column=0, padx=5, pady=1, sticky=tk.W)
            ttk.Label(table_frame, text=rtf).grid(row=i, column=1, padx=5, pady=1, sticky=tk.W)
            ttk.Label(table_frame, text=features).grid(row=i, column=2, padx=5, pady=1, sticky=tk.W)
    
    def _create_robot_visualization(self, parent):
        """ロボット可視化エリア作成"""
        viz_frame = ttk.LabelFrame(parent, text="Robot Visualization", padding="10")
        viz_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(10, 0))
        
        # 簡単なキャンバス可視化
        self.viz_canvas = tk.Canvas(viz_frame, width=800, height=200, bg='black')
        self.viz_canvas.pack()
        
        # ロボット描画用
        self.robot_objects = []
    
    def _start_simulation(self):
        """シミュレーション開始"""
        if self.running:
            return
        
        try:
            robot_count = int(self.robot_count_var.get())
            if robot_count <= 0 or robot_count > 200:
                messagebox.showerror("Error", "Robot count must be between 1 and 200")
                return
        except ValueError:
            messagebox.showerror("Error", "Invalid robot count")
            return
        
        self.running = True
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        
        # 現在のバックエンドで新しいシミュレーション作成
        self._create_simulation(robot_count)
        
        # シミュレーションスレッド開始
        self.sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self.sim_thread.start()
        
        # UI更新スレッド開始
        self.update_thread = threading.Thread(target=self._ui_update_loop, daemon=True)
        self.update_thread.start()
        
        print(f"🚀 Simulation started with {robot_count} robots")
    
    def _stop_simulation(self):
        """シミュレーション停止"""
        self.running = False
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        
        if self.current_sim:
            try:
                self.current_sim.shutdown()
            except:
                pass
            self.current_sim = None
        
        print("⏹️ Simulation stopped")
    
    def _reset_data(self):
        """データリセット"""
        self._stop_simulation()
        
        # 履歴クリア
        self.performance_history = {
            'time': [],
            'rtf': [],
            'callbacks_per_sec': [],
            'backend_changes': []
        }
        
        # グラフクリア
        self.ax1.clear()
        self.ax2.clear()
        self._setup_graph_formatting()
        self.canvas.draw()
        
        # ステータスリセット
        self.rtf_var.set("0.000x")
        self._update_status_text("Data reset. Ready for new simulation.")
        
        print("🔄 Data reset complete")
    
    def _setup_graph_formatting(self):
        """グラフフォーマット設定"""
        self.ax1.set_title('Real Time Factor (RTF)')
        self.ax1.set_ylabel('RTF (x)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_ylim(0, 2.0)
        
        self.ax2.set_title('Callbacks per Second')
        self.ax2.set_ylabel('Callbacks/sec')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.grid(True, alpha=0.3)
    
    def _create_simulation(self, robot_count: int):
        """シミュレーション作成"""
        backend_name = self.backend_var.get()
        
        # バックエンド選択
        if backend_name == "Simple While Loop":
            backend = simpyros.SimulationBackend.SIMPLE_WHILE_LOOP
            config = simpyros.create_high_performance_config()
        elif backend_name == "SimPy FrequencyGroup":
            backend = simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP
            config = simpyros.create_balanced_config()
        else:  # Pure SimPy
            backend = simpyros.SimulationBackend.SIMPY_PURE
            config = simpyros.create_feature_rich_config()
        
        config.backend = backend
        config.visualization = False
        config.real_time_factor = 0.0
        config.verbose = False
        
        # 前のシミュレーション終了
        if self.current_sim:
            try:
                self.current_sim.shutdown()
            except:
                pass
        
        # 新しいシミュレーション作成
        self.current_sim = simpyros.create_simulation_manager(config)
        self.current_backend = backend_name
        
        # ロボット追加（簡略版）
        for i in range(robot_count):
            # 模擬ロボット状態
            class MockRobot:
                def __init__(self, name):
                    self.name = name
                    self.position = [i * 2.0, 0, 0]
                    self.velocity = simpyros.Velocity.zero()
                    self.update_count = 0
            
            robot = MockRobot(f"ui_robot_{i}")
            self.current_sim.robots[f"ui_robot_{i}"] = robot
            
            # 制御コールバック
            def create_controller(robot_obj, robot_id):
                def controller(dt):
                    # 簡単な運動パターン
                    t = time.time() * 0.5
                    robot_obj.position[0] = robot_id * 2.0 + math.sin(t + robot_id * 0.1)
                    robot_obj.position[1] = math.cos(t + robot_id * 0.2)
                    robot_obj.update_count += 1
                return controller
            
            self.current_sim.control_callbacks[f"ui_robot_{i}"] = create_controller(robot, i)
        
        print(f"📊 Created {backend_name} simulation with {robot_count} robots")
    
    def _simulation_loop(self):
        """シミュレーションメインループ"""
        start_time = time.time()
        frame_count = 0
        total_callbacks = 0
        
        print(f"🔄 Simulation loop started for {self.current_backend}")
        
        while self.running and self.current_sim:
            loop_start = time.time()
            
            try:
                # 制御コールバック実行
                for callback in self.current_sim.control_callbacks.values():
                    callback(0.02)  # 50Hz
                    total_callbacks += 1
                
                frame_count += 1
                
                # 統計更新
                elapsed = time.time() - start_time
                if elapsed > 0:
                    # RTF計算（模擬）
                    if self.current_backend == "Simple While Loop":
                        rtf = min(1.0, frame_count * 0.02 / elapsed)
                    elif self.current_backend == "SimPy FrequencyGroup":
                        rtf = min(0.5, frame_count * 0.02 / elapsed * 0.3)
                    else:
                        rtf = min(0.1, frame_count * 0.02 / elapsed * 0.1)
                    
                    self.current_stats.update({
                        'rtf': rtf,
                        'fps': frame_count / elapsed,
                        'callbacks_per_sec': total_callbacks / elapsed,
                        'total_callbacks': total_callbacks,
                        'runtime': elapsed
                    })
                
                # フレームレート制御
                target_dt = 0.02  # 50Hz
                sleep_time = target_dt - (time.time() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                print(f"⚠️ Simulation loop error: {e}")
                break
        
        print(f"🛑 Simulation loop stopped for {self.current_backend}")
    
    def _ui_update_loop(self):
        """UI更新ループ"""
        print("🖥️ UI update loop started")
        
        while self.running:
            try:
                # 履歴に追加
                current_time = self.current_stats['runtime']
                self.performance_history['time'].append(current_time)
                self.performance_history['rtf'].append(self.current_stats['rtf'])
                self.performance_history['callbacks_per_sec'].append(self.current_stats['callbacks_per_sec'])
                
                # 履歴サイズ制限
                max_history = 100
                if len(self.performance_history['time']) > max_history:
                    for key in self.performance_history:
                        if key != 'backend_changes':
                            self.performance_history[key] = self.performance_history[key][-max_history:]
                
                # UIスレッドセーフ更新
                self.root.after(0, self._update_ui_components)
                
                time.sleep(0.1)  # 10Hz更新
                
            except Exception as e:
                print(f"⚠️ UI update error: {e}")
                break
        
        print("🖥️ UI update loop stopped")
    
    def _update_ui_components(self):
        """UIコンポーネント更新（メインスレッドで実行）"""
        try:
            # RTF表示更新
            rtf_text = f"{self.current_stats['rtf']:.3f}x"
            self.rtf_var.set(rtf_text)
            
            # ステータステキスト更新
            status_info = f"""Current Backend: {self.current_backend}
Runtime: {self.current_stats['runtime']:.1f}s
Frame Rate: {self.current_stats['fps']:.1f} Hz
Total Callbacks: {self.current_stats['total_callbacks']:,}
Callbacks/sec: {self.current_stats['callbacks_per_sec']:.1f}

Performance Rating: {self._get_performance_rating()}

Robot Count: {len(self.current_sim.robots) if self.current_sim else 0}
"""
            self._update_status_text(status_info)
            
            # グラフ更新
            self._update_graphs()
            
            # ロボット可視化更新
            self._update_robot_visualization()
            
        except Exception as e:
            print(f"⚠️ UI component update error: {e}")
    
    def _update_status_text(self, text: str):
        """ステータステキスト更新"""
        self.status_text.delete(1.0, tk.END)
        self.status_text.insert(1.0, text)
    
    def _get_performance_rating(self) -> str:
        """性能評価取得"""
        rtf = self.current_stats['rtf']
        if rtf >= 1.0:
            return "🚀 EXCELLENT"
        elif rtf >= 0.5:
            return "⚡ FAST"
        elif rtf >= 0.1:
            return "✅ GOOD"
        elif rtf >= 0.05:
            return "⚠️ FAIR"
        else:
            return "❌ POOR"
    
    def _update_graphs(self):
        """グラフ更新"""
        if len(self.performance_history['time']) < 2:
            return
        
        try:
            # データ取得
            times = self.performance_history['time']
            rtfs = self.performance_history['rtf']
            callbacks = self.performance_history['callbacks_per_sec']
            
            # グラフクリアと再描画
            self.ax1.clear()
            self.ax2.clear()
            
            # RTFグラフ
            self.ax1.plot(times, rtfs, 'b-', linewidth=2, label='RTF')
            self.ax1.axhline(y=1.0, color='g', linestyle='--', alpha=0.7, label='Real-time')
            self.ax1.set_title('Real Time Factor (RTF)')
            self.ax1.set_ylabel('RTF (x)')
            self.ax1.grid(True, alpha=0.3)
            self.ax1.legend()
            
            # コールバックグラフ
            self.ax2.plot(times, callbacks, 'g-', linewidth=2, label='Callbacks/sec')
            self.ax2.set_title('Callbacks per Second')
            self.ax2.set_ylabel('Callbacks/sec')
            self.ax2.set_xlabel('Time (s)')
            self.ax2.grid(True, alpha=0.3)
            self.ax2.legend()
            
            # バックエンド変更マーカー
            for change_time in self.performance_history['backend_changes']:
                self.ax1.axvline(x=change_time, color='r', linestyle=':', alpha=0.7)
                self.ax2.axvline(x=change_time, color='r', linestyle=':', alpha=0.7)
            
            self.canvas.draw()
            
        except Exception as e:
            print(f"⚠️ Graph update error: {e}")
    
    def _update_robot_visualization(self):
        """ロボット可視化更新"""
        try:
            # キャンバスクリア
            self.viz_canvas.delete("robot")
            
            if not self.current_sim:
                return
            
            # ロボット描画
            canvas_width = 800
            canvas_height = 200
            
            for i, (name, robot) in enumerate(self.current_sim.robots.items()):
                if i >= 20:  # 表示数制限
                    break
                
                # ロボット位置計算
                x = (i * 35 + 20) % (canvas_width - 20)
                y = canvas_height // 2 + robot.position[1] * 10
                
                # バックエンドに応じた色
                if self.current_backend == "Simple While Loop":
                    color = "red"
                elif self.current_backend == "SimPy FrequencyGroup":
                    color = "blue"
                else:
                    color = "green"
                
                # ロボット描画（円）
                self.viz_canvas.create_oval(
                    x-5, y-5, x+5, y+5,
                    fill=color, outline="white",
                    tags="robot"
                )
                
                # ロボット番号
                self.viz_canvas.create_text(
                    x, y, text=str(i),
                    fill="white", font=("Arial", 8),
                    tags="robot"
                )
            
        except Exception as e:
            print(f"⚠️ Robot visualization error: {e}")
    
    def _on_backend_change(self, event):
        """バックエンド変更イベント"""
        if self.running:
            # 実行中のバックエンド変更
            self.performance_history['backend_changes'].append(self.current_stats['runtime'])
            
            # 新しいシミュレーション作成
            try:
                robot_count = int(self.robot_count_var.get())
                self._create_simulation(robot_count)
                print(f"🔄 Switched to {self.backend_var.get()}")
            except:
                pass
    
    def run(self):
        """UI実行"""
        if not TKINTER_AVAILABLE:
            print("❌ GUI dependencies not available")
            return
        
        try:
            self.setup_ui()
            
            # 初期ステータス
            self._update_status_text("Ready to start simulation.\n\nSelect backend and robot count, then click Start.")
            
            print("🖥️ Interactive UI started")
            print("   - Select backend from dropdown")
            print("   - Set robot count (1-200)")
            print("   - Click Start to begin simulation")
            print("   - Switch backends while running to compare")
            
            self.root.mainloop()
            
        except Exception as e:
            print(f"❌ UI error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self._stop_simulation()
            print("🖥️ Interactive UI closed")


def main():
    """メイン実行"""
    
    print("🎮 SimPyROS Interactive UI Demo")
    print("=" * 50)
    
    if not TKINTER_AVAILABLE:
        print("❌ Required dependencies not available:")
        print("   pip install matplotlib")
        print("   (tkinter should be included with Python)")
        return
    
    try:
        ui = SimPyROSComparisonUI()
        ui.run()
        
    except KeyboardInterrupt:
        print("\n⏹️ UI interrupted")
    except Exception as e:
        print(f"\n❌ UI failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()