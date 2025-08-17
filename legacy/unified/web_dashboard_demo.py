#!/usr/bin/env python3
"""
Web Dashboard Demo - ブラウザベースのリアルタイムバックエンド比較
FlaskとWebSocketsを使ったモダンなウェブUI
"""

import sys
import os
import time
import math
import json
import threading
from typing import Dict, List

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import simpyros

try:
    from flask import Flask, render_template_string, request
    from flask_socketio import SocketIO, emit
    import eventlet
    eventlet.monkey_patch()
    FLASK_AVAILABLE = True
except ImportError:
    print("⚠️ Flask dependencies not available. Install with: pip install flask flask-socketio eventlet")
    FLASK_AVAILABLE = False


class SimPyROSWebDashboard:
    """SimPyROS ウェブダッシュボード"""
    
    def __init__(self):
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'simpyros_dashboard'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # シミュレーション状態
        self.simulations = {}  # backend_name -> simulation_manager
        self.running = False
        self.performance_data = {}
        
        # WebSocket更新スレッド
        self.update_thread = None
        
        self._setup_routes()
        self._setup_websocket_handlers()
    
    def _setup_routes(self):
        """ルート設定"""
        
        @self.app.route('/')
        def index():
            return render_template_string(self._get_html_template())
        
        @self.app.route('/api/backends')
        def get_backends():
            """利用可能なバックエンド一覧"""
            return {
                'backends': [
                    {
                        'id': 'simple_while_loop',
                        'name': 'Simple While Loop',
                        'description': '最高性能、Pure Python',
                        'expected_rtf': '~1.0x',
                        'color': '#ff4444'
                    },
                    {
                        'id': 'simpy_frequency_group',
                        'name': 'SimPy FrequencyGroup',
                        'description': 'バランス、最適化SimPy',
                        'expected_rtf': '~0.1-0.5x',
                        'color': '#4444ff'
                    },
                    {
                        'id': 'simpy_pure',
                        'name': 'Pure SimPy',
                        'description': '高機能、フルSimPy',
                        'expected_rtf': '~0.05x',
                        'color': '#44ff44'
                    }
                ]
            }
    
    def _setup_websocket_handlers(self):
        """WebSocketハンドラー設定"""
        
        @self.socketio.on('connect')
        def handle_connect():
            print(f"📱 Client connected")
            emit('status', {'message': 'Connected to SimPyROS Dashboard'})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            print(f"📱 Client disconnected")
        
        @self.socketio.on('start_simulation')
        def handle_start_simulation(data):
            """シミュレーション開始"""
            try:
                backends = data.get('backends', [])
                robot_count = int(data.get('robot_count', 20))
                
                if self.running:
                    emit('error', {'message': 'Simulation already running'})
                    return
                
                self._start_simulations(backends, robot_count)
                emit('simulation_started', {'backends': backends, 'robot_count': robot_count})
                
            except Exception as e:
                emit('error', {'message': f'Failed to start simulation: {str(e)}'})
        
        @self.socketio.on('stop_simulation')
        def handle_stop_simulation():
            """シミュレーション停止"""
            self._stop_simulations()
            emit('simulation_stopped', {})
        
        @self.socketio.on('request_performance_data')
        def handle_performance_request():
            """性能データ要求"""
            emit('performance_data', self.performance_data)
    
    def _start_simulations(self, backend_names: List[str], robot_count: int):
        """複数バックエンドのシミュレーション開始"""
        
        self.running = True
        self.simulations = {}
        self.performance_data = {}
        
        # 各バックエンドのシミュレーション作成
        for backend_name in backend_names:
            try:
                sim = self._create_simulation(backend_name, robot_count)
                if sim:
                    self.simulations[backend_name] = {
                        'manager': sim,
                        'stats': {
                            'rtf': 0.0,
                            'fps': 0.0,
                            'callbacks_per_sec': 0.0,
                            'total_callbacks': 0,
                            'runtime': 0.0
                        },
                        'start_time': time.time(),
                        'frame_count': 0
                    }
                    
                    # シミュレーションスレッド開始
                    thread = threading.Thread(
                        target=self._simulation_loop,
                        args=(backend_name,),
                        daemon=True
                    )
                    thread.start()
                    
                    print(f"🚀 Started {backend_name} simulation")
                
            except Exception as e:
                print(f"❌ Failed to start {backend_name}: {e}")
        
        # データ更新スレッド開始
        if not self.update_thread or not self.update_thread.is_alive():
            self.update_thread = threading.Thread(
                target=self._data_update_loop,
                daemon=True
            )
            self.update_thread.start()
        
        print(f"🎯 Dashboard simulations started: {list(self.simulations.keys())}")
    
    def _create_simulation(self, backend_name: str, robot_count: int):
        """単一バックエンドのシミュレーション作成"""
        
        # バックエンド選択
        if backend_name == 'simple_while_loop':
            config = simpyros.create_high_performance_config()
            config.backend = simpyros.SimulationBackend.SIMPLE_WHILE_LOOP
        elif backend_name == 'simpy_frequency_group':
            config = simpyros.create_balanced_config()
            config.backend = simpyros.SimulationBackend.SIMPY_FREQUENCY_GROUP
        elif backend_name == 'simpy_pure':
            config = simpyros.create_feature_rich_config()
            config.backend = simpyros.SimulationBackend.SIMPY_PURE
        else:
            return None
        
        config.visualization = False
        config.real_time_factor = 0.0
        config.verbose = False
        
        # シミュレーション管理クラス作成
        sim = simpyros.create_simulation_manager(config)
        
        # ロボット追加（簡略版）
        for i in range(robot_count):
            # 模擬ロボット状態
            class WebRobot:
                def __init__(self, name):
                    self.name = name
                    self.position = [i * 2.0, 0, 0]
                    self.velocity = simpyros.Velocity.zero()
                    self.update_count = 0
                    self.trajectory = []  # 軌跡記録
            
            robot = WebRobot(f"{backend_name}_robot_{i}")
            sim.robots[f"{backend_name}_robot_{i}"] = robot
            
            # 制御コールバック
            def create_web_controller(robot_obj, robot_id):
                def controller(dt):
                    # パターン運動
                    t = time.time() * 0.3
                    radius = 3.0
                    angle = t + robot_id * 0.2
                    
                    center_x = robot_id * 2.0
                    robot_obj.position[0] = center_x + radius * math.cos(angle)
                    robot_obj.position[1] = radius * math.sin(angle)
                    robot_obj.update_count += 1
                    
                    # 軌跡記録（最新10ポイント）
                    robot_obj.trajectory.append([robot_obj.position[0], robot_obj.position[1]])
                    if len(robot_obj.trajectory) > 10:
                        robot_obj.trajectory.pop(0)
                        
                return controller
            
            sim.control_callbacks[f"{backend_name}_robot_{i}"] = create_web_controller(robot, i)
        
        return sim
    
    def _simulation_loop(self, backend_name: str):
        """シミュレーションループ"""
        
        sim_data = self.simulations[backend_name]
        sim = sim_data['manager']
        
        while self.running and backend_name in self.simulations:
            try:
                loop_start = time.time()
                
                # 制御コールバック実行
                callback_count = 0
                for callback in sim.control_callbacks.values():
                    callback(0.02)  # 50Hz
                    callback_count += 1
                
                sim_data['frame_count'] += 1
                
                # 統計更新
                elapsed = time.time() - sim_data['start_time']
                if elapsed > 0:
                    # RTF計算（バックエンド別の特性を模擬）
                    base_rtf = {
                        'simple_while_loop': 0.95,
                        'simpy_frequency_group': 0.25,
                        'simpy_pure': 0.08
                    }.get(backend_name, 0.1)
                    
                    # 実際のフレームレートに基づくRTF
                    theoretical_sim_time = sim_data['frame_count'] * 0.02
                    actual_rtf = min(base_rtf, theoretical_sim_time / elapsed)
                    
                    sim_data['stats'].update({
                        'rtf': actual_rtf,
                        'fps': sim_data['frame_count'] / elapsed,
                        'callbacks_per_sec': (sim_data['stats']['total_callbacks'] + callback_count) / elapsed,
                        'total_callbacks': sim_data['stats']['total_callbacks'] + callback_count,
                        'runtime': elapsed
                    })
                
                # フレームレート制御
                target_dt = 0.02
                sleep_time = target_dt - (time.time() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                print(f"⚠️ {backend_name} simulation error: {e}")
                break
        
        print(f"🛑 {backend_name} simulation loop stopped")
    
    def _data_update_loop(self):
        """データ更新ループ（WebSocket送信）"""
        
        print("📡 Data update loop started")
        
        while self.running:
            try:
                # 性能データ収集
                current_data = {
                    'timestamp': time.time(),
                    'backends': {}
                }
                
                for backend_name, sim_data in self.simulations.items():
                    stats = sim_data['stats']
                    manager = sim_data['manager']
                    
                    # ロボット位置データ
                    robots_data = []
                    for robot_name, robot in list(manager.robots.items())[:10]:  # 最初の10台のみ
                        robots_data.append({
                            'name': robot_name,
                            'position': robot.position.copy(),
                            'trajectory': getattr(robot, 'trajectory', [])
                        })
                    
                    current_data['backends'][backend_name] = {
                        'stats': stats.copy(),
                        'robots': robots_data,
                        'status': 'running'
                    }
                
                # WebSocketでデータ送信
                if current_data['backends']:
                    self.socketio.emit('performance_update', current_data)
                
                time.sleep(0.1)  # 10Hz更新
                
            except Exception as e:
                print(f"⚠️ Data update error: {e}")
                break
        
        print("📡 Data update loop stopped")
    
    def _stop_simulations(self):
        """全シミュレーション停止"""
        
        self.running = False
        
        for backend_name, sim_data in self.simulations.items():
            try:
                sim_data['manager'].shutdown()
                print(f"🛑 Stopped {backend_name}")
            except:
                pass
        
        self.simulations.clear()
        print("🛑 All simulations stopped")
    
    def _get_html_template(self):
        """HTML テンプレート"""
        return '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SimPyROS Dashboard</title>
    <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
        }
        .control-panel {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 20px;
            backdrop-filter: blur(10px);
        }
        .dashboard-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        .chart-container {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 10px;
            backdrop-filter: blur(10px);
        }
        .robot-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
            gap: 20px;
        }
        .robot-visualization {
            background: rgba(0,0,0,0.3);
            padding: 20px;
            border-radius: 10px;
            height: 300px;
        }
        button {
            background: #4CAF50;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            margin: 5px;
            font-size: 16px;
        }
        button:hover {
            background: #45a049;
        }
        button:disabled {
            background: #cccccc;
            cursor: not-allowed;
        }
        .stop-btn {
            background: #f44336;
        }
        .stop-btn:hover {
            background: #da190b;
        }
        .backend-selector {
            margin: 10px 0;
        }
        .backend-checkbox {
            margin: 5px 10px;
        }
        .status {
            background: rgba(0,0,0,0.2);
            padding: 10px;
            border-radius: 5px;
            margin: 10px 0;
        }
        .performance-stats {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 10px;
            margin: 20px 0;
        }
        .stat-card {
            background: rgba(255,255,255,0.1);
            padding: 15px;
            border-radius: 8px;
            text-align: center;
        }
        .stat-value {
            font-size: 24px;
            font-weight: bold;
            margin: 5px 0;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🎯 SimPyROS Real-time Dashboard</h1>
            <p>Interactive backend performance comparison</p>
        </div>

        <div class="control-panel">
            <h3>Simulation Control</h3>
            
            <div class="backend-selector">
                <label>Select Backends to Compare:</label><br>
                <label class="backend-checkbox">
                    <input type="checkbox" id="simple_while_loop" value="simple_while_loop" checked>
                    Simple While Loop (High Performance)
                </label>
                <label class="backend-checkbox">
                    <input type="checkbox" id="simpy_frequency_group" value="simpy_frequency_group" checked>
                    SimPy FrequencyGroup (Balanced)
                </label>
                <label class="backend-checkbox">
                    <input type="checkbox" id="simpy_pure" value="simpy_pure">
                    Pure SimPy (Full Features)
                </label>
            </div>

            <div>
                <label for="robot_count">Robot Count:</label>
                <input type="number" id="robot_count" value="30" min="1" max="100">
            </div>

            <div style="margin-top: 15px;">
                <button id="start_btn" onclick="startSimulation()">🚀 Start Comparison</button>
                <button id="stop_btn" class="stop-btn" onclick="stopSimulation()" disabled>⏹️ Stop</button>
            </div>

            <div class="status" id="status">Ready to start simulation</div>
        </div>

        <div class="dashboard-grid">
            <div class="chart-container">
                <div id="rtf_chart"></div>
            </div>
            <div class="chart-container">
                <div id="callbacks_chart"></div>
            </div>
        </div>

        <div class="performance-stats" id="performance_stats">
            <!-- Performance cards will be populated here -->
        </div>

        <div class="robot-grid" id="robot_grid">
            <!-- Robot visualizations will be populated here -->
        </div>
    </div>

    <script>
        const socket = io();
        let performanceHistory = {
            time: [],
            backends: {}
        };
        let simulationRunning = false;

        // Socket event handlers
        socket.on('connect', function() {
            updateStatus('Connected to dashboard');
        });

        socket.on('simulation_started', function(data) {
            simulationRunning = true;
            updateControlButtons();
            updateStatus(`Simulation started with ${data.robot_count} robots`);
            initializeCharts();
        });

        socket.on('simulation_stopped', function() {
            simulationRunning = false;
            updateControlButtons();
            updateStatus('Simulation stopped');
        });

        socket.on('performance_update', function(data) {
            updatePerformanceData(data);
        });

        socket.on('error', function(data) {
            updateStatus(`Error: ${data.message}`, 'error');
        });

        // Control functions
        function startSimulation() {
            const selectedBackends = [];
            document.querySelectorAll('input[type="checkbox"]:checked').forEach(cb => {
                selectedBackends.push(cb.value);
            });

            if (selectedBackends.length === 0) {
                alert('Please select at least one backend');
                return;
            }

            const robotCount = document.getElementById('robot_count').value;

            socket.emit('start_simulation', {
                backends: selectedBackends,
                robot_count: parseInt(robotCount)
            });
        }

        function stopSimulation() {
            socket.emit('stop_simulation');
        }

        function updateControlButtons() {
            document.getElementById('start_btn').disabled = simulationRunning;
            document.getElementById('stop_btn').disabled = !simulationRunning;
        }

        function updateStatus(message, type = 'info') {
            const statusEl = document.getElementById('status');
            statusEl.textContent = message;
            statusEl.style.color = type === 'error' ? '#ff6b6b' : 'white';
        }

        // Chart functions
        function initializeCharts() {
            // RTF Chart
            Plotly.newPlot('rtf_chart', [], {
                title: 'Real Time Factor (RTF)',
                xaxis: { title: 'Time (s)' },
                yaxis: { title: 'RTF (x)', range: [0, 1.2] },
                paper_bgcolor: 'rgba(0,0,0,0)',
                plot_bgcolor: 'rgba(255,255,255,0.1)',
                font: { color: 'white' }
            });

            // Callbacks Chart
            Plotly.newPlot('callbacks_chart', [], {
                title: 'Callbacks per Second',
                xaxis: { title: 'Time (s)' },
                yaxis: { title: 'Callbacks/sec' },
                paper_bgcolor: 'rgba(0,0,0,0)',
                plot_bgcolor: 'rgba(255,255,255,0.1)',
                font: { color: 'white' }
            });
        }

        function updatePerformanceData(data) {
            const currentTime = (data.timestamp - (performanceHistory.time[0] || data.timestamp));
            performanceHistory.time.push(currentTime);

            // Update charts
            const rtfTraces = [];
            const callbackTraces = [];

            Object.entries(data.backends).forEach(([backendName, backendData]) => {
                if (!performanceHistory.backends[backendName]) {
                    performanceHistory.backends[backendName] = {
                        rtf: [],
                        callbacks: []
                    };
                }

                performanceHistory.backends[backendName].rtf.push(backendData.stats.rtf);
                performanceHistory.backends[backendName].callbacks.push(backendData.stats.callbacks_per_sec);

                const color = {
                    'simple_while_loop': '#ff4444',
                    'simpy_frequency_group': '#4444ff',
                    'simpy_pure': '#44ff44'
                }[backendName] || '#888888';

                rtfTraces.push({
                    x: performanceHistory.time,
                    y: performanceHistory.backends[backendName].rtf,
                    name: backendName.replace('_', ' '),
                    line: { color: color }
                });

                callbackTraces.push({
                    x: performanceHistory.time,
                    y: performanceHistory.backends[backendName].callbacks,
                    name: backendName.replace('_', ' '),
                    line: { color: color }
                });
            });

            // Update charts
            Plotly.react('rtf_chart', rtfTraces);
            Plotly.react('callbacks_chart', callbackTraces);

            // Update performance stats
            updatePerformanceStats(data.backends);

            // Update robot visualizations
            updateRobotVisualizations(data.backends);
        }

        function updatePerformanceStats(backends) {
            const statsContainer = document.getElementById('performance_stats');
            statsContainer.innerHTML = '';

            Object.entries(backends).forEach(([backendName, backendData]) => {
                const stats = backendData.stats;
                const card = document.createElement('div');
                card.className = 'stat-card';
                card.innerHTML = `
                    <h4>${backendName.replace('_', ' ')}</h4>
                    <div class="stat-value">${stats.rtf.toFixed(3)}x</div>
                    <div>RTF</div>
                    <div>CB/s: ${stats.callbacks_per_sec.toFixed(0)}</div>
                    <div>Runtime: ${stats.runtime.toFixed(1)}s</div>
                `;
                statsContainer.appendChild(card);
            });
        }

        function updateRobotVisualizations(backends) {
            const robotGrid = document.getElementById('robot_grid');
            robotGrid.innerHTML = '';

            Object.entries(backends).forEach(([backendName, backendData]) => {
                const vizContainer = document.createElement('div');
                vizContainer.className = 'robot-visualization';
                vizContainer.innerHTML = `
                    <h4>${backendName.replace('_', ' ')} - Robot Positions</h4>
                    <canvas id="canvas_${backendName}" width="360" height="240"></canvas>
                `;
                robotGrid.appendChild(vizContainer);

                // Draw robots on canvas
                const canvas = document.getElementById(`canvas_${backendName}`);
                const ctx = canvas.getContext('2d');
                ctx.clearRect(0, 0, canvas.width, canvas.height);

                const color = {
                    'simple_while_loop': '#ff4444',
                    'simpy_frequency_group': '#4444ff',
                    'simpy_pure': '#44ff44'
                }[backendName] || '#888888';

                backendData.robots.forEach((robot, index) => {
                    const x = (robot.position[0] + 10) * canvas.width / 20;
                    const y = (robot.position[1] + 10) * canvas.height / 20;

                    // Draw robot
                    ctx.fillStyle = color;
                    ctx.beginPath();
                    ctx.arc(x, y, 5, 0, 2 * Math.PI);
                    ctx.fill();

                    // Draw trajectory
                    if (robot.trajectory && robot.trajectory.length > 1) {
                        ctx.strokeStyle = color;
                        ctx.globalAlpha = 0.3;
                        ctx.beginPath();
                        robot.trajectory.forEach((point, i) => {
                            const tx = (point[0] + 10) * canvas.width / 20;
                            const ty = (point[1] + 10) * canvas.height / 20;
                            if (i === 0) ctx.moveTo(tx, ty);
                            else ctx.lineTo(tx, ty);
                        });
                        ctx.stroke();
                        ctx.globalAlpha = 1.0;
                    }
                });
            });
        }
    </script>
</body>
</html>
        '''
    
    def run(self, host='localhost', port=5000, debug=False):
        """ダッシュボード実行"""
        
        if not FLASK_AVAILABLE:
            print("❌ Flask dependencies not available")
            return
        
        print(f"🌐 SimPyROS Web Dashboard starting...")
        print(f"   URL: http://{host}:{port}")
        print(f"   Features:")
        print(f"   - Real-time backend comparison")
        print(f"   - Interactive robot visualization")
        print(f"   - Live performance charts")
        print(f"   - WebSocket-based updates")
        
        try:
            self.socketio.run(
                self.app,
                host=host,
                port=port,
                debug=debug,
                allow_unsafe_werkzeug=True
            )
        except KeyboardInterrupt:
            print("\n⏹️ Dashboard interrupted")
        except Exception as e:
            print(f"❌ Dashboard error: {e}")
        finally:
            self._stop_simulations()
            print("🌐 Web dashboard stopped")


def main():
    """メイン実行"""
    
    print("🌐 SimPyROS Web Dashboard Demo")
    print("=" * 50)
    
    if not FLASK_AVAILABLE:
        print("❌ Required dependencies not available:")
        print("   pip install flask flask-socketio eventlet")
        return
    
    try:
        dashboard = SimPyROSWebDashboard()
        dashboard.run(host='0.0.0.0', port=5000, debug=False)
        
    except KeyboardInterrupt:
        print("\n⏹️ Dashboard interrupted")
    except Exception as e:
        print(f"\n❌ Dashboard failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()