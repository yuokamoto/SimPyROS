#!/usr/bin/env python3
"""
Process-Separated PyVista Visualizer for SimPyROS

PyVistaを別プロセスで実行し、共有メモリでデータを高速受け渡し

Features:
1. 完全なプロセス分離 - SimPyシミュレーションに影響しない
2. 共有メモリ - 高速データ転送
3. Non-blocking updates - シミュレーション性能維持
4. Crash isolation - PyVistaクラッシュがシミュレーションに影響しない

Architecture:
SimPy Process -> Shared Memory -> PyVista Process
"""

import os
import time
import signal
import numpy as np
import multiprocessing as mp
from multiprocessing import shared_memory
from typing import Dict, List, Optional, Tuple, Any
import struct
import warnings
from dataclasses import dataclass
import threading

# Add parent directory to path
import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from core.simulation_object import Pose


@dataclass
class RobotVisualizationData:
    """ロボット可視化データ構造"""
    robot_name: str
    num_links: int
    link_transforms: np.ndarray  # shape: (num_links, 4, 4)
    link_names: List[str]
    timestamp: float
    

@dataclass
class SharedMemoryConfig:
    """共有メモリ設定"""
    max_robots: int = 10
    max_links_per_robot: int = 20
    transform_size: int = 16  # 4x4 matrix = 16 floats
    update_frequency: float = 30.0  # Hz


class SharedMemoryManager:
    """
    共有メモリ管理クラス
    
    データレイアウト:
    - Header: [num_robots, update_counter, timestamp]
    - Robot Data: [robot_id, num_links, transforms[num_links][4][4], dirty_flag]
    """
    
    def __init__(self, config: SharedMemoryConfig):
        self.config = config
        self.shm_size = self._calculate_memory_size()
        
        # 共有メモリ名を一意にする（複数インスタンス対応）
        import time
        self.shm_name = f"simpyros_pyvista_{int(time.time() * 1000) % 10000}"
        
        # 共有メモリ作成
        try:
            self.shm = shared_memory.SharedMemory(
                name=self.shm_name, 
                create=True, 
                size=self.shm_size
            )
            print(f"✅ 共有メモリ作成: {self.shm_name}, {self.shm_size} bytes")
            
        except Exception as e:
            print(f"❌ 共有メモリ作成失敗: {e}")
            raise
        
        # メモリを初期化
        self.shm.buf[:] = b'\x00' * self.shm_size
        
        # メモリレイアウト計算
        self._setup_memory_layout()
        
        # ロボット管理
        self.robot_registry = {}  # robot_name -> robot_id
        self.next_robot_id = 0
        
        # ヘッダー初期化
        self._initialize_header()
        
    def _calculate_memory_size(self) -> int:
        """必要な共有メモリサイズを計算"""
        header_size = 3 * 8  # num_robots, update_counter, timestamp (double)
        
        robot_data_size = (
            8 +  # robot_id (int64)
            8 +  # num_links (int64) 
            8 +  # timestamp (double)
            8 +  # dirty_flag (int64)
            self.config.max_links_per_robot * self.config.transform_size * 8 +  # transforms (double)
            256  # robot_name (fixed string)
        )
        
        total_size = header_size + (self.config.max_robots * robot_data_size)
        
        # アライメント調整
        return ((total_size + 4095) // 4096) * 4096
        
    def _setup_memory_layout(self):
        """メモリレイアウトのオフセット計算"""
        self.header_offset = 0
        self.robots_offset = 3 * 8  # header size
        
        self.robot_size = (
            8 +  # robot_id
            8 +  # num_links
            8 +  # timestamp  
            8 +  # dirty_flag
            self.config.max_links_per_robot * self.config.transform_size * 8 +  # transforms
            256  # robot_name
        )
        
    def _initialize_header(self):
        """ヘッダー部分を初期化"""
        try:
            # num_robots = 0
            struct.pack_into('q', self.shm.buf, 0, 0)
            # update_counter = 0
            struct.pack_into('q', self.shm.buf, 8, 0)
            # timestamp = current time
            struct.pack_into('d', self.shm.buf, 16, time.time())
            
            # 全ロボットスロットを初期化
            for robot_id in range(self.config.max_robots):
                robot_offset = self.robots_offset + (robot_id * self.robot_size)
                # robot_id = -1 (未使用を示す)
                struct.pack_into('q', self.shm.buf, robot_offset, -1)
                # num_links = 0
                struct.pack_into('q', self.shm.buf, robot_offset + 8, 0)
                # dirty_flag = 0
                struct.pack_into('q', self.shm.buf, robot_offset + 24, 0)
                
            print(f"🔧 共有メモリヘッダー初期化完了 (max_robots: {self.config.max_robots})")
            
        except Exception as e:
            print(f"⚠️ ヘッダー初期化エラー: {e}")
    
    def _find_available_slot(self) -> int:
        """空いているロボットスロットを探す"""
        for robot_id in range(self.config.max_robots):
            robot_offset = self.robots_offset + (robot_id * self.robot_size)
            try:
                stored_robot_id = struct.unpack_from('q', self.shm.buf, robot_offset)[0]
                if stored_robot_id == -1:  # 未使用スロット
                    return robot_id
            except:
                continue
        return -1  # 空きスロットなし
        
    def register_robot(self, robot_name: str, num_links: int) -> int:
        """ロボットを登録し、robot_idを返す"""
        if robot_name in self.robot_registry:
            return self.robot_registry[robot_name]
        
        # 空いているスロットを探す
        robot_id = self._find_available_slot()
        if robot_id == -1:
            raise ValueError(f"Maximum robots ({self.config.max_robots}) exceeded. Current: {len(self.robot_registry)}")
            
        self.robot_registry[robot_name] = robot_id
        
        # 共有メモリにロボット情報を書き込み
        robot_offset = self.robots_offset + (robot_id * self.robot_size)
        
        # robot_id
        struct.pack_into('q', self.shm.buf, robot_offset, robot_id)
        # num_links
        struct.pack_into('q', self.shm.buf, robot_offset + 8, num_links)
        # robot_name (256 bytes固定長)
        name_bytes = robot_name.encode('utf-8')[:255]
        name_bytes += b'\x00' * (256 - len(name_bytes))
        self.shm.buf[robot_offset + 32:robot_offset + 32 + 256] = name_bytes
        
        print(f"📝 ロボット登録: {robot_name} -> ID {robot_id}, {num_links} links")
        return robot_id
        
    def update_robot_transforms(self, robot_name: str, transforms: np.ndarray) -> bool:
        """ロボットの変換行列を更新"""
        if robot_name not in self.robot_registry:
            print(f"⚠️ Unknown robot: {robot_name}")
            return False
            
        robot_id = self.robot_registry[robot_name]
        robot_offset = self.robots_offset + (robot_id * self.robot_size)
        
        try:
            # timestamp更新
            current_time = time.time()
            struct.pack_into('d', self.shm.buf, robot_offset + 16, current_time)
            
            # transforms更新 (flatten to 1D array)
            transforms_flat = transforms.flatten()
            max_elements = self.config.max_links_per_robot * self.config.transform_size
            
            if len(transforms_flat) > max_elements:
                print(f"⚠️ Too many transform elements: {len(transforms_flat)} > {max_elements}")
                transforms_flat = transforms_flat[:max_elements]
            
            # 共有メモリに書き込み
            transforms_offset = robot_offset + 32 + 256  # skip header + name
            for i, value in enumerate(transforms_flat):
                struct.pack_into('d', self.shm.buf, transforms_offset + (i * 8), float(value))
            
            # dirty flagを設定
            struct.pack_into('q', self.shm.buf, robot_offset + 24, 1)
            
            # グローバル更新カウンター
            self._increment_update_counter()
            
            return True
            
        except Exception as e:
            print(f"❌ Transform update failed: {e}")
            return False
    
    def _increment_update_counter(self):
        """グローバル更新カウンターをインクリメント"""
        try:
            counter = struct.unpack_from('q', self.shm.buf, 8)[0]
            struct.pack_into('q', self.shm.buf, 8, counter + 1)
            struct.pack_into('d', self.shm.buf, 16, time.time())
        except:
            pass
    
    def get_robot_data(self, robot_id: int) -> Optional[Dict]:
        """ロボットデータを取得（PyVistaプロセス用）"""
        if robot_id >= self.config.max_robots:
            return None
            
        robot_offset = self.robots_offset + (robot_id * self.robot_size)
        
        try:
            # ヘッダー読み取り
            stored_robot_id = struct.unpack_from('q', self.shm.buf, robot_offset)[0]
            if stored_robot_id != robot_id:
                return None
                
            num_links = struct.unpack_from('q', self.shm.buf, robot_offset + 8)[0]
            timestamp = struct.unpack_from('d', self.shm.buf, robot_offset + 16)[0]
            dirty_flag = struct.unpack_from('q', self.shm.buf, robot_offset + 24)[0]
            
            if dirty_flag == 0:
                return None  # データ更新なし
            
            # robot_name読み取り
            name_bytes = bytes(self.shm.buf[robot_offset + 32:robot_offset + 32 + 256])
            robot_name = name_bytes.split(b'\x00')[0].decode('utf-8')
            
            # transforms読み取り
            transforms_offset = robot_offset + 32 + 256
            transforms_data = []
            
            for i in range(int(num_links)):
                transform = np.zeros((4, 4))
                for row in range(4):
                    for col in range(4):
                        idx = i * 16 + row * 4 + col
                        offset = transforms_offset + (idx * 8)
                        if offset + 8 <= len(self.shm.buf):
                            value = struct.unpack_from('d', self.shm.buf, offset)[0]
                            transform[row, col] = value
                transforms_data.append(transform)
            
            # dirty flagをクリア
            struct.pack_into('q', self.shm.buf, robot_offset + 24, 0)
            
            return {
                'robot_id': robot_id,
                'robot_name': robot_name,
                'num_links': int(num_links),
                'transforms': transforms_data,
                'timestamp': timestamp
            }
            
        except Exception as e:
            print(f"❌ データ読み取りエラー: {e}")
            return None
    
    def get_update_counter(self) -> int:
        """グローバル更新カウンターを取得"""
        try:
            return struct.unpack_from('q', self.shm.buf, 8)[0]
        except:
            return 0
    
    def cleanup(self):
        """共有メモリクリーンアップ"""
        try:
            self.shm.close()
            self.shm.unlink()
            print(f"🗑️ 共有メモリクリーンアップ完了: {self.shm_name}")
        except:
            pass


class PyVistaVisualizationProcess:
    """
    別プロセスで動作するPyVistaビジュアライザー
    """
    
    def __init__(self, config: SharedMemoryConfig, shm_name: str):
        self.config = config
        self.shm_name = shm_name
        self.running = False
        
    def run(self):
        """PyVista可視化プロセスのメインループ"""
        print("🚀 PyVista可視化プロセス開始")
        
        try:
            # 共有メモリに接続
            shm = shared_memory.SharedMemory(name=self.shm_name, create=False)
            
            # PyVista初期化
            import pyvista as pv
            
            # 可視化設定（標準PyVistaと同等）
            pv.OFF_SCREEN = False  # 明示的にオンスクリーンを設定
            pv.set_plot_theme('document')  # 標準PyVistaと同じテーマ
            
            plotter = pv.Plotter(
                title="SimPyROS - Process Separated PyVista", 
                window_size=(1200, 800)  # 標準PyVistaと同じサイズ
            )
            
            # 標準PyVistaと同じ背景色
            plotter.set_background('lightblue')
            
            # GPU最適化設定（標準PyVistaと同等）
            try:
                # マルチサンプリング設定
                import vtk
                render_window = plotter.render_window
                render_window.SetMultiSamples(4)
                
                # 影を無効化してパフォーマンス向上
                plotter.enable_shadows = False
                
                print("🚀 GPU optimizations applied")
            except Exception as e:
                print(f"⚠️ GPU optimization warning: {e}")
            
            # ロボット管理
            robot_actors = {}
            last_update_counter = 0
            
            # 座標軸追加
            plotter.add_axes()
            
            # グリッド表示（標準PyVistaスタイル）
            plotter.show_grid()
            
            # ウィンドウを表示（ノンブロッキングモード）
            plotter.show(
                auto_close=False, 
                interactive_update=True
            )
            
            self.running = True
            print("✅ PyVista初期化完了")
            print("🖥️ ウィンドウ表示開始")
            
            # メインループ
            update_interval = 1.0 / self.config.update_frequency
            last_update_time = time.time()
            
            while self.running:
                current_time = time.time()
                
                try:
                    # 更新確認
                    current_counter = struct.unpack_from('q', shm.buf, 8)[0]
                    
                    if current_counter > last_update_counter:
                        # データ更新あり
                        self._update_visualization(shm, plotter, robot_actors)
                        last_update_counter = current_counter
                        last_update_time = current_time
                        
                        # レンダリング
                        plotter.render()
                    
                    # 適度な頻度で更新
                    if current_time - last_update_time >= update_interval:
                        plotter.render()
                        plotter.update()
                        last_update_time = current_time
                    
                    # CPU使用率制御
                    time.sleep(0.016)  # ~60 FPS
                    
                except KeyboardInterrupt:
                    print("🛑 KeyboardInterrupt受信")
                    break
                except Exception as e:
                    print(f"⚠️ 可視化ループエラー: {e}")
                    # ウィンドウが閉じられた場合の検出
                    if "VTK" in str(e) or "render" in str(e).lower():
                        print("🪟 ウィンドウが閉じられました")
                        break
                    time.sleep(0.1)
            
            plotter.close()
            shm.close()
            
        except Exception as e:
            print(f"❌ PyVista可視化プロセスエラー: {e}")
            import traceback
            traceback.print_exc()
        
        print("🛑 PyVista可視化プロセス終了")
    
    def _update_visualization(self, shm, plotter, robot_actors):
        """可視化更新"""
        # すべてのロボットをチェック
        for robot_id in range(self.config.max_robots):
            robot_data = self._get_robot_data_from_shm(shm, robot_id)
            
            if robot_data:
                robot_name = robot_data['robot_name']
                transforms = robot_data['transforms']
                
                # ロボットがまだ追加されていない場合
                if robot_name not in robot_actors:
                    self._add_robot_to_scene(plotter, robot_actors, robot_name, len(transforms))
                
                # 変換行列を更新
                self._update_robot_transforms(plotter, robot_actors, robot_name, transforms)
    
    def _get_robot_data_from_shm(self, shm, robot_id: int) -> Optional[Dict]:
        """共有メモリからロボットデータを取得"""
        robots_offset = 3 * 8  # header size
        robot_size = (
            8 +  # robot_id
            8 +  # num_links
            8 +  # timestamp  
            8 +  # dirty_flag
            self.config.max_links_per_robot * self.config.transform_size * 8 +  # transforms
            256  # robot_name
        )
        
        robot_offset = robots_offset + (robot_id * robot_size)
        
        try:
            # ヘッダー読み取り
            stored_robot_id = struct.unpack_from('q', shm.buf, robot_offset)[0]
            if stored_robot_id != robot_id:
                return None
                
            num_links = struct.unpack_from('q', shm.buf, robot_offset + 8)[0]
            if num_links <= 0:
                return None
                
            timestamp = struct.unpack_from('d', shm.buf, robot_offset + 16)[0]
            dirty_flag = struct.unpack_from('q', shm.buf, robot_offset + 24)[0]
            
            if dirty_flag == 0:
                return None  # データ更新なし
            
            # robot_name読み取り
            name_bytes = bytes(shm.buf[robot_offset + 32:robot_offset + 32 + 256])
            robot_name = name_bytes.split(b'\x00')[0].decode('utf-8')
            
            if not robot_name:
                return None
            
            # transforms読み取り
            transforms_offset = robot_offset + 32 + 256
            transforms_data = []
            
            for i in range(int(num_links)):
                transform = np.eye(4)  # デフォルト単位行列
                for row in range(4):
                    for col in range(4):
                        idx = i * 16 + row * 4 + col
                        offset = transforms_offset + (idx * 8)
                        if offset + 8 <= len(shm.buf):
                            try:
                                value = struct.unpack_from('d', shm.buf, offset)[0]
                                transform[row, col] = value
                            except:
                                pass
                transforms_data.append(transform)
            
            # dirty flagをクリア
            struct.pack_into('q', shm.buf, robot_offset + 24, 0)
            
            return {
                'robot_id': robot_id,
                'robot_name': robot_name,
                'num_links': int(num_links),
                'transforms': transforms_data,
                'timestamp': timestamp
            }
            
        except Exception as e:
            return None
    
    def _add_robot_to_scene(self, plotter, robot_actors, robot_name: str, num_links: int):
        """ロボットをシーンに追加"""
        import pyvista as pv
        
        robot_actors[robot_name] = []
        
        print(f"🤖 ロボット '{robot_name}' をシーンに追加 ({num_links} links)")
        
        for i in range(num_links):
            # より見やすいリンク表現を選択
            if i == 0:
                # ベースリンク（シリンダー）
                link_mesh = pv.Cylinder(radius=0.1, height=0.2)
            elif i == num_links - 1:
                # エンドエフェクタ（球）
                link_mesh = pv.Sphere(radius=0.05)
            else:
                # 中間リンク（ボックス）
                link_mesh = pv.Box(bounds=[-0.03, 0.03, -0.03, 0.03, -0.1, 0.1])
            
            # カラフルな色設定
            colors = [
                [0.8, 0.2, 0.2],  # 赤
                [0.2, 0.8, 0.2],  # 緑
                [0.2, 0.2, 0.8],  # 青
                [0.8, 0.8, 0.2],  # 黄
                [0.8, 0.2, 0.8],  # マゼンタ
            ]
            color = colors[i % len(colors)]
            
            actor = plotter.add_mesh(
                link_mesh,
                color=color,
                opacity=0.9,
                show_edges=True,
                edge_color='black'
            )
            
            robot_actors[robot_name].append({
                'actor': actor,
                'mesh': link_mesh
            })
        
        print(f"🤖 ロボット追加: {robot_name} ({num_links} links)")
    
    def _update_robot_transforms(self, plotter, robot_actors, robot_name: str, transforms: List[np.ndarray]):
        """ロボットの変換行列を更新"""
        if robot_name not in robot_actors:
            return
        
        actors = robot_actors[robot_name]
        
        for i, (transform, actor_info) in enumerate(zip(transforms, actors)):
            if i < len(actors):
                try:
                    # メッシュを変換
                    mesh = actor_info['mesh'].copy()
                    mesh.transform(transform, inplace=True)
                    
                    # アクターを更新
                    plotter.remove_actor(actor_info['actor'])
                    
                    color = [0.7, 0.3, 0.3] if i % 2 == 0 else [0.3, 0.7, 0.3]
                    new_actor = plotter.add_mesh(
                        mesh,
                        color=color,
                        opacity=0.8,
                        show_edges=True
                    )
                    
                    actor_info['actor'] = new_actor
                    
                except Exception as e:
                    print(f"⚠️ Transform update error for {robot_name} link {i}: {e}")
    
    def stop(self):
        """可視化プロセス停止"""
        self.running = False


class ProcessSeparatedPyVistaVisualizer:
    """
    プロセス分離PyVistaビジュアライザーのメインクラス
    
    SimPyシミュレーション側で使用
    """
    
    def __init__(self, config: Optional[SharedMemoryConfig] = None):
        self.config = config or SharedMemoryConfig()
        self.shm_manager = None
        self.viz_process = None
        self.available = False
        
        # パフォーマンス統計
        self.performance_stats = {
            'update_count': 0,
            'last_update_time': 0.0,
            'avg_update_time': 0.0,
            'start_time': time.time(),
            'total_robots': 0
        }
        
    def initialize(self) -> bool:
        """ビジュアライザーを初期化"""
        try:
            # 共有メモリマネージャー初期化
            self.shm_manager = SharedMemoryManager(self.config)
            
            # PyVista可視化プロセス開始（共有メモリ名を引数で渡す）
            self.viz_process = mp.Process(
                target=self._run_visualization_process,
                args=(self.shm_manager.shm_name,)
            )
            self.viz_process.start()
            
            # 少し待って初期化完了を確認
            time.sleep(2.0)  # より長い初期化時間
            
            if self.viz_process.is_alive():
                self.available = True
                print("✅ プロセス分離PyVistaビジュアライザー初期化完了")
                return True
            else:
                print("❌ PyVista可視化プロセス開始失敗")
                return False
                
        except Exception as e:
            print(f"❌ 初期化失敗: {e}")
            return False
    
    def _run_visualization_process(self, shm_name: str):
        """可視化プロセス実行"""
        try:
            print(f"🔧 プロセス分離PyVista開始 (PID: {os.getpid()})")
            print(f"   共有メモリ名: {shm_name}")
            print(f"   DISPLAY: {os.environ.get('DISPLAY', 'Not set')}")
            
            viz = PyVistaVisualizationProcess(self.config, shm_name)
            viz.run()
        except Exception as e:
            print(f"❌ 可視化プロセス実行エラー: {e}")
            import traceback
            traceback.print_exc()
    
    def add_robot(self, robot_name: str, num_links: int) -> bool:
        """ロボットを追加"""
        if not self.available or not self.shm_manager:
            return False
        
        try:
            robot_id = self.shm_manager.register_robot(robot_name, num_links)
            self.performance_stats['total_robots'] += 1
            return True
        except Exception as e:
            print(f"❌ ロボット追加失敗: {e}")
            return False
    
    def add_robot_with_urdf(self, robot_name: str, urdf_data: Dict) -> bool:
        """URDFデータ付きでロボットを追加"""
        if not self.available or not self.shm_manager:
            return False
        
        try:
            # リンク数を取得
            num_links = len(urdf_data.get('links', {}))
            
            # 基本的なロボット追加
            robot_id = self.shm_manager.register_robot(robot_name, num_links)
            self.performance_stats['total_robots'] += 1
            
            print(f"🤖 Robot '{robot_name}' added with URDF data ({num_links} links)")
            
            # URDFデータをPyVistaプロセスに転送（今後の改善で実装）
            # TODO: 共有メモリにURDFデータを保存
            
            return True
                
        except Exception as e:
            print(f"❌ URDF robot addition error: {e}")
            return False
    
    def update_robot_transforms(self, robot_name: str, transforms: List[np.ndarray]) -> bool:
        """ロボットの変換行列を更新"""
        if not self.available or not self.shm_manager:
            return False
        
        start_time = time.time()
        
        try:
            # numpy配列に変換
            transforms_array = np.array(transforms)
            
            success = self.shm_manager.update_robot_transforms(robot_name, transforms_array)
            
            if success:
                # パフォーマンス統計更新
                update_time = time.time() - start_time
                self.performance_stats['update_count'] += 1
                self.performance_stats['last_update_time'] = update_time
                
                # 平均更新時間
                count = self.performance_stats['update_count']
                if count > 1:
                    old_avg = self.performance_stats['avg_update_time']
                    self.performance_stats['avg_update_time'] = old_avg + (update_time - old_avg) / count
                else:
                    self.performance_stats['avg_update_time'] = update_time
            
            return success
            
        except Exception as e:
            print(f"❌ Transform更新失敗: {e}")
            return False
    
    def get_performance_stats(self) -> Dict:
        """パフォーマンス統計を取得"""
        total_time = time.time() - self.performance_stats['start_time']
        
        stats = self.performance_stats.copy()
        stats['total_time'] = total_time
        stats['avg_update_rate'] = self.performance_stats['update_count'] / total_time if total_time > 0 else 0
        stats['process_separated'] = True
        stats['shared_memory_size'] = self.shm_manager.shm_size if self.shm_manager else 0
        
        return stats
    
    def print_performance_summary(self):
        """パフォーマンス概要を表示"""
        stats = self.get_performance_stats()
        
        print(f"\n📊 Process-Separated PyVista Performance Summary")
        print("=" * 50)
        print(f"Total time: {stats['total_time']:.1f}s")
        print(f"Update count: {stats['update_count']}")
        print(f"Total robots: {stats['total_robots']}")
        print(f"Avg update rate: {stats['avg_update_rate']:.1f} Hz")
        print(f"Avg update time: {stats['avg_update_time']:.4f}s")
        print(f"Shared memory size: {stats['shared_memory_size']} bytes")
        print(f"Process separation: ✅ ENABLED")
    
    def shutdown(self):
        """ビジュアライザーをシャットダウン"""
        try:
            if self.viz_process and self.viz_process.is_alive():
                self.viz_process.terminate()
                self.viz_process.join(timeout=5.0)
                
                if self.viz_process.is_alive():
                    self.viz_process.kill()
                    
            if self.shm_manager:
                self.shm_manager.cleanup()
                
            print("🛑 プロセス分離PyVistaビジュアライザーシャットダウン完了")
            
        except Exception as e:
            print(f"⚠️ シャットダウンエラー: {e}")


def create_process_separated_visualizer(max_robots: int = 10, max_links_per_robot: int = 20) -> ProcessSeparatedPyVistaVisualizer:
    """プロセス分離PyVistaビジュアライザーを作成"""
    config = SharedMemoryConfig(
        max_robots=max_robots,
        max_links_per_robot=max_links_per_robot,
        update_frequency=30.0
    )
    
    visualizer = ProcessSeparatedPyVistaVisualizer(config)
    
    if visualizer.initialize():
        return visualizer
    else:
        raise RuntimeError("プロセス分離PyVistaビジュアライザー初期化失敗")


if __name__ == "__main__":
    print("🧪 プロセス分離PyVistaビジュアライザーテスト")
    print("=" * 50)
    
    try:
        # ビジュアライザー作成
        visualizer = create_process_separated_visualizer(max_robots=2, max_links_per_robot=5)
        
        # テストロボット追加
        visualizer.add_robot("test_robot", 3)
        
        print("🤖 テストロボット追加完了")
        print("🚀 5秒間のアニメーションテスト開始...")
        
        # アニメーションテスト
        start_time = time.time()
        duration = 5.0
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            # テスト用変換行列生成
            transforms = []
            for i in range(3):
                transform = np.eye(4)
                transform[0, 3] = i * 0.2  # X offset
                transform[1, 3] = 0.1 * np.sin(t * 2.0 + i)  # Y motion
                transform[2, 3] = 0.05 * np.cos(t * 1.5 + i)  # Z motion
                
                # 回転
                angle = t * 0.5 + i * np.pi / 3
                transform[0, 0] = np.cos(angle)
                transform[0, 1] = -np.sin(angle)
                transform[1, 0] = np.sin(angle)
                transform[1, 1] = np.cos(angle)
                
                transforms.append(transform)
            
            # 更新
            visualizer.update_robot_transforms("test_robot", transforms)
            
            time.sleep(1.0 / 60.0)  # 60 FPS
        
        print("✅ アニメーションテスト完了")
        
        # パフォーマンス統計表示
        visualizer.print_performance_summary()
        
        print("\n💡 Process-Separated PyVista Benefits:")
        print("   ✅ 完全なプロセス分離 - SimPyに影響しない")
        print("   ✅ 共有メモリ - 高速データ転送")
        print("   ✅ クラッシュ分離 - PyVistaエラーが伝播しない")
        print("   ✅ 独立したOpenGLコンテキスト")
        print("   ✅ Non-blocking updates")
        
        # 少し待ってからシャットダウン
        print("\n5秒後にシャットダウンします...")
        time.sleep(5.0)
        
    except KeyboardInterrupt:
        print("\n⏹️ ユーザー割り込み")
    except Exception as e:
        print(f"\n❌ テスト失敗: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            visualizer.shutdown()
        except:
            pass