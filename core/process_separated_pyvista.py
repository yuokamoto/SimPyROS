#!/usr/bin/env python3
"""
Process-Separated PyVista Visualizer for SimPyROS

Optimized architecture:
1. One-time geometry transfer via multiprocessing.Queue
2. Continuous link_pose updates via shared memory
3. Standard PyVistaVisualizer logic compatibility
"""

import os
import time
import signal
import numpy as np
import multiprocessing as mp
from multiprocessing import shared_memory, Queue
from typing import Dict, List, Optional, Tuple, Any
import struct
import warnings
from dataclasses import dataclass
import threading
import pickle

# Add parent directory to path
import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from core.simulation_object import Pose


@dataclass
class RobotGeometryData:
    """Robot geometry information (one-time transfer)"""
    robot_name: str
    links_data: Dict[str, Any]  # Detailed serializable link data
    urdf_links_data: Dict[str, Any]  # URDF visual origin data 
    initial_pose: Pose
    timestamp: float


@dataclass
class RobotPoseUpdate:
    """Robot pose update (continuous shared memory)"""
    robot_name: str
    link_poses: Dict[str, Pose]  # link_name -> current_pose
    timestamp: float


@dataclass 
class SharedMemoryConfig:
    """Shared memory configuration for pose updates only"""
    max_robots: int = 10
    max_links_per_robot: int = 20
    update_frequency: float = 30.0  # Hz
    
    # Data size calculations
    pose_size: int = 7  # x, y, z, qw, qx, qy, qz


class PoseSharedMemoryManager:
    """
    Shared memory manager for link poses only
    
    Data layout:
    - Header: [num_robots, update_counter, timestamp]
    - Robot Data: [robot_id, num_links, link_poses[num_links][7], dirty_flag]
    """
    
    def __init__(self, config: SharedMemoryConfig):
        self.config = config
        self.shm_size = self._calculate_memory_size()
        
        # Generate unique shared memory name
        import time
        self.shm_name = f"simpyros_poses_{int(time.time() * 1000) % 10000}"
        
        # Create shared memory
        try:
            self.shm = shared_memory.SharedMemory(
                name=self.shm_name, 
                create=True, 
                size=self.shm_size
            )
            print(f"✅ 共有メモリ作成(Pose): {self.shm_name}, {self.shm_size} bytes")
            
        except Exception as e:
            print(f"❌ 共有メモリ作成失敗: {e}")
            raise
        
        # Initialize memory
        self.shm.buf[:] = b'\x00' * self.shm_size
        
        # Setup memory layout
        self._setup_memory_layout()
        
        # Robot registry
        self.robot_registry = {}  # robot_name -> robot_id
        self.next_robot_id = 0
        
        # Initialize header
        self._initialize_header()
        
    def _calculate_memory_size(self) -> int:
        """Calculate required memory size for poses only"""
        header_size = 3 * 8  # num_robots, update_counter, timestamp
        
        # Robot data size calculation
        robot_header_size = 3 * 8  # robot_id, num_links, dirty_flag
        link_poses_size = self.config.max_links_per_robot * self.config.pose_size * 8  # poses
        robot_name_size = 64  # Fixed string size
        robot_data_size = robot_header_size + link_poses_size + robot_name_size
        
        total_size = header_size + (self.config.max_robots * robot_data_size)
        
        # Alignment
        return ((total_size + 4095) // 4096) * 4096
        
    def _setup_memory_layout(self):
        """Setup memory layout offsets"""
        self.header_offset = 0
        self.robots_offset = 3 * 8  # header size
        
        self.robot_size = (
            8 +  # robot_id
            8 +  # num_links  
            8 +  # dirty_flag
            self.config.max_links_per_robot * self.config.pose_size * 8 +  # link poses
            64   # robot_name
        )
        
    def _initialize_header(self):
        """Initialize shared memory header"""
        header_data = struct.pack('QQd', 0, 0, time.time())  # num_robots, counter, timestamp
        self.shm.buf[self.header_offset:self.header_offset + len(header_data)] = header_data
        
    def register_robot(self, robot_name: str, num_links: int) -> int:
        """Register a new robot and return robot_id"""
        if robot_name in self.robot_registry:
            return self.robot_registry[robot_name]
            
        robot_id = self.next_robot_id
        self.robot_registry[robot_name] = robot_id
        self.next_robot_id += 1
        
        # Update num_robots in header
        current_header = struct.unpack('QQd', self.shm.buf[self.header_offset:self.header_offset + 24])
        new_header = struct.pack('QQd', self.next_robot_id, current_header[1], time.time())
        self.shm.buf[self.header_offset:self.header_offset + 24] = new_header
        
        print(f"🤖 Robot registered: {robot_name} (ID: {robot_id}, Links: {num_links})")
        return robot_id
        
    def update_robot_poses(self, robot_name: str, link_poses: Dict[str, Pose]) -> bool:
        """Update robot link poses in shared memory"""
        if robot_name not in self.robot_registry:
            print(f"⚠️ Robot {robot_name} not registered")
            return False
            
        robot_id = self.robot_registry[robot_name]
        robot_offset = self.robots_offset + (robot_id * self.robot_size)
        
        try:
            # Convert poses to numpy array
            pose_data = []
            link_names = sorted(link_poses.keys())  # Consistent ordering
            
            for link_name in link_names[:self.config.max_links_per_robot]:
                pose = link_poses[link_name]
                # Get quaternion as [x, y, z, w] from scipy
                quat = pose.quaternion  # [x, y, z, w]
                pose_array = [
                    pose.x, pose.y, pose.z,
                    quat[3], quat[0], quat[1], quat[2]  # [w, x, y, z] order for shared memory
                ]
                pose_data.extend(pose_array)
            
            # Pad if necessary
            while len(pose_data) < self.config.max_links_per_robot * self.config.pose_size:
                pose_data.append(0.0)
            
            # Pack robot data with fixed size
            max_pose_data_size = self.config.max_links_per_robot * self.config.pose_size
            
            # Pack robot header first
            robot_header = struct.pack('QQQ', robot_id, len(link_names), 1)  # dirty_flag=1
            
            # Pack pose data with fixed size
            pose_data_bytes = struct.pack(f'{max_pose_data_size}d', *pose_data)
            
            # Pack robot name
            robot_name_bytes = robot_name.encode('utf-8')[:63] + b'\x00'
            robot_name_padded = robot_name_bytes.ljust(64, b'\x00')
            
            # Calculate exact offsets
            header_end = robot_offset + 24  # 3 * 8 bytes for header
            pose_end = header_end + len(pose_data_bytes)
            name_end = pose_end + 64
            
            # Write to shared memory in fixed chunks
            self.shm.buf[robot_offset:header_end] = robot_header
            self.shm.buf[header_end:pose_end] = pose_data_bytes
            self.shm.buf[pose_end:name_end] = robot_name_padded
            
            # Update header counter
            current_header = struct.unpack('QQd', self.shm.buf[self.header_offset:self.header_offset + 24])
            new_header = struct.pack('QQd', current_header[0], current_header[1] + 1, time.time())
            self.shm.buf[self.header_offset:self.header_offset + 24] = new_header
            
            return True
            
        except Exception as e:
            print(f"❌ Pose update failed for {robot_name}: {e}")
            return False
    
    def cleanup(self):
        """Cleanup shared memory"""
        try:
            self.shm.close()
            self.shm.unlink()
            print(f"🧹 共有メモリクリーンアップ: {self.shm_name}")
        except:
            pass


class PyVistaVisualizationProcess:
    """PyVista visualization process with geometry + pose updates"""
    
    def __init__(self, config: SharedMemoryConfig, shm_name: str, geometry_queue: Queue):
        self.config = config
        self.shm_name = shm_name
        self.geometry_queue = geometry_queue
        
        # Robot storage
        self.robots = {}  # robot_name -> visualization data
        self.robot_actors = {}  # robot_name -> pyvista actors
        self.robot_initial_meshes = {}  # robot_name -> {link_name -> initial_mesh}
        
        # Performance tracking
        self.frame_count = 0
        self.start_time = time.time()
        
        # PyVista module reference (will be set in run())
        self.pv = None
        self.plotter = None
        self.shm = None
        
    def run(self):
        """Main visualization process loop"""
        print(f"🚀 PyVista process starting (PID: {os.getpid()})")
        
        try:
            # Import PyVista in the visualization process
            print("🔄 Importing PyVista...")
            import pyvista as pv
            self.pv = pv  # Store reference for other methods
            print(f"✅ PyVista imported successfully (version: {pv.__version__})")
            
            # Check if we have a display available
            display = os.environ.get('DISPLAY', '')
            interactive_mode = bool(display and display != '')
            print(f"📺 Display check: DISPLAY={display}, interactive_mode={interactive_mode}")
            
            if not interactive_mode:
                print("💻 Headless mode detected - starting Xvfb")
                pv.start_xvfb()  # Only for headless support
                pv.OFF_SCREEN = True
            else:
                print(f"📺 Interactive mode detected - DISPLAY={display}")
                # Set PyVista to use the display
                pv.OFF_SCREEN = False
                print(f"⚙️ PyVista OFF_SCREEN set to: {pv.OFF_SCREEN}")
                
            # Setup plotter with appropriate settings
            print("🎨 Creating plotter...")
            self.plotter = pv.Plotter(
                window_size=(1200, 800), 
                off_screen=not interactive_mode,
                title="SimPyROS Process-Separated PyVista"
            )
            print("✅ Plotter created successfully")
            
            print("🔧 Adding axes...")
            self.plotter.add_axes()
            print("✅ Axes added")
            
            print(f"🚀 PyVista可視化プロセス開始 ({'Interactive' if interactive_mode else 'Headless'} mode)")
            
            # Connect to shared memory
            self.shm = shared_memory.SharedMemory(name=self.shm_name)
            
            # Setup initial scene
            self._setup_scene()
            
            # Start main loop
            self._main_loop()
            
        except ImportError as e:
            print(f"❌ PyVista not available in visualization process: {e}")
            import traceback
            traceback.print_exc()
        except Exception as e:
            print(f"❌ PyVista可視化プロセスエラー: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print(f"🛑 PyVista可視化プロセス終了 (PID: {os.getpid()})")
            
    def _setup_scene(self):
        """Setup initial PyVista scene"""
        print("🎨 Setting up PyVista scene...")
        
        # Add ground plane
        plane = self.pv.Plane(i_size=10, j_size=10)
        self.plotter.add_mesh(plane, color='lightgray', opacity=0.3)
        print("✅ Ground plane added")
        
        # Set camera
        self.plotter.camera_position = [(5, 5, 5), (0, 0, 0), (0, 0, 1)]
        print("✅ Camera position set")
        
    def _main_loop(self):
        """Main visualization update loop"""
        # Check if we're in interactive mode
        display = os.environ.get('DISPLAY', '')
        interactive_mode = bool(display and display != '')
        
        if interactive_mode:
            # Interactive mode - show window and keep updating
            print("📺 Opening interactive PyVista window...")
            print(f"   Window size: {self.plotter.window_size}")
            print(f"   OFF_SCREEN: {self.pv.OFF_SCREEN}")
            
            try:
                self.plotter.show(auto_close=False, interactive_update=True, interactive=True)
                print("✅ Interactive window opened successfully")
            except Exception as e:
                print(f"❌ Failed to open interactive window: {e}")
                import traceback
                traceback.print_exc()
                return
            
            # Keep the window alive and update in background
            import threading
            
            def update_loop():
                while True:
                    try:
                        # Check for new robot geometry
                        self._process_geometry_queue()
                        
                        # Update robot poses from shared memory
                        self._update_robot_poses()
                        
                        # Update visualization
                        if hasattr(self.plotter, 'render'):
                            self.plotter.render()
                        self.frame_count += 1
                        
                        # Control frame rate
                        time.sleep(1.0 / self.config.update_frequency)
                        
                    except KeyboardInterrupt:
                        break
                    except Exception as e:
                        print(f"⚠️ Visualization loop error: {e}")
                        time.sleep(0.1)
            
            # Start update thread
            update_thread = threading.Thread(target=update_loop, daemon=True)
            update_thread.start()
            
            # Keep main thread alive for window interaction
            try:
                while True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
                
        else:
            # Headless mode - simple loop
            print("💻 Headless visualization loop")
            while True:
                try:
                    # Check for new robot geometry
                    self._process_geometry_queue()
                    
                    # Update robot poses from shared memory
                    self._update_robot_poses()
                    
                    # Render offscreen
                    self.plotter.render()
                    self.frame_count += 1
                    
                    # Control frame rate
                    time.sleep(1.0 / self.config.update_frequency)
                    
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"⚠️ Visualization loop error: {e}")
                    time.sleep(0.1)
                    
        self.plotter.close()
        
    def _process_geometry_queue(self):
        """Process robot geometry from queue"""
        try:
            while not self.geometry_queue.empty():
                geometry_data: RobotGeometryData = self.geometry_queue.get_nowait()
                self._load_robot_geometry(geometry_data)
        except:
            pass  # Queue empty
            
    def _load_robot_geometry(self, geometry_data: RobotGeometryData):
        """Load robot geometry using extracted data with URDFRobotVisualizer logic"""
        robot_name = geometry_data.robot_name
        links_data = geometry_data.links_data
        urdf_links_data = geometry_data.urdf_links_data
        
        print(f"🎨 Loading geometry for robot: {robot_name} using URDFRobotVisualizer logic")
        
        # Use the exact same logic as URDFRobotVisualizer._create_robot_link_actors_from_robot
        actors = {}
        initial_meshes = {}
        
        for link_name, link_data in links_data.items():
            # Create mesh based on geometry type (same as URDFRobotVisualizer)
            mesh = None
            geometry_type = link_data.get('geometry_type', 'box')
            geometry_params = link_data.get('geometry_params', {})
            color = link_data.get('color', [0.6, 0.6, 0.6, 1.0])
            
            print(f"  🔧 Creating {geometry_type} mesh for {link_name}")
            print(f"      Params: {geometry_params}")
            print(f"      Color: {color}")
            
            if geometry_type == "box":
                size = geometry_params.get('size', [0.3, 0.3, 0.1])
                mesh = self.pv.Cube(x_length=size[0], y_length=size[1], z_length=size[2])
                
            elif geometry_type == "cylinder":
                radius = geometry_params.get('radius', 0.05)
                length = geometry_params.get('length', 0.35)
                # Use same parameters as URDFRobotVisualizer
                mesh = self.pv.Cylinder(radius=radius, height=length, direction=(0, 0, 1))
                
            elif geometry_type == "sphere":
                radius = geometry_params.get('radius', 0.08)
                mesh = self.pv.Sphere(radius=radius)
            
            if mesh is not None:
                # Store original mesh before any transforms
                original_mesh = mesh.copy()
                
                # Apply visual origin transformation from URDF data (same as URDFRobotVisualizer)
                if link_name in urdf_links_data and urdf_links_data[link_name].get('has_pose', False):
                    urdf_link_data = urdf_links_data[link_name]
                    position = np.array(urdf_link_data['position'])
                    quaternion = np.array(urdf_link_data['quaternion'])
                    
                    # Recreate Pose and transform matrix
                    from core.simulation_object import Pose
                    pose = Pose.from_position_quaternion(position, quaternion)
                    transform_matrix = pose.to_transformation_matrix()
                    original_mesh.transform(transform_matrix, inplace=True)
                    print(f"      🔧 Applied visual origin to {link_name}: pos={position}")
                
                # Use color from extracted link data (same as URDFRobotVisualizer)
                color_rgb = color[:3] if len(color) >= 3 else (0.6, 0.6, 0.6)
                print(f"      🎨 Using link color for {link_name}: {color_rgb}")
                
                # Add to plotter (same as URDFRobotVisualizer)
                display_mesh = original_mesh.copy()
                actor = self.plotter.add_mesh(
                    display_mesh, 
                    color=color_rgb,
                    opacity=1.0,
                    name=f"{robot_name}_{link_name}"
                )
                
                # Store both actor and initial mesh
                actors[link_name] = actor
                initial_meshes[link_name] = original_mesh
                
                print(f"  ✅ Created {geometry_type} mesh for link: {link_name} with color {color_rgb}")
        
        self.robot_actors[robot_name] = actors
        self.robot_initial_meshes[robot_name] = initial_meshes
        self.robots[robot_name] = geometry_data
        print(f"✅ Robot geometry loaded: {robot_name} ({len(actors)} links)")
        
    def _update_robot_poses(self):
        """Update robot poses from shared memory"""
        try:
            # Read header
            header_data = struct.unpack('QQd', self.shm.buf[0:24])
            num_robots, update_counter, timestamp = header_data
            
            robots_offset = 24
            
            for robot_id in range(int(num_robots)):
                robot_offset = robots_offset + (robot_id * self._get_robot_data_size())
                
                # Read robot data
                robot_header = struct.unpack('QQQ', self.shm.buf[robot_offset:robot_offset + 24])
                robot_id_read, num_links, dirty_flag = robot_header
                
                if dirty_flag == 0:
                    continue  # No update needed
                
                # Read robot name
                name_offset = robot_offset + 24 + (self.config.max_links_per_robot * 7 * 8)
                name_bytes = bytes(self.shm.buf[name_offset:name_offset + 64])
                robot_name = name_bytes.decode('utf-8').rstrip('\x00')
                
                if robot_name not in self.robot_actors:
                    continue
                
                # Read pose data
                pose_offset = robot_offset + 24
                pose_data = struct.unpack(
                    f'{self.config.max_links_per_robot * 7}d',
                    self.shm.buf[pose_offset:pose_offset + self.config.max_links_per_robot * 7 * 8]
                )
                
                # Update actor positions
                actors = self.robot_actors[robot_name]
                initial_meshes = self.robot_initial_meshes.get(robot_name, {})
                link_names = sorted(actors.keys())
                
                for i, link_name in enumerate(link_names[:int(num_links)]):
                    if link_name in actors and link_name in initial_meshes:
                        base_idx = i * 7
                        x, y, z = pose_data[base_idx:base_idx + 3]
                        qw, qx, qy, qz = pose_data[base_idx + 3:base_idx + 7]  # [w, x, y, z]
                        
                        # Get actor and initial mesh
                        actor = actors[link_name]
                        initial_mesh = initial_meshes[link_name]
                        
                        # Create new mesh from initial state
                        new_mesh = initial_mesh.copy()
                        
                        # Apply current pose transformation
                        from scipy.spatial.transform import Rotation
                        if qw != 0 or qx != 0 or qy != 0 or qz != 0:  # Valid quaternion
                            # Convert quaternion [w, x, y, z] to scipy format [x, y, z, w]
                            rotation = Rotation.from_quat([qx, qy, qz, qw])
                            rotation_matrix = rotation.as_matrix()
                            
                            # Create 4x4 transform matrix
                            transform = np.eye(4)
                            transform[:3, :3] = rotation_matrix
                            transform[:3, 3] = [x, y, z]
                            
                            # Apply transform to new mesh (from initial state)
                            new_mesh.transform(transform)
                        else:
                            # No rotation, just translation
                            transform = np.eye(4)
                            transform[:3, 3] = [x, y, z]
                            new_mesh.transform(transform)
                        
                        # Update actor with new mesh
                        if hasattr(actor, 'GetMapper') and hasattr(actor.GetMapper(), 'GetInput'):
                            # Replace the mesh data in the actor
                            current_mesh = actor.GetMapper().GetInput()
                            if hasattr(current_mesh, 'copy_from'):
                                current_mesh.copy_from(new_mesh)
                            elif hasattr(current_mesh, 'DeepCopy'):
                                current_mesh.DeepCopy(new_mesh)
                            else:
                                # Fallback: remove and re-add actor
                                self.plotter.remove_actor(actor)
                                # Use default color since we don't have geometry_data here
                                color = 'blue'
                                new_actor = self.plotter.add_mesh(
                                    new_mesh,
                                    color=color,
                                    opacity=0.8,
                                    name=f"{robot_name}_{link_name}"
                                )
                                actors[link_name] = new_actor
                        
        except Exception as e:
            print(f"⚠️ Pose update error: {e}")
            
    def _get_robot_data_size(self):
        """Get size of robot data in shared memory"""
        return (
            8 +  # robot_id
            8 +  # num_links  
            8 +  # dirty_flag
            self.config.max_links_per_robot * 7 * 8 +  # link poses
            64   # robot_name
        )


class ProcessSeparatedPyVistaVisualizer:
    """
    Process-separated PyVista visualizer with optimized data flow
    
    Features:
    1. One-time geometry transfer via Queue
    2. Continuous pose updates via shared memory
    3. Standard PyVista logic compatibility
    """
    
    def __init__(self, config: Optional[SharedMemoryConfig] = None):
        self.config = config or SharedMemoryConfig()
        
        # Communication channels
        self.geometry_queue = Queue()
        self.pose_manager = None
        self.viz_process = None
        
        # State tracking
        self.robots = {}
        self.is_initialized = False
        self.available = True  # For compatibility
        
    def initialize(self) -> bool:
        """Initialize the process-separated visualizer"""
        try:
            print("🚀 Initializing ProcessSeparatedPyVistaVisualizer...")
            
            # Create shared memory for poses
            print("💾 Creating shared memory...")
            self.pose_manager = PoseSharedMemoryManager(self.config)
            print(f"✅ Shared memory created: {self.pose_manager.shm_name}")
            
            # Start visualization process
            print("🚀 Starting visualization process...")
            self.viz_process = mp.Process(
                target=self._run_visualization_process,
                args=(self.config, self.pose_manager.shm_name, self.geometry_queue)
            )
            self.viz_process.start()
            print(f"✅ Visualization process started (PID: {self.viz_process.pid})")
            
            # Wait for process to start and check if it's alive
            time.sleep(2.0)  # Give more time for startup
            
            if self.viz_process.is_alive():
                print(f"✅ Visualization process is running (PID: {self.viz_process.pid})")
                self.is_initialized = True
                print("✅ ProcessSeparatedPyVistaVisualizer 初期化完了")
                return True
            else:
                print(f"❌ Visualization process died (exit code: {self.viz_process.exitcode})")
                return False
            
        except Exception as e:
            print(f"❌ 初期化失敗: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def add_robot(self, robot_name: str, robot_instance: Any) -> bool:
        """Add robot with geometry and pose data"""
        if not self.is_initialized:
            return False
            
        try:
            # Extract comprehensive robot data to replicate URDFRobotVisualizer logic
            links_data = {}
            urdf_links_data = {}
            
            if hasattr(robot_instance, 'links'):
                for link_name, link_obj in robot_instance.links.items():
                    # Extract all data needed for URDFRobotVisualizer logic
                    links_data[link_name] = {
                        'geometry_type': getattr(link_obj, 'geometry_type', 'box'),
                        'geometry_params': getattr(link_obj, 'geometry_params', {}),
                        'color': getattr(link_obj, 'color', [0.6, 0.6, 0.6, 1.0])  # This is the key!
                    }
                    
                    print(f"  📋 Extracted {link_name}: {links_data[link_name]}")
            
            # Extract URDF visual origin data
            if hasattr(robot_instance, 'urdf_loader') and robot_instance.urdf_loader:
                urdf_loader = robot_instance.urdf_loader
                if hasattr(urdf_loader, 'links'):
                    for link_name, urdf_link_info in urdf_loader.links.items():
                        if hasattr(urdf_link_info, 'pose') and urdf_link_info.pose is not None:
                            # Store pose data for visual origin transformation
                            pose = urdf_link_info.pose
                            urdf_links_data[link_name] = {
                                'has_pose': True,
                                'position': pose.position.tolist() if hasattr(pose.position, 'tolist') else list(pose.position),
                                'quaternion': pose.quaternion.tolist() if hasattr(pose.quaternion, 'tolist') else list(pose.quaternion)
                            }
                            print(f"  🔧 URDF visual origin for {link_name}: {urdf_links_data[link_name]}")
                        else:
                            urdf_links_data[link_name] = {'has_pose': False}
            
            # Send comprehensive geometry data
            geometry_data = RobotGeometryData(
                robot_name=robot_name,
                links_data=links_data,
                urdf_links_data=urdf_links_data,
                initial_pose=robot_instance.pose if hasattr(robot_instance, 'pose') else Pose(),
                timestamp=time.time()
            )
            self.geometry_queue.put(geometry_data)
            
            # Register robot for pose updates
            if hasattr(robot_instance, 'links'):
                num_links = len(robot_instance.links)
            else:
                num_links = 1
                
            self.pose_manager.register_robot(robot_name, num_links)
            self.robots[robot_name] = robot_instance
            
            print(f"✅ Robot added: {robot_name}")
            return True
            
        except Exception as e:
            print(f"❌ Robot add failed: {e}")
            return False
    
    def update_robot_poses(self, robot_name: str, link_poses: Dict[str, Pose]) -> bool:
        """Update robot link poses (high-frequency)"""
        if not self.is_initialized or robot_name not in self.robots:
            return False
            
        return self.pose_manager.update_robot_poses(robot_name, link_poses)
    
    def shutdown(self):
        """Shutdown the visualizer"""
        if self.viz_process and self.viz_process.is_alive():
            self.viz_process.terminate()
            self.viz_process.join(timeout=2.0)
            
        if self.pose_manager:
            self.pose_manager.cleanup()
            
        print("🛑 ProcessSeparatedPyVistaVisualizer 終了")
    
    @staticmethod
    def _run_visualization_process(config: SharedMemoryConfig, shm_name: str, geometry_queue: Queue):
        """Static method to run visualization process"""
        print(f"🚀 Starting PyVista visualization process (PID: {os.getpid()})")
        print(f"   DISPLAY: {os.environ.get('DISPLAY', 'NOT_SET')}")
        print(f"   Shared memory: {shm_name}")
        
        try:
            viz = PyVistaVisualizationProcess(config, shm_name, geometry_queue)
            viz.run()
        except Exception as e:
            print(f"❌ Visualization process failed: {e}")
            import traceback
            traceback.print_exc()


def create_process_separated_visualizer(config: Optional[SharedMemoryConfig] = None):
    """Factory function to create the visualizer"""
    return ProcessSeparatedPyVistaVisualizer(config)


if __name__ == "__main__":
    # Test the visualizer
    config = SharedMemoryConfig(max_robots=3, max_links_per_robot=10)
    visualizer = ProcessSeparatedPyVistaVisualizer(config)
    
    if visualizer.initialize():
        print("🧪 Test mode - visualizer initialized")
        time.sleep(5.0)
        visualizer.shutdown()
    else:
        print("❌ Test failed - initialization error")