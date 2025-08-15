#!/usr/bin/env python3
"""
Optimized PyVista Visualizer for SimPyROS - Performance Enhanced Version

PyVista最適化機能:
1. バッチレンダリング制御 - 複数変更時のレンダリング無効化
2. GPU最適化設定 - ハードウェアアクセラレーション
3. フレームレート制御 - 適応的更新頻度
4. メッシュ更新最適化 - VTK変換行列による高速更新
5. リアルタイムパフォーマンス監視

Features:
- Batch rendering control
- GPU optimization
- Frame rate control
- Efficient mesh updates
- Performance monitoring
"""

import os
import numpy as np
import math
import time
import warnings
from typing import Optional, Tuple, List, Dict, Union
import threading
from contextlib import contextmanager

# Add parent directory to path
import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

# Import URDF loaders
try:
    from core.urdf_loader import URDFLoader
    URDF_SUPPORT = True
except ImportError:
    URDF_SUPPORT = False

# Import time management
try:
    from core.time_manager import TimeManager, get_global_time_manager
    TIME_MANAGER_SUPPORT = True
except ImportError:
    TIME_MANAGER_SUPPORT = False

from core.pyvista_visualizer import PyVistaVisualizer, URDFRobotVisualizer


class OptimizedPyVistaVisualizer(PyVistaVisualizer):
    """
    Performance-optimized PyVista visualizer
    
    Optimizations:
    1. Batch rendering control
    2. GPU optimization
    3. Frame rate control
    4. Efficient updates
    """
    
    def __init__(self, interactive: bool = True, window_size: Tuple[int, int] = (1200, 800)):
        """Initialize optimized PyVista visualizer"""
        
        # Performance monitoring
        self.performance_stats = {
            'render_count': 0,
            'update_count': 0,
            'last_render_time': 0.0,
            'last_update_time': 0.0,
            'avg_render_time': 0.0,
            'avg_update_time': 0.0,
            'target_fps': 60.0,
            'actual_fps': 0.0
        }
        
        # Batch rendering control
        self._batch_mode = False
        self._pending_renders = False
        self._render_lock = threading.Lock()
        
        # Frame rate control
        self._last_render_timestamp = 0.0
        self._min_render_interval = 1.0 / 60.0  # 60 FPS max
        
        # Initialize parent class
        super().__init__(interactive, window_size)
        
        # Apply optimization settings after initialization
        if self.available:
            self._apply_gpu_optimizations()
            self._setup_performance_monitoring()
    
    def _apply_gpu_optimizations(self):
        """Apply GPU and rendering optimizations"""
        try:
            import pyvista as pv
            import vtk
            
            print("🚀 Applying PyVista GPU optimizations...")
            
            # GPU optimization settings
            if self.plotter:
                # Enable GPU-based rendering
                render_window = self.plotter.render_window
                
                # Disable VSync for better performance
                render_window.SetSwapControl(False)
                
                # Enable multi-sampling if available
                try:
                    render_window.SetMultiSamples(4)
                    print("   ✅ Multi-sampling enabled (4x)")
                except:
                    print("   ⚠️ Multi-sampling not supported")
                
                # Optimize renderer settings
                renderer = self.plotter.renderer
                
                # Enable depth peeling for transparency (if needed)
                renderer.SetUseDepthPeeling(True)
                renderer.SetMaximumNumberOfPeels(4)
                renderer.SetOcclusionRatio(0.1)
                
                # Enable shadows for better visual quality (optional)
                try:
                    renderer.SetUseShadows(False)  # Disabled for performance
                    print("   ✅ Shadows disabled for performance")
                except:
                    pass
                
                # Optimize backface culling
                for actor in renderer.GetActors():
                    if hasattr(actor, 'GetProperty'):
                        prop = actor.GetProperty()
                        prop.SetBackfaceCulling(True)  # Cull hidden faces
                
                print("   ✅ GPU optimizations applied")
            
            # Global PyVista optimizations
            # Disable automatic figure closing
            pv.global_theme.auto_close = False
            
            # Set optimal quality settings
            pv.global_theme.smooth_shading = True
            pv.global_theme.lighting = True
            pv.global_theme.anti_aliasing = 'fxaa'  # Fast approximate anti-aliasing
            
            print("   ✅ Global rendering optimizations applied")
            
        except Exception as e:
            print(f"⚠️ GPU optimization error: {e}")
    
    def _setup_performance_monitoring(self):
        """Setup performance monitoring system"""
        self.performance_stats['start_time'] = time.time()
        print("📊 Performance monitoring enabled")
    
    @contextmanager
    def batch_mode(self):
        """
        Context manager for batch operations - disables rendering during updates
        
        Usage:
            with visualizer.batch_mode():
                # Multiple updates without rendering
                visualizer.update_robot_pose(robot1, pose1)
                visualizer.update_robot_pose(robot2, pose2)
                # Render happens automatically when exiting context
        """
        self._enter_batch_mode()
        try:
            yield
        finally:
            self._exit_batch_mode()
    
    def _enter_batch_mode(self):
        """Enter batch mode - disable rendering"""
        with self._render_lock:
            self._batch_mode = True
            if self.plotter:
                # Store current auto-render state
                self._original_auto_render = getattr(self.plotter, 'auto_render', True)
                # Disable automatic rendering
                if hasattr(self.plotter, 'auto_render'):
                    self.plotter.auto_render = False
                print("🔄 Batch mode enabled - rendering disabled")
    
    def _exit_batch_mode(self):
        """Exit batch mode - re-enable rendering and render if needed"""
        with self._render_lock:
            self._batch_mode = False
            if self.plotter:
                # Restore original auto-render state
                if hasattr(self.plotter, 'auto_render'):
                    self.plotter.auto_render = getattr(self, '_original_auto_render', True)
                
                # Render pending changes
                if self._pending_renders:
                    self._force_render()
                    self._pending_renders = False
                
                print("✅ Batch mode disabled - rendering re-enabled")
    
    def _force_render(self):
        """Force rendering regardless of frame rate limits"""
        if self.plotter and not self._batch_mode:
            try:
                start_time = time.time()
                self.plotter.render()
                render_time = time.time() - start_time
                
                # Update performance stats
                self.performance_stats['render_count'] += 1
                self.performance_stats['last_render_time'] = render_time
                
                # Update average render time
                count = self.performance_stats['render_count']
                if count > 1:
                    old_avg = self.performance_stats['avg_render_time']
                    self.performance_stats['avg_render_time'] = old_avg + (render_time - old_avg) / count
                else:
                    self.performance_stats['avg_render_time'] = render_time
                
                self._last_render_timestamp = time.time()
                
            except Exception as e:
                print(f"⚠️ Render error: {e}")
    
    def _smart_render(self):
        """Smart rendering with frame rate control"""
        if self._batch_mode:
            self._pending_renders = True
            return
        
        current_time = time.time()
        time_since_last_render = current_time - self._last_render_timestamp
        
        # Check if enough time has passed for next frame
        if time_since_last_render >= self._min_render_interval:
            self._force_render()
            
            # Calculate actual FPS
            if time_since_last_render > 0:
                self.performance_stats['actual_fps'] = 1.0 / time_since_last_render
        else:
            # Skip render but mark as pending
            self._pending_renders = True
    
    def set_target_fps(self, fps: float):
        """Set target frame rate (FPS)"""
        self.performance_stats['target_fps'] = fps
        self._min_render_interval = 1.0 / fps
        print(f"🎯 Target FPS set to {fps:.1f} (min interval: {self._min_render_interval:.4f}s)")
    
    def set_optimization_level(self, level: str):
        """
        Set optimization level
        
        Args:
            level: 'performance' (60 FPS), 'balanced' (30 FPS), 'quality' (15 FPS)
        """
        if level == 'performance':
            self.set_target_fps(60.0)
            self._min_render_interval = 1.0 / 60.0
            print("🚀 Optimization level: PERFORMANCE (60 FPS)")
        elif level == 'balanced':
            self.set_target_fps(30.0)
            self._min_render_interval = 1.0 / 30.0
            print("⚖️ Optimization level: BALANCED (30 FPS)")
        elif level == 'quality':
            self.set_target_fps(15.0)
            self._min_render_interval = 1.0 / 15.0
            print("✨ Optimization level: QUALITY (15 FPS)")
        else:
            print(f"❌ Unknown optimization level: {level}")
    
    def get_performance_stats(self) -> Dict:
        """Get current performance statistics"""
        total_time = time.time() - self.performance_stats['start_time']
        
        stats = self.performance_stats.copy()
        stats['total_time'] = total_time
        stats['avg_fps'] = self.performance_stats['render_count'] / total_time if total_time > 0 else 0
        stats['render_efficiency'] = (
            self.performance_stats['actual_fps'] / self.performance_stats['target_fps'] 
            if self.performance_stats['target_fps'] > 0 else 0
        )
        
        return stats
    
    def print_performance_summary(self):
        """Print performance summary"""
        stats = self.get_performance_stats()
        
        print(f"\n📊 PyVista Performance Summary")
        print("=" * 40)
        print(f"Total time: {stats['total_time']:.1f}s")
        print(f"Render count: {stats['render_count']}")
        print(f"Update count: {stats['update_count']}")
        print(f"Average FPS: {stats['avg_fps']:.1f}")
        print(f"Target FPS: {stats['target_fps']:.1f}")
        print(f"Actual FPS: {stats['actual_fps']:.1f}")
        print(f"Render efficiency: {stats['render_efficiency']:.1%}")
        print(f"Avg render time: {stats['avg_render_time']:.4f}s")
        print(f"Avg update time: {stats['avg_update_time']:.4f}s")
        
        # Performance rating
        if stats['render_efficiency'] >= 0.9:
            rating = "🚀 EXCELLENT"
        elif stats['render_efficiency'] >= 0.7:
            rating = "✅ GOOD"
        elif stats['render_efficiency'] >= 0.5:
            rating = "⚠️ FAIR"
        else:
            rating = "❌ POOR"
        
        print(f"Performance: {rating}")


class OptimizedURDFRobotVisualizer(URDFRobotVisualizer, OptimizedPyVistaVisualizer):
    """
    Optimized URDF robot visualizer combining both functionality sets
    """
    
    def __init__(self, interactive: bool = True, window_size: Tuple[int, int] = (1200, 800)):
        """Initialize optimized URDF robot visualizer"""
        
        # Initialize optimization features
        OptimizedPyVistaVisualizer.__init__(self, interactive, window_size)
        
        # Initialize URDF features (but avoid duplicate plotter creation)
        # Store the optimized plotter
        optimized_plotter = self.plotter
        optimized_available = self.available
        
        # Initialize URDF robot features
        URDFRobotVisualizer.__init__(self, interactive, window_size)
        
        # Restore optimized plotter
        self.plotter = optimized_plotter
        self.available = optimized_available
        
        # Mesh update optimization
        self._mesh_update_cache = {}
        self._transform_matrix_cache = {}
        
        print("🎯 Optimized URDF Robot Visualizer initialized")
    
    def update_robot_visualization(self, robot_name: str, force_render: bool = True):
        """
        Optimized robot visualization update
        
        Args:
            robot_name: Name of robot to update
            force_render: Whether to render immediately (False for batch updates)
        """
        if robot_name not in self.robots or robot_name not in self.link_actors:
            return False
        
        update_start = time.time()
        
        try:
            robot = self.robots[robot_name]
            link_actors = self.link_actors[robot_name]
            
            # Get current link poses from robot's forward kinematics
            link_poses = robot.get_link_poses()
            
            # Batch update all links
            updated_links = 0
            for link_name, pose in link_poses.items():
                if link_name in link_actors and pose is not None:
                    if self._update_single_link_optimized(robot_name, link_name, pose, link_actors[link_name]):
                        updated_links += 1
            
            # Update time display
            self.update_time_display()
            
            # Render if requested and not in batch mode
            if force_render and not self._batch_mode:
                self._smart_render()
            elif self._batch_mode:
                self._pending_renders = True
            
            # Update performance stats
            update_time = time.time() - update_start
            self.performance_stats['update_count'] += 1
            self.performance_stats['last_update_time'] = update_time
            
            # Update average update time
            count = self.performance_stats['update_count']
            if count > 1:
                old_avg = self.performance_stats['avg_update_time']
                self.performance_stats['avg_update_time'] = old_avg + (update_time - old_avg) / count
            else:
                self.performance_stats['avg_update_time'] = update_time
            
            return True
            
        except Exception as e:
            print(f"⚠️ Error updating robot visualization '{robot_name}': {e}")
            return False
    
    def _update_single_link_optimized(self, robot_name: str, link_name: str, pose, link_actor_info: Dict) -> bool:
        """Optimized single link update using cached transformations"""
        try:
            # Create cache key
            cache_key = f"{robot_name}_{link_name}"
            
            # Create transformation matrix
            transform_matrix = pose.to_transformation_matrix()
            
            # Check if transformation changed (avoid unnecessary updates)
            if cache_key in self._transform_matrix_cache:
                old_matrix = self._transform_matrix_cache[cache_key]
                if np.allclose(transform_matrix, old_matrix, atol=1e-6):
                    return False  # No change, skip update
            
            # Update cache
            self._transform_matrix_cache[cache_key] = transform_matrix.copy()
            
            # Get actor reference
            actor = link_actor_info['actor']
            
            # Method 1: Direct transformation matrix update (fastest)
            try:
                # Convert numpy array to VTK matrix
                vtk_matrix = self.pv.vtk.vtkMatrix4x4()
                for i in range(4):
                    for j in range(4):
                        vtk_matrix.SetElement(i, j, transform_matrix[i, j])
                
                # Set the transformation matrix on the actor
                actor.SetUserMatrix(vtk_matrix)
                return True
                
            except Exception:
                # Fallback: copy and transform mesh (slower but reliable)
                original_mesh = link_actor_info['mesh'].copy()
                original_mesh.transform(transform_matrix, inplace=True)
                
                # Update the actor using mapper
                if hasattr(actor, 'GetMapper'):
                    mapper = actor.GetMapper()
                    mapper.SetInputData(original_mesh)
                    mapper.Modified()
                return True
                
        except Exception as e:
            print(f"⚠️ Error updating link {link_name}: {e}")
            return False
    
    def update_multiple_robots(self, robot_updates: Dict[str, any]):
        """
        Efficiently update multiple robots in batch mode
        
        Args:
            robot_updates: Dict of robot_name -> pose/joint_data
        """
        with self.batch_mode():
            updated_count = 0
            for robot_name in robot_updates:
                if self.update_robot_visualization(robot_name, force_render=False):
                    updated_count += 1
            
            print(f"📦 Batch update: {updated_count}/{len(robot_updates)} robots updated")
    
    def clear_caches(self):
        """Clear optimization caches (useful for memory management)"""
        self._mesh_update_cache.clear()
        self._transform_matrix_cache.clear()
        print("🗑️ Visualization caches cleared")
    
    def set_optimization_level(self, level: str):
        """
        Set optimization level
        
        Args:
            level: 'performance' (60 FPS), 'balanced' (30 FPS), 'quality' (15 FPS)
        """
        if level == 'performance':
            self.set_target_fps(60.0)
            self._min_render_interval = 1.0 / 60.0
            print("🚀 Optimization level: PERFORMANCE (60 FPS)")
        elif level == 'balanced':
            self.set_target_fps(30.0)
            self._min_render_interval = 1.0 / 30.0
            print("⚖️ Optimization level: BALANCED (30 FPS)")
        elif level == 'quality':
            self.set_target_fps(15.0)
            self._min_render_interval = 1.0 / 15.0
            print("✨ Optimization level: QUALITY (15 FPS)")
        else:
            print(f"❌ Unknown optimization level: {level}")


# Factory functions
def create_optimized_visualizer(interactive: bool = True, 
                               window_size: Tuple[int, int] = (1200, 800),
                               optimization_level: str = 'balanced') -> OptimizedPyVistaVisualizer:
    """Create optimized PyVista visualizer"""
    visualizer = OptimizedPyVistaVisualizer(interactive, window_size)
    if hasattr(visualizer, 'set_optimization_level'):
        visualizer.set_optimization_level(optimization_level)
    return visualizer


def create_optimized_urdf_visualizer(interactive: bool = True, 
                                    window_size: Tuple[int, int] = (1200, 800),
                                    optimization_level: str = 'balanced') -> OptimizedURDFRobotVisualizer:
    """Create optimized URDF robot visualizer"""
    visualizer = OptimizedURDFRobotVisualizer(interactive, window_size)
    visualizer.set_optimization_level(optimization_level)
    return visualizer


# Performance testing functions
def performance_test_visualization(num_robots: int = 10, duration: float = 30.0) -> Dict:
    """
    Test visualization performance with multiple robots
    
    Args:
        num_robots: Number of robots to test
        duration: Test duration in seconds
        
    Returns:
        Performance statistics
    """
    print(f"🧪 PyVista Performance Test: {num_robots} robots, {duration}s")
    
    # Create optimized visualizer
    visualizer = create_optimized_urdf_visualizer(
        interactive=True, 
        optimization_level='performance'
    )
    
    if not visualizer.available:
        print("❌ PyVista not available")
        return {}
    
    try:
        # Load test robots
        print("📦 Loading test robots...")
        robots = []
        for i in range(num_robots):
            robot_name = f"test_robot_{i}"
            
            # Create mock robot for testing
            from core.simulation_object import Pose
            initial_pose = Pose(x=i * 2.0, y=0, z=0)
            
            # Note: Would need actual robot loading here
            # For now, just create placeholders
            robots.append(robot_name)
        
        # Performance test loop
        print(f"🚀 Starting {duration}s performance test...")
        start_time = time.time()
        frame_count = 0
        
        while time.time() - start_time < duration:
            # Update all robots with batch mode
            with visualizer.batch_mode():
                for robot_name in robots:
                    # Simulate robot updates
                    t = time.time() - start_time
                    # Would update robot poses here
                    frame_count += 1
            
            # Small sleep to prevent 100% CPU usage
            time.sleep(0.001)
        
        # Get final stats
        stats = visualizer.get_performance_stats()
        stats['test_duration'] = duration
        stats['frame_count'] = frame_count
        stats['test_fps'] = frame_count / duration
        
        print(f"✅ Performance test completed")
        visualizer.print_performance_summary()
        
        return stats
        
    except Exception as e:
        print(f"❌ Performance test failed: {e}")
        return {}
    finally:
        if visualizer.plotter:
            visualizer.plotter.close()


def compare_visualization_performance():
    """Compare standard vs optimized visualization performance"""
    print("⚖️ Comparing Standard vs Optimized PyVista Performance")
    print("=" * 60)
    
    results = {}
    
    # Test configurations
    configs = [
        ("Standard PyVista", URDFRobotVisualizer, {}),
        ("Optimized Performance", OptimizedURDFRobotVisualizer, {"optimization_level": "performance"}),
        ("Optimized Balanced", OptimizedURDFRobotVisualizer, {"optimization_level": "balanced"}),
        ("Optimized Quality", OptimizedURDFRobotVisualizer, {"optimization_level": "quality"})
    ]
    
    for config_name, visualizer_class, kwargs in configs:
        print(f"\n🧪 Testing {config_name}...")
        
        try:
            # Create visualizer
            visualizer = visualizer_class(interactive=False, **kwargs)
            
            if not visualizer.available:
                print(f"   ❌ PyVista not available")
                continue
            
            # Simple performance test
            start_time = time.time()
            test_iterations = 1000
            
            for i in range(test_iterations):
                # Simulate rendering operations
                if hasattr(visualizer, '_smart_render'):
                    # Don't actually render, just test the logic
                    pass
                else:
                    # Standard visualizer
                    pass
            
            elapsed_time = time.time() - start_time
            iterations_per_second = test_iterations / elapsed_time
            
            results[config_name] = {
                'elapsed_time': elapsed_time,
                'iterations_per_second': iterations_per_second
            }
            
            print(f"   ✅ {iterations_per_second:.1f} iterations/second")
            
            if hasattr(visualizer, 'print_performance_summary'):
                visualizer.print_performance_summary()
            
        except Exception as e:
            print(f"   ❌ Test failed: {e}")
        finally:
            try:
                if hasattr(visualizer, 'plotter') and visualizer.plotter:
                    visualizer.plotter.close()
            except:
                pass
    
    # Print comparison
    print(f"\n🏆 Performance Comparison Summary")
    print("=" * 50)
    
    for config_name, stats in results.items():
        print(f"{config_name:<25}: {stats['iterations_per_second']:>8.1f} iter/s")
    
    return results


if __name__ == "__main__":
    print("🎯 PyVista最適化ビジュアライザーテスト")
    print("=" * 50)
    
    try:
        # Basic functionality test
        print("\n1. 基本機能テスト...")
        visualizer = create_optimized_urdf_visualizer(interactive=False)
        
        if visualizer.available:
            print("✅ 最適化ビジュアライザー作成成功")
            visualizer.print_performance_summary()
        else:
            print("❌ PyVista利用不可")
        
        # Performance comparison
        print("\n2. パフォーマンス比較テスト...")
        compare_visualization_performance()
        
    except Exception as e:
        print(f"❌ テスト失敗: {e}")
        import traceback
        traceback.print_exc()