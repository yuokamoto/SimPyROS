"""
SimPyROS Core Module

This module provides the core functionality for the SimPyROS discrete event robotics simulation framework.

Organized into functional modules:
- visualization: PyVista-based 3D visualization
- robotics: Robot, sensor, and navigation components  
- monitoring: Simulation monitoring and performance tracking
- utils: Common utilities and helpers
"""

# Core framework components (remain in root)
from .simulation_manager import SimulationManager, SimulationConfig
from .simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose, Velocity
from .time_manager import TimeManager, TimingStats, set_global_time_manager, get_global_time_manager

# Import all submodule functionality for backward compatibility
from .visualization import *
from .robotics import *
from .monitoring import *
from .utils import *

# Explicitly expose main classes at core level for convenience
__all__ = [
    # Core framework
    'SimulationManager', 'SimulationConfig',
    'SimulationObject', 'ObjectParameters', 'ObjectType', 'Pose', 'Velocity', 
    'TimeManager', 'TimingStats', 'set_global_time_manager', 'get_global_time_manager',
    
    # Visualization
    'PyVistaVisualizer', 'URDFRobotVisualizer', 'PyVistaCoreHandler', 'RobotMeshCreator',
    'ProcessManager', 'ProcessURDFAdapter',
    'BaseRobotVisualizer',
    
    # Robotics
    'Robot', 'RobotParameters', 'URDFLoader',
    'SensorBase', 'LidarSensor', 'CameraSensor', 'IMUSensor', 'SensorManager',
    'RobotLinkConnector', 'Navigation',
    
    # Monitoring
    'SimulationMonitor', 'ProcessSeparatedMonitor',
    
    # Utils  
    'ExternalMeshManager', 'MultiprocessingCleaner', 'cleanup_multiprocessing_resources'
]