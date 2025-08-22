#!/usr/bin/env python3
"""
Sensor Module - Pluggable Sensor Implementation

This module provides base Sensor classes and concrete implementations 
that can be integrated into SimulationObject.

Features:
- Pluggable sensor strategies
- Multiple sensor support
- LIDAR sensor
- Camera sensor
- IMU sensor
- Extensible architecture
"""

import math
import numpy as np
from typing import List, Optional, Dict, Any, Generator
from abc import ABC, abstractmethod
from dataclasses import dataclass
import simpy

# Relative imports
from .simulation_object import Pose, Velocity
from .logger import get_logger, log_info, log_error, log_debug, log_warning


@dataclass
class SensorData:
    """Base class for sensor data"""
    timestamp: float
    sensor_name: str
    sensor_type: str
    data: Any
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary format"""
        return {
            'timestamp': self.timestamp,
            'sensor_name': self.sensor_name,
            'sensor_type': self.sensor_type,
            'data': self.data
        }


@dataclass
class LidarData(SensorData):
    """LIDAR sensor data"""
    ranges: np.ndarray  # Distance data
    angles: np.ndarray  # Angle data
    max_range: float
    min_range: float
    
    def __post_init__(self):
        self.sensor_type = "LIDAR"
        self.data = {
            'ranges': self.ranges,
            'angles': self.angles,
            'max_range': self.max_range,
            'min_range': self.min_range
        }


@dataclass 
class CameraData(SensorData):
    """Camera sensor data"""
    width: int
    height: int
    image_data: np.ndarray  # Image data (simplified for simulation)
    
    def __post_init__(self):
        self.sensor_type = "Camera"
        self.data = {
            'width': self.width,
            'height': self.height,
            'image_data': self.image_data
        }


@dataclass
class IMUData(SensorData):
    """IMU sensor data"""
    linear_acceleration: np.ndarray  # Linear acceleration [x, y, z]
    angular_velocity: np.ndarray     # Angular velocity [x, y, z]
    orientation: np.ndarray          # Orientation [x, y, z, w] quaternion
    
    def __post_init__(self):
        self.sensor_type = "IMU"
        self.data = {
            'linear_acceleration': self.linear_acceleration,
            'angular_velocity': self.angular_velocity,
            'orientation': self.orientation
        }


class SensorBase(ABC):
    """Abstract base class for sensors"""
    
    def __init__(self, 
                 name: str,
                 frequency: float = 10.0,
                 enabled: bool = True):
        """
        Initialize sensor
        
        Args:
            name: Sensor name
            frequency: Update frequency (Hz)
            enabled: Sensor enable flag
        """
        self.name = name
        self.frequency = frequency
        self.enabled = enabled
        
        # Sensor state
        self.last_update_time = 0.0
        self.data_callback = None
        
        # Logging
        self.logger = get_logger(f'simpyros.sensor.{self.name}')
        
    def set_data_callback(self, callback):
        """Set sensor data callback"""
        self.data_callback = callback
        
    def should_update(self, current_time: float) -> bool:
        """Check if update is needed"""
        if not self.enabled:
            return False
        update_interval = 1.0 / self.frequency
        return (current_time - self.last_update_time) >= update_interval
    
    def update(self, current_time: float, object_pose: Pose, object_velocity: Velocity) -> Optional[SensorData]:
        """
        Update sensor
        
        Args:
            current_time: Current time
            object_pose: Object pose
            object_velocity: Object velocity
            
        Returns:
            Sensor data (None if no update)
        """
        if not self.should_update(current_time):
            return None
            
        try:
            # Specific sensor processing
            sensor_data = self._process_sensor_data(current_time, object_pose, object_velocity)
            self.last_update_time = current_time
            
            # Call callback
            if self.data_callback and sensor_data:
                self.data_callback(sensor_data)
                
            return sensor_data
            
        except Exception as e:
            log_error(self.logger, f"Sensor update error: {e}")
            return None
    
    @abstractmethod
    def _process_sensor_data(self, current_time: float, object_pose: Pose, object_velocity: Velocity) -> SensorData:
        """Specific sensor data processing (implemented by subclass)"""
        pass
    
    @abstractmethod
    def get_sensor_type(self) -> str:
        """Get sensor type name"""
        pass
    
    def enable(self):
        """Enable sensor"""
        self.enabled = True
        log_info(self.logger, f"Sensor '{self.name}' enabled")
        
    def disable(self):
        """Disable sensor"""
        self.enabled = False
        log_info(self.logger, f"Sensor '{self.name}' disabled")
        
    def get_status(self) -> Dict[str, Any]:
        """Get sensor status"""
        return {
            'name': self.name,
            'type': self.get_sensor_type(),
            'frequency': self.frequency,
            'enabled': self.enabled,
            'last_update': self.last_update_time
        }


class LidarSensor(SensorBase):
    """LIDAR sensor implementation"""
    
    def __init__(self, 
                 name: str,
                 frequency: float = 20.0,
                 max_range: float = 10.0,
                 min_range: float = 0.1,
                 angle_min: float = -math.pi,
                 angle_max: float = math.pi,
                 angle_increment: float = math.pi / 180.0):  # 1 degree increment
        """
        Initialize LIDAR
        
        Args:
            name: Sensor name
            frequency: Update frequency (Hz)
            max_range: Maximum detection range (m)
            min_range: Minimum detection range (m)
            angle_min: Minimum angle (rad)
            angle_max: Maximum angle (rad)
            angle_increment: Angle increment (rad)
        """
        super().__init__(name, frequency)
        self.max_range = max_range
        self.min_range = min_range
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        
        # Generate angle array
        self.angles = np.arange(angle_min, angle_max + angle_increment, angle_increment)
        
    def _process_sensor_data(self, current_time: float, object_pose: Pose, object_velocity: Velocity) -> LidarData:
        """LIDAR data processing (simulation)"""
        
        # Generate random obstacle data for simulation
        num_rays = len(self.angles)
        ranges = np.full(num_rays, self.max_range)
        
        # Place some random obstacles
        for _ in range(np.random.randint(0, 5)):
            # Place obstacle at random angle and distance
            obstacle_angle_idx = np.random.randint(0, num_rays)
            obstacle_range = np.random.uniform(self.min_range, self.max_range)
            
            # Affect nearby measurement points as well
            for i in range(max(0, obstacle_angle_idx - 2), min(num_rays, obstacle_angle_idx + 3)):
                ranges[i] = min(ranges[i], obstacle_range + np.random.normal(0, 0.1))
        
        # Clip to min/max range
        ranges = np.clip(ranges, self.min_range, self.max_range)
        
        return LidarData(
            timestamp=current_time,
            sensor_name=self.name,
            sensor_type="LIDAR",
            data=None,  # Set by __post_init__
            ranges=ranges,
            angles=self.angles,
            max_range=self.max_range,
            min_range=self.min_range
        )
    
    def get_sensor_type(self) -> str:
        return "LIDAR"


class CameraSensor(SensorBase):
    """Camera sensor implementation"""
    
    def __init__(self, 
                 name: str,
                 frequency: float = 30.0,
                 width: int = 640,
                 height: int = 480,
                 fov: float = math.pi / 3):  # 60 degree field of view
        """
        Initialize camera
        
        Args:
            name: Sensor name
            frequency: Update frequency (Hz)
            width: Image width
            height: Image height
            fov: Field of view (rad)
        """
        super().__init__(name, frequency)
        self.width = width
        self.height = height
        self.fov = fov
        
    def _process_sensor_data(self, current_time: float, object_pose: Pose, object_velocity: Velocity) -> CameraData:
        """Camera data processing (simulation)"""
        
        # Generate simple image data for simulation
        # In actual implementation, 3D environment rendering would be performed
        image_data = np.random.randint(0, 255, (self.height, self.width, 3), dtype=np.uint8)
        
        # Add simple feature in center (example: cross mark)
        center_x, center_y = self.width // 2, self.height // 2
        image_data[center_y-5:center_y+5, center_x-20:center_x+20] = [255, 0, 0]  # Red horizontal line
        image_data[center_y-20:center_y+20, center_x-5:center_x+5] = [0, 255, 0]  # Green vertical line
        
        return CameraData(
            timestamp=current_time,
            sensor_name=self.name,
            sensor_type="Camera",
            data=None,  # Set by __post_init__
            width=self.width,
            height=self.height,
            image_data=image_data
        )
    
    def get_sensor_type(self) -> str:
        return "Camera"


class IMUSensor(SensorBase):
    """IMU sensor implementation"""
    
    def __init__(self, 
                 name: str,
                 frequency: float = 100.0,
                 noise_level: float = 0.01):
        """
        Initialize IMU
        
        Args:
            name: Sensor name
            frequency: Update frequency (Hz)
            noise_level: Noise level
        """
        super().__init__(name, frequency)
        self.noise_level = noise_level
        self.prev_velocity = None
        self.prev_time = 0.0
        
    def _process_sensor_data(self, current_time: float, object_pose: Pose, object_velocity: Velocity) -> IMUData:
        """IMU data processing"""
        
        # Calculate linear acceleration (from velocity change)
        if self.prev_velocity is not None and current_time > self.prev_time:
            dt = current_time - self.prev_time
            linear_accel = (object_velocity.linear - self.prev_velocity.linear) / dt
        else:
            linear_accel = np.zeros(3)
        
        # Add noise
        linear_accel += np.random.normal(0, self.noise_level, 3)
        angular_vel = object_velocity.angular + np.random.normal(0, self.noise_level, 3)
        
        # Orientation (quaternion)
        orientation = object_pose.rotation.as_quat()
        
        # Save previous values
        self.prev_velocity = object_velocity
        self.prev_time = current_time
        
        return IMUData(
            timestamp=current_time,
            sensor_name=self.name,
            sensor_type="IMU",
            data=None,  # Set by __post_init__
            linear_acceleration=linear_accel,
            angular_velocity=angular_vel,
            orientation=orientation
        )
    
    def get_sensor_type(self) -> str:
        return "IMU"


class SensorManager:
    """
    Manager class for multiple sensors
    
    Integrated into SimulationObject to manage multiple sensors comprehensively.
    """
    
    def __init__(self, env: simpy.Environment):
        """
        Initialize sensor manager
        
        Args:
            env: SimPy environment
        """
        self.env = env
        self.sensors: Dict[str, SensorBase] = {}
        self.sensor_data_history: Dict[str, List[SensorData]] = {}
        self.max_history_length = 100  # Maximum length of data history
        
        # Callbacks
        self.data_callbacks = []
        
        # Logging
        self.logger = get_logger('simpyros.sensor_manager')
        
    def add_sensor(self, sensor: SensorBase) -> bool:
        """
        Add sensor
        
        Args:
            sensor: Sensor to add
            
        Returns:
            Success flag
        """
        if sensor.name in self.sensors:
            log_warning(self.logger, f"Sensor '{sensor.name}' already exists")
            return False
            
        self.sensors[sensor.name] = sensor
        self.sensor_data_history[sensor.name] = []
        
        # データコールバック設定
        sensor.set_data_callback(self._on_sensor_data)
        
        log_info(self.logger, f"Added sensor '{sensor.name}' ({sensor.get_sensor_type()})")
        return True
    
    def remove_sensor(self, sensor_name: str) -> bool:
        """
        Remove sensor
        
        Args:
            sensor_name: Name of sensor to remove
            
        Returns:
            Success flag
        """
        if sensor_name not in self.sensors:
            log_warning(self.logger, f"Sensor '{sensor_name}' not found")
            return False
            
        del self.sensors[sensor_name]
        del self.sensor_data_history[sensor_name]
        
        log_info(self.logger, f"Removed sensor '{sensor_name}'")
        return True
    
    def update_all_sensors(self, current_time: float, object_pose: Pose, object_velocity: Velocity):
        """
        Update all sensors
        
        Args:
            current_time: Current time
            object_pose: Object pose
            object_velocity: Object velocity
        """
        for sensor in self.sensors.values():
            sensor_data = sensor.update(current_time, object_pose, object_velocity)
            if sensor_data:
                # データ履歴に追加
                history = self.sensor_data_history[sensor.name]
                history.append(sensor_data)
                
                # 履歴長制限
                if len(history) > self.max_history_length:
                    history.pop(0)
    
    def _on_sensor_data(self, sensor_data: SensorData):
        """Callback when sensor data is received"""
        # 全ての登録されたコールバックに通知
        for callback in self.data_callbacks:
            try:
                callback(sensor_data)
            except Exception as e:
                log_error(self.logger, f"Sensor data callback error: {e}")
    
    def add_data_callback(self, callback):
        """Add sensor data callback"""
        self.data_callbacks.append(callback)
        
    def get_sensor_by_name(self, name: str) -> Optional[SensorBase]:
        """Get sensor by name"""
        return self.sensors.get(name)
    
    def get_sensors_by_type(self, sensor_type: str) -> List[SensorBase]:
        """Get sensors by type"""
        return [sensor for sensor in self.sensors.values() 
                if sensor.get_sensor_type() == sensor_type]
    
    def get_latest_data(self, sensor_name: str) -> Optional[SensorData]:
        """Get latest sensor data"""
        if sensor_name in self.sensor_data_history:
            history = self.sensor_data_history[sensor_name]
            return history[-1] if history else None
        return None
    
    def get_data_history(self, sensor_name: str, count: int = None) -> List[SensorData]:
        """Get sensor data history"""
        if sensor_name in self.sensor_data_history:
            history = self.sensor_data_history[sensor_name]
            return history[-count:] if count else history
        return []
    
    def get_status(self) -> Dict[str, Any]:
        """Get sensor manager status"""
        sensor_statuses = {}
        for name, sensor in self.sensors.items():
            sensor_statuses[name] = sensor.get_status()
            
        return {
            'sensor_count': len(self.sensors),
            'sensors': sensor_statuses,
            'data_history_lengths': {name: len(history) 
                                   for name, history in self.sensor_data_history.items()}
        }
    
    def enable_all_sensors(self):
        """Enable all sensors"""
        for sensor in self.sensors.values():
            sensor.enable()
        log_info(self.logger, "All sensors enabled")
    
    def disable_all_sensors(self):
        """Disable all sensors"""
        for sensor in self.sensors.values():
            sensor.disable()
        log_info(self.logger, "All sensors disabled")


# Factory functions
def create_lidar_sensor(name: str = "lidar", 
                       frequency: float = 20.0,
                       max_range: float = 10.0) -> LidarSensor:
    """Create LIDAR sensor"""
    return LidarSensor(name=name, frequency=frequency, max_range=max_range)


def create_camera_sensor(name: str = "camera",
                        frequency: float = 30.0,
                        width: int = 640,
                        height: int = 480) -> CameraSensor:
    """Create camera sensor"""
    return CameraSensor(name=name, frequency=frequency, width=width, height=height)


def create_imu_sensor(name: str = "imu",
                     frequency: float = 100.0,
                     noise_level: float = 0.01) -> IMUSensor:
    """Create IMU sensor"""
    return IMUSensor(name=name, frequency=frequency, noise_level=noise_level)