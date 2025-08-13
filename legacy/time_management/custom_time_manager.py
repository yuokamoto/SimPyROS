#!/usr/bin/env python3
"""
Legacy Custom Time Management System - DEPRECATED

This was the original centralized time management implementation before 
migrating to simpy.rt.RealtimeEnvironment. Preserved for reference.

DO NOT USE - Use core/time_manager.py with RealtimeEnvironment instead.
"""

import time
import simpy
from typing import Optional, Dict, Any, List
from dataclasses import dataclass
import threading
import warnings

# Deprecated - kept for reference only
warnings.warn("legacy.time_management.custom_time_manager is deprecated. Use core.time_manager with RealtimeEnvironment instead.", 
              DeprecationWarning, stacklevel=2)


@dataclass
class LegacyTimingStats:
    """Legacy timing statistics structure"""
    target_real_dt: float = 0.0
    actual_avg_dt: float = 0.0
    accuracy_ratio: float = 0.0
    avg_processing_time: float = 0.0
    avg_sleep_time: float = 0.0
    samples: int = 0


class LegacyTimeManager:
    """
    DEPRECATED: Original custom time management system
    
    This implementation has been replaced with simpy.rt.RealtimeEnvironment
    for better reliability and standard compliance.
    """
    
    def __init__(self, env: simpy.Environment, 
                 real_time_factor: float = 1.0, 
                 time_step: float = 0.01):
        warnings.warn("LegacyTimeManager is deprecated. Use RealtimeEnvironment-based TimeManager instead.", 
                     DeprecationWarning, stacklevel=2)
        
        self.env = env
        self._real_time_factor = real_time_factor
        self._time_step = time_step
        
        # Simulation time state
        self._sim_time = 0.0
        self._real_time_start = 0.0
        self._simulation_started = False
        
        # Performance tracking
        self._timing_stats = {
            'target_real_time': 0.0,
            'actual_real_time': 0.0,
            'processing_times': [],
            'sleep_times': []
        }
        
        # Thread safety for visualization access
        self._lock = threading.RLock()
    
    def advance_sim_time(self) -> float:
        """Legacy method - advance simulation time manually"""
        with self._lock:
            self._sim_time += self._time_step
            return self._sim_time
    
    def calculate_real_time_delay(self, processing_time: float) -> float:
        """Legacy real-time delay calculation with processing time compensation"""
        if self._real_time_factor == 0.0:
            return 0.0001
        
        target_real_dt = self._time_step / self._real_time_factor
        remaining_time = target_real_dt - processing_time
        
        # Store timing statistics
        self._timing_stats['processing_times'].append(processing_time)
        if len(self._timing_stats['processing_times']) > 100:
            self._timing_stats['processing_times'].pop(0)
        
        if remaining_time > 0:
            self._timing_stats['sleep_times'].append(remaining_time)
            return remaining_time
        else:
            self._timing_stats['sleep_times'].append(0.0)
            return 0.0001


class LegacyFrequencyController:
    """
    DEPRECATED: Original frequency controller implementation
    
    This has been replaced with simpler frequency control using RealtimeEnvironment.
    """
    
    def __init__(self, time_manager: LegacyTimeManager, name: str, frequency: float):
        warnings.warn("LegacyFrequencyController is deprecated.", DeprecationWarning, stacklevel=2)
        
        self.time_manager = time_manager
        self.name = name
        self.frequency = frequency
        self.interval = 1.0 / frequency if frequency > 0 else 0.1
        self._last_update_sim_time = 0.0
    
    def should_update(self) -> bool:
        """Legacy frequency check logic"""
        current_sim_time = self.time_manager.get_sim_time()
        return current_sim_time >= self._last_update_sim_time + self.interval
    
    def mark_updated(self):
        """Mark that an update has occurred"""
        self._last_update_sim_time = self.time_manager.get_sim_time()