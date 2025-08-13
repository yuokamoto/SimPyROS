#!/usr/bin/env python3
"""
SimPy RealtimeEnvironment-based Time Management System for SimPyROS

Provides unified time access across all components using SimPy's official
RealtimeEnvironment for reliable real-time synchronization.
"""

import simpy.rt
import time
import threading
from typing import Optional, Dict, Any, Callable
from dataclasses import dataclass


@dataclass
class TimingStats:
    """Real-time synchronization statistics"""
    sim_time: float = 0.0
    real_time_elapsed: float = 0.0
    real_time_factor: float = 1.0
    target_speed: str = "1.0x"
    actual_speed: str = "N/A"


class TimeManager:
    """
    RealtimeEnvironment-based time management system for SimPyROS
    
    Features:
    - Uses SimPy's official RealtimeEnvironment for reliable real-time sync
    - Unified simulation time access across all components
    - Dynamic real-time factor control
    - Thread-safe access for visualization components
    - Simple frequency control helpers
    
    Usage:
        time_mgr = TimeManager(real_time_factor=1.0)
        env = time_mgr.env  # Use this environment for SimPy processes
        current_sim_time = time_mgr.get_sim_time()
        time_mgr.set_real_time_factor(2.0)  # 2x speed
    """
    
    def __init__(self, real_time_factor: float = 1.0, strict: bool = True):
        """
        Initialize RealtimeEnvironment-based time manager
        
        Args:
            real_time_factor: Real-time multiplier (1.0=real time, 0.5=half speed, 2.0=double speed)
            strict: If True, enforce strict real-time synchronization
        """
        self._real_time_factor = max(0.001, real_time_factor)  # Prevent division by zero
        self._strict = strict
        
        # Create RealtimeEnvironment
        self.env = simpy.rt.RealtimeEnvironment(
            factor=self._real_time_factor,
            strict=self._strict
        )
        
        # Time tracking
        self._start_real_time = 0.0
        self._simulation_started = False
        
        # Thread safety for visualization access
        self._lock = threading.RLock()
        
        # Frequency controllers for different update rates
        self._frequency_controllers: Dict[str, 'FrequencyController'] = {}
        
        print(f"⏰ TimeManager: RealtimeEnvironment initialized (factor={self._real_time_factor}x)")
    
    def start_simulation(self):
        """Mark simulation as started and initialize timing"""
        with self._lock:
            self._simulation_started = True
            self._start_real_time = time.time()
            print(f"🚀 TimeManager: Simulation started (real_time_factor={self._real_time_factor}x)")
    
    def get_sim_time(self) -> float:
        """Get current simulation time (thread-safe)"""
        with self._lock:
            return self.env.now
    
    def get_real_time_elapsed(self) -> float:
        """Get real time elapsed since simulation start"""
        with self._lock:
            if not self._simulation_started:
                return 0.0
            return time.time() - self._start_real_time
    
    def get_real_time_factor(self) -> float:
        """Get current real-time factor"""
        return self._real_time_factor
    
    def set_real_time_factor(self, factor: float):
        """
        Update real-time factor dynamically
        
        Args:
            factor: New real-time factor (0.1 to 10.0 recommended range)
        """
        old_factor = self._real_time_factor
        
        # Validate factor range
        if factor <= 0:
            factor = 0.001  # Minimum to prevent issues
        elif factor > 10.0:
            factor = 10.0  # Reasonable upper limit
            
        with self._lock:
            self._real_time_factor = factor
            # Update the RealtimeEnvironment factor
            self.env.factor = factor
            
        print(f"⏱️ TimeManager: Real-time factor updated: {old_factor:.2f}x → {factor:.2f}x")
    
    def get_timing_stats(self) -> TimingStats:
        """Get comprehensive timing statistics"""
        with self._lock:
            sim_time = self.get_sim_time()
            real_elapsed = self.get_real_time_elapsed()
            
            # Calculate actual speed ratio
            if real_elapsed > 0 and sim_time > 0:
                actual_ratio = sim_time / real_elapsed
                actual_speed = f"{actual_ratio:.1f}x"
            else:
                actual_speed = "N/A"
            
            return TimingStats(
                sim_time=sim_time,
                real_time_elapsed=real_elapsed,
                real_time_factor=self._real_time_factor,
                target_speed=f"{self._real_time_factor:.1f}x",
                actual_speed=actual_speed
            )
    
    def create_frequency_controller(self, name: str, frequency: float) -> 'FrequencyController':
        """
        Create a frequency controller for specific update rates
        
        Args:
            name: Unique name for this controller
            frequency: Target frequency in Hz
            
        Returns:
            FrequencyController instance
        """
        controller = FrequencyController(self, name, frequency)
        self._frequency_controllers[name] = controller
        return controller
    
    def remove_frequency_controller(self, name: str):
        """Remove a frequency controller"""
        if name in self._frequency_controllers:
            del self._frequency_controllers[name]
    
    def get_frequency_controller(self, name: str) -> Optional['FrequencyController']:
        """Get existing frequency controller by name"""
        return self._frequency_controllers.get(name)
    
    def reset(self):
        """Reset time manager state"""
        with self._lock:
            # Create new RealtimeEnvironment with same settings
            old_factor = self._real_time_factor
            self.env = simpy.rt.RealtimeEnvironment(
                factor=old_factor,
                strict=self._strict
            )
            self._simulation_started = False
            
            # Clear frequency controllers
            self._frequency_controllers.clear()
            
            print("🔄 TimeManager: Reset to initial state")


class FrequencyController:
    """
    Simple frequency controller using RealtimeEnvironment timing
    
    Provides easy frequency control without complex custom timing logic.
    """
    
    def __init__(self, time_manager: TimeManager, name: str, frequency: float):
        """
        Initialize frequency controller
        
        Args:
            time_manager: Reference to central time manager
            name: Unique name for this controller  
            frequency: Target frequency in Hz
        """
        self.time_manager = time_manager
        self.name = name
        self.frequency = frequency
        self.interval = 1.0 / frequency if frequency > 0 else 0.1
        self._last_update_sim_time = 0.0
    
    def should_update(self) -> bool:
        """
        Check if it's time for the next update based on simulation time
        
        Returns:
            True if update should occur, False otherwise
        """
        current_sim_time = self.time_manager.get_sim_time()
        return current_sim_time >= self._last_update_sim_time + self.interval
    
    def mark_updated(self):
        """Mark that an update has occurred"""
        self._last_update_sim_time = self.time_manager.get_sim_time()
    
    def update_if_needed(self, update_func: Callable[[], None]) -> bool:
        """
        Execute update function if frequency timer indicates it's time
        
        Args:
            update_func: Function to call for update (should take no arguments)
            
        Returns:
            True if update was performed, False otherwise
        """
        if self.should_update():
            try:
                update_func()
                self.mark_updated()
                return True
            except Exception as e:
                print(f"⚠️ FrequencyController '{self.name}' update error: {e}")
                return False
        return False
    
    def set_frequency(self, frequency: float):
        """Update the frequency"""
        if frequency > 0:
            self.frequency = frequency
            self.interval = 1.0 / frequency
            print(f"⏱️ FrequencyController '{self.name}': Updated to {frequency} Hz")
    
    def get_frequency(self) -> float:
        """Get current frequency"""
        return self.frequency
    
    def reset(self):
        """Reset frequency controller timing"""
        self._last_update_sim_time = 0.0


# Global time manager instance for singleton access
_global_time_manager: Optional[TimeManager] = None


def set_global_time_manager(time_manager: TimeManager):
    """Set the global time manager instance"""
    global _global_time_manager
    _global_time_manager = time_manager


def get_global_time_manager() -> Optional[TimeManager]:
    """Get the global time manager instance"""
    return _global_time_manager


def get_sim_time() -> float:
    """Convenience function to get simulation time from global manager"""
    time_mgr = get_global_time_manager()
    if time_mgr:
        return time_mgr.get_sim_time()
    return 0.0


def get_real_time_factor() -> float:
    """Convenience function to get real-time factor from global manager"""
    time_mgr = get_global_time_manager()
    if time_mgr:
        return time_mgr.get_real_time_factor()
    return 1.0


def create_process_with_frequency(time_manager: TimeManager, 
                                 process_func: Callable,
                                 frequency: float,
                                 name: str = "process") -> Any:
    """
    Convenience function to create a SimPy process with specific frequency
    
    Args:
        time_manager: TimeManager instance
        process_func: Process function (should be a generator)
        frequency: Target frequency in Hz
        name: Process name for debugging
        
    Returns:
        SimPy process instance
    """
    def frequency_controlled_process():
        interval = 1.0 / frequency
        while True:
            try:
                # Execute one step of the process
                next(process_func)
            except StopIteration:
                break
            except Exception as e:
                print(f"⚠️ Process '{name}' error: {e}")
            
            # Wait for next interval
            yield time_manager.env.timeout(interval)
    
    return time_manager.env.process(frequency_controlled_process())