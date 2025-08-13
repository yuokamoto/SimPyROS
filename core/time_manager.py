#!/usr/bin/env python3
"""
SimPy RealtimeEnvironment-based Time Management System for SimPyROS

Provides unified time access across all components using SimPy's official
RealtimeEnvironment for reliable real-time synchronization.
"""

import simpy.rt
import time
import threading
from typing import Optional, Dict, Any
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
        self._real_time_factor = real_time_factor
        self._strict = strict
        
        # Handle special case: real_time_factor=0.0 means maximum speed (no real-time constraints)
        if real_time_factor == 0.0:
            # Use standard Environment for maximum speed (no real-time constraints)
            self.env = simpy.Environment()
            self._use_realtime = False
            simpy_factor = None  # Not applicable for standard Environment
        else:
            # Create RealtimeEnvironment for real-time synchronization
            # NOTE: SimPy RealtimeEnvironment factor is INVERSE of our real_time_factor
            # Our real_time_factor: 1.0=real time, 2.0=double speed, 0.5=half speed
            # SimPy factor: 1.0=real time, 0.5=double speed, 2.0=half speed
            safe_factor = max(0.001, real_time_factor)  # Prevent division by zero for non-zero values
            simpy_factor = 1.0 / safe_factor  # Convert to SimPy's factor
            self._use_realtime = True
            self.env = simpy.rt.RealtimeEnvironment(
                factor=simpy_factor,
                strict=self._strict
            )
        
        # Time tracking
        self._start_real_time = 0.0
        self._simulation_started = False
        
        # Thread safety for visualization access
        self._lock = threading.RLock()
        
        # Note: FrequencyController functionality moved to legacy/
        # Modern architecture uses independent SimPy processes with direct timing
        
        if self._use_realtime:
            print(f"⏰ TimeManager: RealtimeEnvironment initialized (factor={self._real_time_factor}x)")
        else:
            print(f"⏰ TimeManager: Standard Environment initialized (MAX SPEED mode)")
    
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
            # Update the RealtimeEnvironment factor (convert to SimPy's inverse factor)
            simpy_factor = 1.0 / factor
            self.env.factor = simpy_factor
            
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
    
    # Note: FrequencyController functionality moved to legacy/time_management/
    # Modern architecture uses independent SimPy processes with direct timing.
    
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
            
            # Note: No frequency controllers to clear in modern architecture
            
            print("🔄 TimeManager: Reset to initial state")


# Note: FrequencyController moved to legacy/time_management/frequency_controller.py
# Modern architecture uses independent SimPy processes with yield env.timeout()


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


# Note: Legacy helper functions moved to legacy/time_management/
# Modern approach: Define SimPy processes directly with their own timing