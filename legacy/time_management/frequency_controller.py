#!/usr/bin/env python3
"""
Legacy FrequencyController - DEPRECATED

This FrequencyController implementation was part of the original time management
system before migrating to SimPy's independent process architecture.

DO NOT USE - Modern SimPy processes handle their own timing with yield statements.
"""

import warnings
from typing import Callable, Optional

# Deprecated - kept for reference only
warnings.warn("legacy.time_management.frequency_controller is deprecated. "
              "Use independent SimPy processes with yield env.timeout() instead.", 
              DeprecationWarning, stacklevel=2)


class LegacyFrequencyController:
    """
    DEPRECATED: Legacy frequency controller implementation
    
    This has been replaced with direct SimPy process timing in the new architecture.
    Each robot subsystem now manages its own timing with yield env.timeout().
    """
    
    def __init__(self, time_manager, name: str, frequency: float):
        warnings.warn("LegacyFrequencyController is deprecated. "
                     "Use SimPy processes with yield env.timeout() instead.", 
                     DeprecationWarning, stacklevel=2)
        
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
    
    def update_if_needed(self, update_func: Callable[[], None]) -> bool:
        """Legacy update logic"""
        if self.should_update():
            try:
                update_func()
                self.mark_updated()
                return True
            except Exception as e:
                print(f"⚠️ LegacyFrequencyController '{self.name}' update error: {e}")
                return False
        return False
    
    def set_frequency(self, frequency: float):
        """Update the frequency"""
        if frequency > 0:
            self.frequency = frequency
            self.interval = 1.0 / frequency
            print(f"⏱️ LegacyFrequencyController '{self.name}': Updated to {frequency} Hz")
    
    def get_frequency(self) -> float:
        """Get current frequency"""
        return self.frequency
    
    def reset(self):
        """Reset frequency controller timing"""
        self._last_update_sim_time = 0.0


# Migration guide
MIGRATION_GUIDE = """
Migration from LegacyFrequencyController to SimPy Processes:

OLD (FrequencyController):
    controller = time_manager.create_frequency_controller("joints", 100.0)
    controller.update_if_needed(update_joints)

NEW (SimPy Process):
    def joint_control_process():
        while active:
            update_joints()
            yield env.timeout(1.0 / 100.0)  # 100 Hz
    
    env.process(joint_control_process())

Benefits of new approach:
- Natural SimPy event-driven timing
- No centralized frequency management needed
- Better resource usage
- Clearer code structure
"""