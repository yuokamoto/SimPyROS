# Legacy Root-Level Test Files

This directory contains test files that were previously in the project root directory. These were moved here during the SimPy architecture refactoring to clean up the project structure.

## Files

- `test_centralized_time.py` - Tests for centralized time management (replaced by TimeManager)
- `test_centralized_updates.py` - Tests for centralized update loops (replaced by independent processes)
- `test_sim_time_based_updates.py` - Tests for sim_time based updates
- `test_timeout_dt_optimization.py` - Tests for timing optimization
- `test_timing_accuracy.py` - Tests for timing accuracy
- `test_unified_time_updates.py` - Tests for unified time updates
- `test_yield_necessity.py` - Tests for yield necessity in SimPy

## Status

These tests are now **OUTDATED** due to the architectural changes:
- Single-loop centralized management → Independent SimPy processes
- Custom time management → simpy.rt.RealtimeEnvironment
- Manual timing control → SimPy automatic scheduling

New tests should be created for the current architecture focusing on:
- RealtimeEnvironment behavior
- Independent process coordination
- Robot subsystem interactions
- Event-driven behaviors