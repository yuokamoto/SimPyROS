# Monitor Test Files

This directory contains development and test files related to the SimPyROS monitor window implementation.

## Files

### data_monitor.py
- **Original PyBullet monitor** that was used as reference for implementing SimPyROS monitor
- Shows how the monitor concept was adapted from PyBullet to SimPyROS
- Uses ttk widgets and would cause X11 RENDER errors

### test_monitor.py
- **Full simulation test** with monitor enabled
- Tests monitor with actual robot control and headless simulation
- Used during monitor development and integration testing

### test_monitor_simple.py  
- **Configuration test** for monitor without creating actual windows
- Tests monitor data structures and configuration parameters
- Safe test that doesn't trigger X11 issues

### test_monitor_x11.py
- **X11 error debugging test** created specifically to isolate the X11 RENDER error
- Minimal monitor window creation for testing font and widget rendering
- Used to verify the fix for X11 RENDER BadLength errors

## Context

These files were created during the X11 RENDER error debugging session where:

1. **Problem**: Monitor window was causing X11 errors when using ttk widgets and fonts
2. **Solution**: Converted to basic tk widgets without font specifications  
3. **Testing**: These files helped isolate and verify the fix

The final working monitor is now at `core/simulation_monitor.py` with simplified widgets that avoid X11 RENDER issues.

## Usage

These files are preserved for reference but are not needed for normal SimPyROS usage. The production monitor is integrated into SimulationManager and works without errors.