# Debug Scripts

This directory contains debugging scripts used during development of the URDF robot movement system.

## Files

### `debug_movement_test.py`
- **Purpose**: Test basic robot movement with simple cube mesh
- **Usage**: `python debug_movement_test.py`
- **Features**: Tests PyVista actor transformation without URDF complexity

### `debug_urdf_movement.py` 
- **Purpose**: Debug URDF robot movement with individual link actors
- **Usage**: `python debug_urdf_movement.py`
- **Features**: 
  - Tests VTK matrix transformations
  - Verifies individual link positioning
  - Debugging coordinate system issues

## Development Context

These scripts were created to solve the robot movement issues discovered in August 2025:
- **Problem**: URDF robots with individual colored links weren't moving properly
- **Root Cause**: VTK matrix access issues and incorrect relative positioning
- **Solution**: Fixed in `pyvista_visualizer.py` individual mesh actor handling

## Current Status

The movement issues have been resolved. These scripts remain for:
- Historical reference
- Future debugging of similar issues
- Understanding the VTK/PyVista actor transformation system

## Usage Notes

To run these scripts:
1. Copy to main project directory
2. Run with `python script_name.py`
3. They use the same dependencies as the main project