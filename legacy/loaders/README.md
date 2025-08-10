# Legacy URDF Loaders

This directory contains older URDF loading implementations that have been replaced by `advanced_urdf_loader.py`.

## Files

### `urdf_loader.py` - Original URDF Loader
- **Status**: Deprecated
- **Dependencies**: urdfpy, trimesh, pycollada
- **Issues**: Heavy dependencies, networkx compatibility problems
- **Replaced by**: `advanced_urdf_loader.py`

### `simple_urdf_loader.py` - Lightweight XML Parser
- **Status**: Deprecated 
- **Dependencies**: Standard library only
- **Features**: Basic geometric primitives parsing
- **Limitations**: No mesh file support, no material colors
- **Replaced by**: `advanced_urdf_loader.py` with fallback mode

## Current Implementation

The active URDF loading system is now in:
- `advanced_urdf_loader.py` (main project root)
- Uses yourdfpy as primary parser with multiple fallbacks
- Supports material colors, coordinate transformations, and mesh files
- Handles individual link coloring for PyVista visualization

## Migration Notes

If you need to use these legacy loaders:
1. Copy the desired file back to the main project directory
2. Install the required dependencies (see each file's imports)
3. Update import statements in your code

**Recommended**: Use the current `advanced_urdf_loader.py` system instead.