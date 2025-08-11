# Legacy Code - SimPyROS

This directory contains legacy code that was replaced by the new SimulationManager-based architecture. These files are preserved for reference and historical purposes.

## Directory Structure

### `examples/` - Original Example Files
- `realtime_demo.py` - Original matplotlib-based visualization demo
- `visualization_demo.py` - Matplotlib 3D animation with speed control
- `realtime_demo_simple.py` - Lightweight real-time data processing

### `urdf_old/` - Legacy URDF Processing Examples **[Moved from examples/urdf/]**
**Original URDF examples replaced by `examples/simple/` architecture**
- `joint_demo.py` - Manual joint control implementation
- `urdf_robot_demo.py` - Original URDF robot visualization
- `simple_joint_demo.py` - Basic joint movement demo
- `realtime_joint_demo.py` - Advanced joint control (manual implementation)
- `robot_visualization_demo.py` - Complete robot system demo

### `basic_old/` - Foundation Examples **[Moved from examples/basic/]**
**Basic concepts replaced by simplified SimulationManager examples**
- `basic_demo.py` - Original SimPyROS foundation demo

### Moved Files from `examples/`
- `robot_demo.py` - Original robot class demonstration  
- `demo_without_scipy.py` - Temporary scipy version conflict workaround
- `visualization_demos.py` - Separated visualization functions
- `simpy_rt_demo.py` - SimPy.rt alternative implementation

### `backends/` - Alternative Visualization Backends
- `matplotlib_backend/` - Matplotlib-based visualization system
- `open3d_backend/` - Open3D visualization experiments  
- `plotly_backend/` - Web-based Plotly visualization attempts

### `tests/` - Deprecated Test Files
- `test_visualization_static.py` - Matplotlib-based static visualization tests

## Why These Were Moved

**Main reasons for migration to PyVista:**
1. **Interactive 3D:** Real-time window manipulation with mouse controls
2. **Professional Quality:** VTK-based high-quality rendering
3. **Better Performance:** GPU acceleration and optimized rendering pipeline
4. **Headless Support:** Robust off-screen rendering for automated environments

**Legacy limitations:**
- **Matplotlib:** Static plots, limited interactivity, slower animation
- **Open3D:** Complex API, dependency issues, limited robot mesh support
- **Plotly:** Web-based overhead, limited offline functionality

## Usage Notes

These files are kept for:
- **Reference purposes** - Understanding evolution of the visualization system
- **Fallback compatibility** - In case PyVista is unavailable in some environments
- **Educational value** - Comparing different 3D visualization approaches

## Running Legacy Examples

If needed, legacy examples can still be run:

```bash
# From the main SimPyROS directory:
python legacy/examples/visualization_demo.py
python legacy/examples/basic_demo.py
```

**Dependencies for legacy examples:**
```bash
pip install matplotlib  # For matplotlib-based demos
```

## Migration Notes

**From matplotlib to PyVista:**
- Static plots → Interactive 3D windows
- 2D/3D matplotlib → VTK mesh rendering
- Manual frame updates → Real-time camera controls

**Code patterns that changed:**
- `plt.show()` → `plotter.show()`
- `ax.plot()` → `plotter.add_mesh()`
- `plt.pause()` → `plotter.update()`

---

**Recommendation:** Use the current PyVista-based examples in the main `examples/` directory for new development.