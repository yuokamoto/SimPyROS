# SimPyROS Examples

Integrated demo collection for the SimPyROS simulation framework.

## 📁 File Structure (7 Files)

### 1. `basic_demo.py` - Basic Operations Learning
**Purpose**: Understanding fundamental SimPyROS features
**Contents**:
- Object creation, connection, and movement
- Bidirectional connection system testing
- Static constraint behavior verification
**Usage**: `python examples/basic_demo.py`
**Learning Focus**: Basic operations and connection systems

### 2. `visualization_demo.py` - 3D Visualization
**Purpose**: 3D display using matplotlib with controllable animation speed
**Contents**:
- Real-time 3D animation with precise timing control
- Four distinct motion phases (circular, figure-8, complex, approach)
- Trajectory display and coordinate frame visualization
- Adjustable animation speed via real-time factor
- Clean phase-based simulation architecture
**Usage**: 
- Normal speed: `python examples/visualization_demo.py`
- Static display: `python examples/visualization_demo.py static`
- Custom speed: `python examples/visualization_demo.py 0.5` (half speed)
- Fast mode: `python examples/visualization_demo.py 2.0` (double speed)
**Learning Focus**: 3D visualization system with temporal control
**Dependencies**: `pip install matplotlib`

### 3. `realtime_demo_simple.py` - Real-time Processing
**Purpose**: Lightweight real-time simulation
**Contents**:
- 3 robot types (Racer, Explorer, Guardian)
- Real-time data export (JSON)
- Different motion patterns
**Usage**: `python examples/realtime_demo_simple.py`
**Output**: `output/realtime_*.json`
**Learning Focus**: Data-focused real-time processing

### 4. `fixed_realtime_demo.py` - Advanced Real-time
**Purpose**: PyVista + fallback support
**Contents**:
- PyVista 3D rendering (when available)
- Automatic fallback (data-only mode)
- Screenshot generation
**Usage**: `python examples/fixed_realtime_demo.py`
**Output**: `output/fixed_realtime_*.json`, `output/realtime_render_*.png`
**Learning Focus**: Robust real-time systems

### 5. `pyvista_simple_demo.py` - High-quality 3D Rendering
**Purpose**: Professional 3D visualization using PyVista
**Contents**:
- High-quality 3D robot meshes
- Headless environment support
- Animation sequence generation
**Usage**: `python examples/pyvista_simple_demo.py`
**Output**: `output/safe_robot_render.png`, `output/robot_*.png`
**Learning Focus**: Professional 3D visualization
**Dependencies**: `pip install pyvista`

### 6. `simpy_rt_demo.py` - SimPy.rt Method
**Purpose**: SimPy built-in real-time functionality
**Contents**:
- simpy.rt.RealtimeEnvironment usage
- 3 demo patterns (circular/multi-robot/interactive)
- Automatic time scale adjustment
**Usage**: `python examples/simpy_rt_demo.py [1|2|3]`
**Learning Focus**: SimPy.rt features and limitations
**Note**: Platform dependent (see Appendix.md)

## 🚀 Recommended Learning Course

### Beginner Course (Essential)
1. **`basic_demo.py`** - Understand basic operations
2. **`visualization_demo.py static`** - Experience static 3D display
3. **`visualization_demo.py`** - Experience dynamic 3D display
4. **`realtime_demo_simple.py`** - Experience real-time processing

### Advanced Course (Optional)
5. **`pyvista_simple_demo.py`** - Experience high-quality 3D rendering
6. **`fixed_realtime_demo.py`** - Experience robust systems
7. **`simpy_rt_demo.py 1`** - Experience alternative real-time method

## 📊 Feature Comparison Table

| Demo | 3D Display | Real-time | Data Output | Difficulty | Use Case |
|------|------------|-----------|-------------|------------|----------|
| basic_demo | ❌ | ❌ | ❌ | ⭐ | Basic learning |
| visualization_demo | ✅ matplotlib | ✅ | ❌ | ⭐⭐ | 3D understanding |
| realtime_demo_simple | ❌ | ✅ | ✅ JSON | ⭐⭐ | Data processing |
| fixed_realtime_demo | ✅ PyVista* | ✅ | ✅ JSON+PNG | ⭐⭐⭐ | Professional development |
| pyvista_simple_demo | ✅ PyVista | ❌ | ✅ PNG | ⭐⭐⭐ | High-quality visualization |
| simpy_rt_demo | ❌ | ✅ SimPy.rt | ❌ | ⭐⭐ | Alternative method |

*Automatic fallback support

## 🛠 Required Dependencies

**Basic execution**:
```bash
pip install simpy scipy numpy
```

**3D visualization**:
```bash
pip install matplotlib
```

**High-quality 3D**:
```bash  
pip install pyvista
```

## 📁 Output Files

Each demo generates files in the `output/` folder:

**Data files**:
- `realtime_*.json` - Robot state snapshots
- `complete_trajectories.json` - Complete trajectory data

**Image files**:
- `frame_*.png` - matplotlib frames
- `realtime_render_*.png` - PyVista rendering  
- `safe_robot_render.png` - High-quality robot images

## ⚠️ Troubleshooting

**ImportError**:
- Requires `simulation_object.py` and `visualizer.py` in parent directory

**Display-related errors**:
- Automatically switches to PNG output mode in headless environments

**PyVista-related errors**:
- Automatically falls back to data-only mode on X11 errors

## 📝 Next Steps

After running demos, consider:

1. **Custom simulation development**: Create your own robot simulations based on the basic structure
2. **URDF model integration**: Add robot model file (URDF/SDF) loading capabilities  
3. **Physics engine integration**: Connect with physics engines like PyBullet
4. **ROS integration**: Execute simulations as ROS 2 nodes

For details, see `Appendix.md` in the parent directory.