# PyVista Examples ⭐ Recommended

High-quality interactive 3D visualization examples using PyVista (VTK-based). These examples demonstrate SimPyROS's advanced 3D visualization capabilities with real-time robot simulation and URDF support.

## 🎯 Quick Start

**Prerequisites**: Make sure you have activated your virtual environment and installed dependencies:
```bash
# Activate your virtual environment first
source simpyros-env/bin/activate  # Linux/macOS
# OR: conda activate simpyros

# Install dependencies (if not done yet)
pip install -r ../../requirements.txt
```

**Test PyVista availability:**
```bash
python -c "import pyvista; print('PyVista available:', pyvista.__version__)"
```

## 🎮 Interactive Demos (Recommended Entry Point)

### 🤖 `urdf_robot_demo.py` - Advanced URDF Robot Visualization
```bash
# Load and visualize URDF robots with material colors
# When run from SimPyROS root directory:
python examples/pyvista/urdf_robot_demo.py 10 examples/robots/mobile_robot.urdf
python examples/pyvista/urdf_robot_demo.py 15 examples/robots/simple_robot.urdf

# When run from examples/pyvista/ directory:
python urdf_robot_demo.py 10 ../../examples/robots/mobile_robot.urdf
python urdf_robot_demo.py 15 ../../examples/robots/simple_robot.urdf

# Interactive mode with mouse controls
python urdf_robot_demo.py 10

# Headless mode for servers/CI
python urdf_robot_demo.py 5 --headless

# Headless with screenshot capture
python urdf_robot_demo.py 10 --headless --screenshots
```
**Features**: URDF loading, individual link coloring, real-time movement, interactive 3D controls

### 🎪 `pyvista_robot_demo.py` - Built-in Robot Demo  
```bash
# Interactive 3D window with built-in wheeled robot
python pyvista_robot_demo.py 10

# Load custom URDF
python pyvista_robot_demo.py 15 ../../examples/robots/rotation_test.urdf
```
**Features**: Built-in robot meshes, figure-8 movement pattern, trajectory trails

### 📸 `pyvista_simple_demo.py` - Image Generation & Testing
```bash
# Generate static images (works headless)
python pyvista_simple_demo.py
```
**Features**: Static image generation, mesh testing, headless compatibility

## 🦾 Advanced Robot Control Demos

### 🎯 `simple_joint_demo.py` - Joint Motion Visualization (Recommended)
```bash
# Clearest joint motion demonstration
python simple_joint_demo.py
```
**Features**: Visible geometric shapes, staged joint motion, clear visual feedback

### ⚡ `realtime_joint_demo.py` - Real-time Joint Control
```bash
# Advanced real-time joint animation
python realtime_joint_demo.py 15
```
**Features**: URDF-based joint control, real-time link pose updates, multiple motion phases

### 🔧 `joint_motion_demo.py` - Joint Motion Patterns
```bash
# Structured joint motion demonstration  
python joint_motion_demo.py 15
```
**Features**: Three-phase motion patterns, joint angle monitoring, visual joint feedback

### 🤖 `robot_visualization_demo.py` - Complete Robot System
```bash
# Full robot class demonstration
python robot_visualization_demo.py 15

# With custom URDF robot
python robot_visualization_demo.py 20 ../../examples/robots/movable_robot.urdf
```
**Features**: Robot class integration, joint-level control, hierarchical movement

## 🔬 Performance & Analysis

### ⚡ `performance_test.py` - Performance Benchmarking
```bash
# Test 3D rendering performance
python performance_test.py
```
**Features**: Frame rate testing, mesh complexity analysis, headless performance measurement

## 🎮 Interactive Controls

**When running interactive demos:**
- **Left-click + drag**: Rotate camera
- **Right-click + drag**: Zoom in/out  
- **Middle-click + drag**: Pan camera
- **Mouse wheel**: Quick zoom
- **'r'**: Reset camera view
- **'q' or close window**: Exit

## 💡 Usage Recommendations

### 🎓 **Learning Path (Progressive Difficulty)**
1. `pyvista_simple_demo.py` - Understand 3D basics
2. `simple_joint_demo.py` - Learn joint motion concepts  
3. `urdf_robot_demo.py` - Experience URDF robot loading
4. `realtime_joint_demo.py` - Advanced real-time control

### 🎯 **Feature-Specific Usage**
- **URDF Robot Loading**: `urdf_robot_demo.py`
- **Joint Motion Learning**: `simple_joint_demo.py`  
- **Performance Testing**: `performance_test.py`
- **Headless Operation**: `pyvista_simple_demo.py` + `--headless` options
- **Screenshot Generation**: Any demo with `--screenshots`

### 🖥️ **Environment-Specific**
- **Interactive Development**: `urdf_robot_demo.py`, `pyvista_robot_demo.py`
- **Headless Servers**: `pyvista_simple_demo.py`, `--headless` options
- **CI/Testing**: `performance_test.py`, headless modes
- **Presentations**: Interactive demos with built-in robots

## 🛠️ Technical Details

### Dependencies
```bash
# Core 3D visualization
pyvista>=0.40.0           # Main 3D visualization
vtk>=9.0.0                # VTK backend
numpy>=1.20.0             # Array operations
scipy>=1.7.0              # Spatial transformations

# URDF robot support  
yourdfpy>=0.0.6           # Modern URDF parsing
trimesh>=3.15.0           # 3D mesh processing

# Simulation framework
simpy>=4.0.0              # Discrete event simulation
```

### System Requirements
- **Python 3.8+** (recommended: 3.9 or 3.10)
- **Display**: Required for interactive demos (X11 on Linux)
- **Memory**: 512MB+ for complex URDF robots
- **Graphics**: OpenGL 2.0+ compatible graphics card

### Headless Environment Setup
```bash
# For servers without display
sudo apt-get install xvfb  # Ubuntu/Debian
export DISPLAY=:99
Xvfb :99 -screen 0 1024x768x24 &

# Then run demos with --headless flag
python urdf_robot_demo.py 5 --headless --screenshots
```

## 🚨 Troubleshooting

**PyVista not available:**
```bash
pip install --upgrade pip setuptools wheel
pip install pyvista --no-cache-dir
# OR: conda install -c conda-forge pyvista
```

**VTK rendering issues:**
```bash
# Force software rendering
export VTK_SILENCE_GET_VOID_POINTER_WARNINGS=1
export MESA_GL_VERSION_OVERRIDE=3.3
```

**URDF loading fails:**
```bash
# Check yourdfpy installation
python -c "import yourdfpy; print('yourdfpy available')"

# Reinstall if needed
pip install yourdfpy trimesh --upgrade
```

**Headless mode issues:**
```bash
# Install headless support
pip install pyvista[headless]

# Verify display setup
echo $DISPLAY
```

## 📈 Performance Notes

- **Interactive demos**: ~30-60 FPS on modern hardware
- **Headless rendering**: Faster, suitable for batch processing
- **URDF robots**: Performance depends on mesh complexity
- **Memory usage**: 100-500MB for typical robot models

## 🔗 Related Examples

- `../basic/basic_demo.py` - Learn SimPyROS fundamentals first
- `../robot_demo.py` - Complete Robot class demonstration
- `../../tests/` - Unit tests and validation scripts

---

**Best for:** Professional-quality 3D robotics visualization with real-time interaction and URDF robot support.