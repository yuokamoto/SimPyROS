# PyVista Examples ⭐ Recommended

High-quality interactive 3D visualization examples using PyVista (VTK-based). These examples focus on PyVista's visualization capabilities using built-in geometric robot models.

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

## 🎮 Interactive PyVista Demos

### 🎪 `pyvista_robot_demo.py` - Built-in Robot Showcase ⭐ **Recommended**
```bash
# Interactive 3D window with built-in wheeled robot
python pyvista_robot_demo.py 10

# Try different robot types
python pyvista_robot_demo.py 15 basic
python pyvista_robot_demo.py 20 quadcopter
python pyvista_robot_demo.py 12 humanoid

# Shorter demo
python pyvista_robot_demo.py 5 wheeled
```
**Features**: Built-in geometric robots, figure-8 movement, trajectory trails, interactive controls
**Robot Types**: wheeled (default), basic, quadcopter, humanoid

### 📸 `pyvista_simple_demo.py` - Image Generation & Testing
```bash
# Generate static images (works headless)
python pyvista_simple_demo.py
```
**Features**: Static image generation, mesh testing, headless compatibility

### ⚡ `performance_test.py` - Performance Benchmarking
```bash
# Test 3D rendering performance
python performance_test.py
```
**Features**: Frame rate testing, mesh complexity analysis, performance measurement

## 🎮 Interactive Controls

**When running interactive demos:**
- **Left-click + drag**: Rotate camera around scene
- **Right-click + drag**: Zoom in/out  
- **Middle-click + drag**: Pan camera view
- **Mouse wheel**: Quick zoom
- **'r'**: Reset camera to default view
- **'q' or close window**: Exit demo

## 🤖 Built-in Robot Types

### Available Robot Models
- **wheeled** (default): Mobile robot with wheels, base, arm, and gripper
- **basic**: Simple robot with base, arm, and end effector
- **quadcopter**: Drone with center body and 4 propeller arms
- **humanoid**: Simple humanoid with torso, head, arms, and legs

### Robot Creation Examples
```bash
# Test individual robot types
python -c "
from sample_robots import SampleRobotFactory
print('Available types:', SampleRobotFactory.get_available_types())
"
```

## 💡 Learning Path

### 🎓 **Progressive PyVista Learning**
1. `pyvista_simple_demo.py` - Understand PyVista basics
2. `pyvista_robot_demo.py wheeled` - Learn robot visualization
3. `pyvista_robot_demo.py basic` - Explore different robot types
4. `performance_test.py` - Understand rendering performance

### 🎯 **Feature-Specific Usage**
- **3D Visualization Basics**: `pyvista_simple_demo.py`
- **Interactive Robot Demo**: `pyvista_robot_demo.py`
- **Robot Type Comparison**: Try all robot types (wheeled, basic, quadcopter, humanoid)
- **Performance Analysis**: `performance_test.py`
- **Headless Operation**: `pyvista_simple_demo.py` (automatic)

### 🖥️ **Environment-Specific**
- **Interactive Development**: `pyvista_robot_demo.py`
- **Headless Servers**: `pyvista_simple_demo.py`
- **CI/Testing**: `performance_test.py`
- **Presentations**: Interactive demos with different robot types

## 🛠️ Technical Details

### Core Dependencies
```bash
# 3D visualization
pyvista>=0.40.0           # Main 3D visualization library
vtk>=9.0.0                # VTK backend for PyVista
numpy>=1.20.0             # Array operations
scipy>=1.7.0              # Spatial transformations

# Simulation framework
simpy>=4.0.0              # Discrete event simulation

# Sample robot geometries (built-in)
# No additional dependencies required for built-in robots
```

### System Requirements
- **Python 3.8+** (recommended: 3.9 or 3.10)
- **Display**: Required for interactive demos (X11 on Linux)
- **Memory**: 128MB+ for built-in robot models
- **Graphics**: OpenGL 2.0+ compatible graphics card

### Headless Environment Setup
```bash
# For servers without display
sudo apt-get install xvfb  # Ubuntu/Debian
export DISPLAY=:99
Xvfb :99 -screen 0 1024x768x24 &

# pyvista_simple_demo.py works headless automatically
python pyvista_simple_demo.py
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

**Sample robots not found:**
```bash
# Check sample_robots.py is in the right place
ls -la sample_robots.py
# Should be in examples/pyvista/sample_robots.py

# Test import
python -c "from sample_robots import SampleRobotFactory; print('Sample robots available')"
```

**Interactive window not opening:**
```bash
# Check display setup
echo $DISPLAY
export DISPLAY=:0  # On local machine
# OR use headless demos instead
```

## 📈 Performance Notes

- **Built-in robots**: Very fast loading (~10ms)
- **Interactive demos**: 30-60 FPS on modern hardware
- **Headless rendering**: Faster, no GUI overhead
- **Memory usage**: 50-200MB for typical geometric robots
- **Startup time**: ~1-2 seconds for PyVista initialization

## 🔗 Related Examples

- `../urdf/` - URDF robot loading and advanced joint control
- `../basic/basic_demo.py` - Learn SimPyROS fundamentals first  
- `../robot_demo.py` - Complete Robot class demonstration
- `../../tests/` - Unit tests and validation scripts

## 📋 Example Commands Summary

```bash
# Quick start - recommended first demo
python pyvista_robot_demo.py 10

# Try different robot types
python pyvista_robot_demo.py 15 basic
python pyvista_robot_demo.py 10 quadcopter
python pyvista_robot_demo.py 12 humanoid

# Test PyVista basics
python pyvista_simple_demo.py

# Performance analysis
python performance_test.py
```

## 🔄 Relationship with URDF Examples

**PyVista examples (this folder):**
- Focus on PyVista visualization features
- Use built-in geometric robot models
- Faster loading, simpler setup
- Good for learning 3D visualization

**URDF examples (`../urdf/`):**
- Focus on URDF robot file loading
- Use real robot description files
- More realistic robot models
- Good for learning robot systems

**Choose based on your goal:**
- **Learning PyVista**: Start here with built-in robots
- **Learning URDF**: Go to `../urdf/` for real robot files
- **Learning both**: Start here, then advance to URDF examples

---

**Best for:** Learning PyVista 3D visualization, testing different geometric robot types, and understanding interactive 3D controls without URDF complexity.