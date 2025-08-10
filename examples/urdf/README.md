# URDF Robot Examples 🤖

Focused demonstrations of URDF robot loading, parsing, and visualization. These examples showcase SimPyROS's URDF support with joint control, real-time visualization, and advanced robot manipulation.

## 🎯 Quick Start

**Prerequisites**: Make sure you have activated your virtual environment and installed dependencies:
```bash
# Activate your virtual environment first
source simpyros-env/bin/activate  # Linux/macOS
# OR: conda activate simpyros

# Install dependencies (if not done yet)
pip install -r ../../requirements.txt
```

**Test URDF support:**
```bash
python -c "from urdf_loader import URDFLoader; loader = URDFLoader(); print('URDF support available:', loader.is_available())"
```

## 🤖 URDF Robot Demos

### 🎬 `urdf_robot_demo.py` - Complete URDF Visualization ⭐ **Recommended**
```bash
# Load and visualize URDF robots with material colors
# When run from SimPyROS root directory:
python examples/urdf/urdf_robot_demo.py 15 examples/robots/simple_robot.urdf
python examples/urdf/urdf_robot_demo.py 10 examples/robots/mobile_robot.urdf

# When run from examples/urdf/ directory:
python urdf_robot_demo.py 10 ../robots/simple_robot.urdf
python urdf_robot_demo.py 15 ../robots/mobile_robot.urdf

# Headless mode for servers/CI
python urdf_robot_demo.py 5 ../robots/simple_robot.urdf --headless

# Headless with screenshot capture
python urdf_robot_demo.py 10 ../robots/mobile_robot.urdf --headless --screenshots
```
**Features**: URDF parsing, individual link colors, material support, interactive 3D controls

### 🔧 `simple_joint_demo.py` - Joint Motion Basics
```bash
# Visual demonstration of joint movement
python simple_joint_demo.py
```
**Features**: Simple geometric shapes, clear joint motion, educational focus

### ⚡ `realtime_joint_demo.py` - Advanced Joint Control
```bash
# Real-time joint animation with URDF robots
python realtime_joint_demo.py 20

# Shorter demo
python realtime_joint_demo.py 10
```
**Features**: URDF-based joint control, real-time link pose updates, multi-phase motion patterns

### 🎯 `joint_motion_demo.py` - Structured Joint Patterns
```bash
# Demonstrate various joint motion patterns
python joint_motion_demo.py 15

# Extended demo
python joint_motion_demo.py 25
```
**Features**: Three-phase motion patterns, joint angle monitoring, systematic movement

### 🏗️ `robot_visualization_demo.py` - Complete Robot System
```bash
# Full robot class integration
python robot_visualization_demo.py 15

# With specific URDF robot
python robot_visualization_demo.py 20 ../robots/movable_robot.urdf
```
**Features**: Robot class integration, hierarchical joint control, complete robot system demo

## 🎮 Interactive Controls

**When running interactive demos:**
- **Left-click + drag**: Rotate camera around robot
- **Right-click + drag**: Zoom in/out  
- **Middle-click + drag**: Pan camera view
- **Mouse wheel**: Quick zoom
- **'r'**: Reset camera to default view
- **'q' or close window**: Exit demo

## 💡 Learning Path

### 🎓 **Progressive URDF Learning**
1. `simple_joint_demo.py` - Understand basic joint concepts
2. `urdf_robot_demo.py` - Learn URDF loading and visualization
3. `joint_motion_demo.py` - Master structured joint control
4. `realtime_joint_demo.py` - Advanced real-time manipulation
5. `robot_visualization_demo.py` - Complete robot system integration

### 🎯 **Specific Use Cases**
- **URDF File Testing**: `urdf_robot_demo.py`
- **Joint Control Learning**: `simple_joint_demo.py` → `joint_motion_demo.py`
- **Real-time Applications**: `realtime_joint_demo.py`
- **Robot Integration**: `robot_visualization_demo.py`
- **Headless Automation**: All demos with `--headless` flag

## 🛠️ Technical Requirements

### URDF Dependencies
```bash
# Essential for URDF support
yourdfpy>=0.0.6           # Modern URDF parsing (primary)
trimesh>=3.15.0           # 3D mesh processing

# 3D Visualization
pyvista>=0.40.0           # Interactive 3D rendering
vtk>=9.0.0                # VTK backend

# Scientific computing
numpy>=1.20.0             # Array operations
scipy>=1.7.0              # Spatial transformations (rotations)

# Simulation framework
simpy>=4.0.0              # Discrete event simulation
```

### Available URDF Test Robots
```bash
# Located in examples/robots/ directory
../robots/simple_robot.urdf      # Basic 3-link robot with colors
../robots/mobile_robot.urdf      # Mobile base with arm and camera
../robots/rotation_test.urdf     # Multi-color rotation testing
../robots/movable_robot.urdf     # Advanced movable joints
```

### System Requirements
- **Python 3.8+** (recommended: Python 3.9 or 3.10)
- **URDF Parser**: yourdfpy (automatic fallback to legacy parsers)
- **Display**: Required for interactive modes (X11 on Linux)
- **Memory**: 256MB+ for typical URDF robots, 1GB+ for complex meshes
- **Graphics**: OpenGL 2.0+ for 3D rendering

## 🚨 URDF-Specific Troubleshooting

**URDF loading fails:**
```bash
# Check URDF parser availability
python -c "from urdf_loader import URDFLoader; loader = URDFLoader(); print('Available:', loader.is_available(), 'Library:', loader.get_library_name())"

# Install primary URDF parser
pip install yourdfpy trimesh --upgrade

# Check specific URDF file
python -c "
from urdf_loader import URDFLoader
loader = URDFLoader()
success = loader.load_urdf('../robots/simple_robot.urdf')
print('URDF loaded successfully:', success)
if success: loader.print_info()
"
```

**"name 'adv_link' is not defined" error:**
```bash
# This was fixed in recent versions, update your code:
git pull  # Get latest fixes
```

**Joint motion not visible:**
```bash
# Check robot has movable joints
python -c "
from robot import create_robot_from_urdf
import simpy
env = simpy.Environment()
robot = create_robot_from_urdf(env, '../robots/movable_robot.urdf', 'test')
print('Movable joints:', [name for name in robot.get_joint_names() if robot.joints[name].joint_type.value != 'fixed'])
"
```

**Missing robot files:**
```bash
# Check robot files exist
ls -la ../robots/
# Should show: simple_robot.urdf, mobile_robot.urdf, rotation_test.urdf, movable_robot.urdf
```

## 🔍 URDF File Analysis

Use the built-in URDF analysis function:
```bash
python -c "
from urdf_loader import URDFLoader
loader = URDFLoader()
if loader.load_urdf('../robots/simple_robot.urdf'):
    loader.print_info()
    print('Links:', len(loader.links))
    print('Joints:', len(loader.joints))
    print('Meshes:', len(loader.get_mesh_files()))
"
```

## 📈 Performance Tips

- **Start with simple_robot.urdf** for learning (fastest loading)
- **Use mobile_robot.urdf** for feature demonstrations
- **Interactive mode**: 30-60 FPS on modern hardware
- **Headless mode**: Faster rendering, good for automation
- **Large URDF files**: May require 1GB+ RAM for complex meshes

## 🔗 Related Examples

- `../pyvista/` - PyVista-specific 3D visualization examples
- `../basic/basic_demo.py` - Learn SimPyROS basics first
- `../robot_demo.py` - Robot class fundamentals
- `../../tests/` - URDF loading validation tests

## 📋 Example Commands Summary

```bash
# Quick start with recommended demo
python urdf_robot_demo.py 10 ../robots/simple_robot.urdf

# Learn joint control
python simple_joint_demo.py

# Advanced real-time control
python realtime_joint_demo.py 15

# Headless automation
python urdf_robot_demo.py 5 ../robots/mobile_robot.urdf --headless --screenshots

# Complete robot system
python robot_visualization_demo.py 15
```

---

**Best for:** URDF robot file loading, joint motion control, robot kinematics visualization, and real-time robot manipulation.