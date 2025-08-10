# SimPyROS Examples

This directory contains comprehensive examples showcasing the SimPyROS robotics simulation framework, organized by functionality and complexity level.

## 📁 Directory Structure

### 🎯 `basic/` - Foundation Examples
**Start here for learning SimPyROS fundamentals**
```bash
python basic/basic_demo.py
```
**Features:**
- Object creation, connection, and movement
- Bidirectional connection system testing  
- Static constraint behavior verification
- No 3D visualization (text output only)

### 🎮 `pyvista/` - 3D Visualization Examples ⭐**Recommended**
**PyVista-based interactive 3D visualization using built-in geometric robots**

#### `pyvista_robot_demo.py` - Interactive Robot Showcase
```bash
python pyvista/pyvista_robot_demo.py 10                    # Wheeled robot (default)
python pyvista/pyvista_robot_demo.py 15 basic              # Basic robot
python pyvista/pyvista_robot_demo.py 20 quadcopter         # Quadcopter
python pyvista/pyvista_robot_demo.py 12 humanoid           # Humanoid robot
```
**Features:**
- **🤖 Built-in Robots**: Multiple geometric robot types (wheeled, basic, quadcopter, humanoid)
- Real-time 3D window with interactive camera controls
- Figure-8 motion patterns with trajectory trails
- Live FPS and position display
- Mouse controls: Left-drag (rotate), Right-drag (zoom), Middle-drag (pan)

#### `pyvista_simple_demo.py` - PyVista Testing & Image Generation
```bash
python pyvista/pyvista_simple_demo.py
```
**Features:**
- PyVista mesh creation and transformation testing
- Screenshot generation with off-screen rendering
- Headless environment support

#### `performance_test.py` - Performance Benchmarking
```bash
python pyvista/performance_test.py
```
**Features:**
- Frame rate testing and performance analysis
- Mesh complexity evaluation
- Rendering optimization insights

### 🤖 `urdf/` - URDF Robot Examples ⭐**Advanced**
**URDF robot file loading, parsing, and joint control demonstrations**

#### `urdf_robot_demo.py` - Complete URDF Visualization
```bash
# Interactive mode with URDF robots  
python urdf/urdf_robot_demo.py 15 robots/simple_robot.urdf     # Simple 3-link arm
python urdf/urdf_robot_demo.py 10 robots/mobile_robot.urdf     # Wheeled robot
python urdf/urdf_robot_demo.py 10 robots/rotation_test.urdf    # Multi-color test

# Headless execution modes
python urdf/urdf_robot_demo.py 5 robots/simple_robot.urdf --headless
python urdf/urdf_robot_demo.py 10 robots/mobile_robot.urdf --headless --screenshots
```
**Features:**
- **🤖 Full URDF Support**: Load robots with material colors
- **⚙️ Individual Link Coloring**: URDF-specified colors per link
- **🎮 Interactive & Headless Modes**: GUI or automated execution
- **📸 Screenshot Control**: Optional screenshot capture

#### `simple_joint_demo.py` - Joint Motion Basics
```bash
python urdf/simple_joint_demo.py
```
**Features:**
- Visual joint movement demonstration
- Educational focus with simple geometric shapes

#### `realtime_joint_demo.py` - Advanced Joint Control
```bash
python urdf/realtime_joint_demo.py 20
```
**Features:**
- URDF-based real-time joint animation
- Multi-phase motion patterns
- Live joint state monitoring

#### `joint_motion_demo.py` - Structured Joint Patterns
```bash
python urdf/joint_motion_demo.py 15
```
**Features:**
- Systematic joint motion patterns
- Joint angle monitoring
- Three-phase motion demonstration

#### `robot_visualization_demo.py` - Complete Robot System
```bash
python urdf/robot_visualization_demo.py 15
python urdf/robot_visualization_demo.py 20 robots/movable_robot.urdf
```
**Features:**
- Full Robot class integration
- Hierarchical joint control
- Complete robot system demonstration

### 🏗️ `robot_demo.py` - Robot Class Fundamentals
```bash
python robot_demo.py
```
**Features:**
- Core Robot class usage patterns
- Joint control and state management
- Foundation for advanced robot programming

### 🤖 `robots/` - URDF Robot Models
**Collection of test robot description files**
- `simple_robot.urdf` - Basic 3-link robot with colors
- `mobile_robot.urdf` - Mobile base with arm and camera
- `rotation_test.urdf` - Multi-color rotation testing
- `movable_robot.urdf` - Advanced movable joints

## 🚀 Quick Start Guide

### 1. Absolute Beginner - Learn Core Concepts
```bash
python basic/basic_demo.py                # SimPyROS fundamentals
```

### 2. 3D Visualization - PyVista Features  
```bash
python pyvista/pyvista_robot_demo.py 10   # Built-in geometric robots
```

### 3. Real Robots - URDF System
```bash
python urdf/urdf_robot_demo.py 15 robots/simple_robot.urdf  # Real robot files
```

### 4. Advanced - Joint Control
```bash
python urdf/realtime_joint_demo.py 20     # Real-time joint manipulation
```

## 📊 Examples Comparison

| Category | Demo | 3D Display | Interactive | URDF Support | Joint Control | Difficulty |
|----------|------|------------|-------------|--------------|---------------|------------|
| **Basic** | basic_demo | ❌ | ❌ | ❌ | ❌ | ⭐ |
| **PyVista** | pyvista_robot_demo | ✅ | ✅ | ❌ | ❌ | ⭐⭐ |
| **PyVista** | pyvista_simple_demo | ✅ | ❌ | ❌ | ❌ | ⭐⭐ |
| **URDF** | urdf_robot_demo | ✅ | ✅ | ✅ | ❌ | ⭐⭐⭐ |
| **URDF** | simple_joint_demo | ✅ | ✅ | ❌ | ✅ | ⭐⭐ |
| **URDF** | realtime_joint_demo | ✅ | ✅ | ✅ | ✅ | ⭐⭐⭐⭐ |

## 🛠 Requirements

### Core Dependencies (All Examples)
```bash
pip install simpy scipy numpy
```

### 3D Visualization (PyVista + URDF Examples)
```bash
pip install pyvista vtk
```

### URDF Robot Support (URDF Examples)
```bash
pip install yourdfpy trimesh
```

### Complete Installation
```bash
pip install -r ../requirements.txt
```

### Optional Legacy Support
```bash
pip install -r ../legacy/requirements-legacy.txt
```

## 💡 Learning Path

### 🎓 **Recommended Progressive Learning**
1. **`basic/basic_demo.py`** - Understand SimPyROS core concepts
2. **`pyvista/pyvista_simple_demo.py`** - Learn 3D visualization basics  
3. **`pyvista/pyvista_robot_demo.py`** - Experience interactive 3D with built-in robots
4. **`urdf/urdf_robot_demo.py`** - Load real robot models from URDF files
5. **`urdf/simple_joint_demo.py`** - Learn joint control concepts
6. **`urdf/realtime_joint_demo.py`** - Master advanced real-time joint control

### 🎯 **Goal-Oriented Paths**

**Learning 3D Visualization:**
- `pyvista/pyvista_simple_demo.py` → `pyvista/pyvista_robot_demo.py` → `pyvista/performance_test.py`

**Learning URDF Robots:**
- `urdf/urdf_robot_demo.py` → `urdf/simple_joint_demo.py` → `urdf/joint_motion_demo.py`

**Learning Real-time Control:**
- `urdf/simple_joint_demo.py` → `urdf/realtime_joint_demo.py` → `urdf/robot_visualization_demo.py`

## 🎮 Interactive Controls

**PyVista demos (all pyvista/ and urdf/ examples):**
- **Mouse Left Button + Drag:** Rotate camera
- **Mouse Right Button + Drag:** Zoom in/out  
- **Mouse Middle Button + Drag:** Pan camera
- **Mouse Wheel:** Quick zoom
- **'r' Key:** Reset camera view
- **'q' Key or Close Window:** Exit demo

## 📁 Output Files

Generated files are saved to the `../output/` directory:

**PyVista outputs:**
- `pyvista_test.png` - Static robot rendering
- `pyvista_frame_*.png` - Animation sequences  
- `pyvista_robot_*.png` - Robot pose variations
- `pyvista_wheeled_*.png`, `pyvista_quadcopter_*.png` - Robot type screenshots

**URDF outputs:**
- `urdf_demo_*.png` - URDF robot screenshots
- `realtime_joint_demo.png` - Joint motion capture

## ⚠️ Troubleshooting

### PyVista Issues
- **Window doesn't open:** Check DISPLAY variable, try `pyvista_simple_demo.py` first
- **Performance issues:** Reduce demo duration, check GPU drivers
- **Import errors:** `pip install pyvista vtk`

### URDF Issues  
- **URDF loading fails:** `pip install yourdfpy trimesh`, check robot file paths
- **Joint motion not visible:** Use `movable_robot.urdf`, check joint types
- **"adv_link not defined":** Update to latest version (this bug was fixed)

### General Issues
- **Import errors:** Ensure you're in the SimPyROS root directory
- **Path issues:** Use relative paths from project root
- **Dependencies:** Install requirements: `pip install -r requirements.txt`

## 📋 Example Command Reference

```bash
# Quick start commands
python basic/basic_demo.py
python pyvista/pyvista_robot_demo.py 10
python urdf/urdf_robot_demo.py 15 robots/simple_robot.urdf

# Try different robot types (PyVista)
python pyvista/pyvista_robot_demo.py 15 basic
python pyvista/pyvista_robot_demo.py 10 quadcopter
python pyvista/pyvista_robot_demo.py 12 humanoid

# Try different URDF robots
python urdf/urdf_robot_demo.py 10 robots/mobile_robot.urdf
python urdf/urdf_robot_demo.py 15 robots/rotation_test.urdf

# Advanced joint control
python urdf/simple_joint_demo.py
python urdf/realtime_joint_demo.py 20
python urdf/robot_visualization_demo.py 15

# Headless/automation
python urdf/urdf_robot_demo.py 5 robots/simple_robot.urdf --headless --screenshots
```

## 🔗 Related Resources

- **Main Documentation:** `../README.md` - Project overview and installation
- **Development History:** `../docs/CLAUDE.md` - Complete development log
- **Legacy Examples:** `../legacy/examples/` - Previous implementations
- **Testing:** `../tests/` - Unit tests and validation scripts

---

**Focus:** Comprehensive examples covering basic concepts to advanced interactive 3D robotics simulation with real URDF robot support.