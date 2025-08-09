# SimPyROS Examples

This directory contains comprehensive examples showcasing the SimPyROS robotics simulation framework with multiple visualization backends and approaches.

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

### 🎮 `pyvista/` - Interactive 3D Visualization ⭐**Recommended**
**High-quality interactive 3D demos using PyVista**

#### `pyvista_robot_demo.py` - Interactive Real-time Demo
```bash
python pyvista/pyvista_robot_demo.py 10    # 10-second interactive demo
```
**Features:**
- Real-time 3D window with interactive camera controls
- Figure-8 robot motion pattern with trajectory trails
- Live FPS and position display
- Mouse controls: Left-drag (rotate), Right-drag (zoom), Middle-drag (pan)

#### `pyvista_simple_demo.py` - Image Generation & Testing
```bash
python pyvista/pyvista_simple_demo.py
```
**Features:**
- Mesh creation and transformation testing
- Screenshot generation with off-screen rendering
- Animation frame sequence creation
- Headless environment support




## 🚀 Quick Start Guide

### 1. Absolute Beginner
```bash
python basic/basic_demo.py                # Learn fundamentals
```

### 2. Interactive 3D (Recommended)
```bash
python pyvista/pyvista_robot_demo.py 5    # Modern interactive 3D
```

### 3. View Generated Images
```bash
ls ../output/pyvista_*.png               # Check PyVista outputs
ls ../output/frame_*.png                 # Check matplotlib outputs
```

## 📊 Examples Comparison

| Category | Demo | 3D Display | Interactive | Real-time | Data Output | Difficulty |
|----------|------|------------|-------------|-----------|-------------|------------|
| **Basic** | basic_demo | ❌ | ❌ | ❌ | ❌ Text | ⭐ |
| **PyVista** | pyvista_robot_demo | ✅ VTK | ✅ Mouse | ✅ | ❌ | ⭐⭐ |
| **PyVista** | pyvista_simple_demo | ✅ VTK | ❌ | ❌ | ✅ PNG | ⭐⭐ |

## 🛠 Requirements

### Core Dependencies
```bash
pip install simpy scipy numpy
```

### Visualization Dependencies
```bash
# For PyVista examples (recommended)
pip install pyvista

# For matplotlib examples
pip install matplotlib
```

### Complete Installation
```bash
pip install pyvista matplotlib simpy scipy numpy
```

## 📁 Output Files

Generated files are saved to the `../output/` directory:

**PyVista outputs:**
- `pyvista_test.png` - Static robot rendering
- `pyvista_frame_*.png` - Animation sequences  
- `pyvista_robot_*.png` - Robot pose variations
- `realtime_render_*.png` - Real-time demo screenshots

**Data outputs:**
- `*.json` - Simulation data exports
- `complete_trajectories.json` - Full trajectory data

## 🎮 Interactive Controls

**PyVista demos (pyvista_robot_demo.py):**
- **Mouse Left Button + Drag:** Rotate camera
- **Mouse Right Button + Drag:** Zoom in/out  
- **Mouse Middle Button + Drag:** Pan camera
- **Close Window:** Exit demo

## ⚠️ Troubleshooting

### PyVista Issues
- **Window doesn't open:** Check DISPLAY variable, try `pyvista_simple_demo.py` first
- **Performance issues:** Reduce demo duration, check GPU drivers
- **Import errors:** `pip install pyvista`

### Matplotlib Issues
- **Slow animation:** Adjust real-time factor parameter
- **Display issues:** Works in most environments, fallback for PyVista

### General Issues
- **Import errors:** Ensure `simulation_object.py` is in parent directory
- **Path issues:** Run from SimPyROS root directory

## 🗂️ Legacy Code

Previous experimental code has been moved to the `legacy/` directory for reference.

## 📝 Learning Path

**Recommended progression:**
1. **`basic/basic_demo.py`** - Understand core concepts
2. **`pyvista/pyvista_simple_demo.py`** - Learn 3D basics
3. **`pyvista/pyvista_robot_demo.py`** - Experience interactive 3D

**Alternative/Legacy:**
- **`../legacy/examples/visualization_demo.py`** - Traditional matplotlib approach

## 🔧 Development Notes

### Adding New Examples
- Place in appropriate category folder
- Follow existing naming conventions
- Update this README.md
- Test in both GUI and headless environments

### Preferred Stack
- **Primary:** PyVista for interactive 3D visualization
- **Secondary:** Matplotlib for educational/simple cases
- **Fallback:** Text output for compatibility

---

**Focus:** Comprehensive examples covering basic concepts to advanced interactive 3D robotics simulation.