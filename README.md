# SimPyROS - Python Robot Simulation Framework

A comprehensive robotics simulation framework built on SimPy with interactive 3D visualization powered by PyVista.

## 🚀 Quick Start

### Installation
```bash
git clone <repository-url>
cd SimPyROS
pip install -r requirements.txt
```

### Run Your First Simulation
```bash
# Learn the basics
python examples/basic/basic_demo.py

# Experience interactive 3D
python examples/pyvista/pyvista_robot_demo.py 5
```

## ✨ Features

- **🎮 Interactive 3D Visualization**: Real-time robot visualization with mouse controls
- **🔧 Flexible Object System**: Bidirectional connections, static constraints, dynamic relationships
- **⚡ Real-time Simulation**: SimPy-based discrete event simulation with real-time capabilities
- **🎯 Educational Focus**: Progressive learning path from basic concepts to advanced 3D interaction
- **📊 Multiple Backends**: PyVista (primary), with matplotlib legacy support
- **🖼️ Headless Support**: Generate images and data without GUI requirements

## 📁 Project Structure

```
SimPyROS/
├── examples/               # Learning-focused examples
│   ├── basic/             # Foundation concepts
│   └── pyvista/           # Interactive 3D demos ⭐
├── legacy/                # Alternative/legacy implementations  
├── tests/                 # Framework tests
├── output/                # Generated visualizations and data
├── simulation_object.py   # Core simulation framework
└── requirements.txt       # Minimal dependencies
```

## 🎯 Examples Overview

| Example | Type | Interactive | 3D Visualization | Best For |
|---------|------|-------------|------------------|----------|
| **basic_demo.py** | Foundation | ❌ | ❌ | Learning core concepts |
| **pyvista_robot_demo.py** | Interactive 3D | ✅ | ✅ PyVista | Real-time robot control |
| **pyvista_simple_demo.py** | Static 3D | ❌ | ✅ PyVista | Image generation, testing |

## 🔧 Core Concepts

### SimulationObject
- **Dynamic objects**: Robots, sensors, movable components
- **Static objects**: Environment fixtures, immovable bases  
- **Bidirectional connections**: Objects move together when connected
- **Pose system**: 6DOF positioning with quaternion rotations

### 3D Visualization
- **PyVista backend**: Professional VTK-based rendering
- **Interactive controls**: Mouse-driven camera manipulation
- **Real-time updates**: Live simulation visualization
- **Screenshot/video export**: High-quality output generation

## 📚 Learning Path

### 1. **Beginner** - Understand the Framework
```bash
python examples/basic/basic_demo.py
```
Learn object creation, connections, and movement without 3D complexity.

### 2. **Intermediate** - 3D Fundamentals  
```bash
python examples/pyvista/pyvista_simple_demo.py
```
Experience 3D mesh creation, transformations, and image generation.

### 3. **Advanced** - Interactive Simulation
```bash
python examples/pyvista/pyvista_robot_demo.py 10
```
Control robots in real-time with interactive 3D visualization.

## 🎮 Interactive Controls

**PyVista 3D Window:**
- **Mouse Left + Drag**: Rotate camera
- **Mouse Right + Drag**: Zoom in/out
- **Mouse Middle + Drag**: Pan view
- **Close Window**: Exit simulation

## 🛠️ Requirements

**Core Dependencies:**
- Python 3.7+
- SimPy 4.0+ (discrete event simulation)
- NumPy, SciPy (numerical computing)
- PyVista 0.40+ (3D visualization)

**Installation:**
```bash
pip install -r requirements.txt
```

**System Requirements:**
- OpenGL support (for 3D visualization)
- X11/Display server (Linux) or equivalent (Windows/macOS)

## 📊 Generated Outputs

Simulations generate files in the `output/` directory:

**Images:**
- `pyvista_test.png` - Static robot renderings
- `pyvista_frame_*.png` - Animation sequences
- `pyvista_robot_*.png` - Multi-pose visualizations

**Data:**
- `*.json` - Simulation state exports
- `trajectory_*.json` - Robot path data

## 🔍 Advanced Usage

### Custom Robot Development
```python
from simulation_object import SimulationObject, ObjectParameters, ObjectType, Pose

# Create custom robot
robot = SimulationObject(env, ObjectParameters(
    name="custom_robot",
    object_type=ObjectType.DYNAMIC,
    initial_pose=Pose(x=0, y=0, z=1)
))

# Add sensors, connections, behaviors...
```

### Batch Processing
```bash
# Generate image sequences
python examples/pyvista/pyvista_simple_demo.py

# Process multiple configurations
for duration in 5 10 15; do
    python examples/pyvista/pyvista_robot_demo.py $duration
done
```

## 🗂️ Legacy Support

Historical implementations are preserved in `legacy/`:
- **matplotlib visualization** - Traditional plotting approach
- **Alternative backends** - Experimental visualization systems
- **Complex demos** - Advanced multi-robot scenarios

See `legacy/README.md` for details.

## 🤝 Contributing

1. **Extend examples**: Add new robot behaviors or scenarios
2. **Improve visualization**: Enhance PyVista rendering capabilities  
3. **Add tests**: Expand framework test coverage
4. **Documentation**: Improve guides and tutorials

## 📄 Documentation

- **`examples/README.md`** - Comprehensive example documentation
- **`CLAUDE.md`** - Development history and technical details
- **`Appendix.md`** - Technical background and comparisons

## ⚠️ Troubleshooting

**3D Window doesn't open:**
```bash
# Check display
echo $DISPLAY

# Test headless mode first
python examples/pyvista/pyvista_simple_demo.py
```

**Import errors:**
```bash
# Reinstall dependencies
pip install -r requirements.txt

# Verify installation
python -c "import pyvista; print('PyVista OK')"
```

**Performance issues:**
- Reduce simulation duration
- Close other 3D applications
- Update graphics drivers
- Use headless mode for batch processing

## 📜 License

[Add your license information here]

## 🔗 Related Projects

- **SimPy**: Discrete event simulation framework
- **PyVista**: 3D visualization and mesh analysis
- **VTK**: Visualization Toolkit (PyVista backend)

---

**Start your robotics simulation journey:**
```bash
python examples/basic/basic_demo.py        # Learn the basics
python examples/pyvista/pyvista_robot_demo.py 5  # Experience 3D interaction
```