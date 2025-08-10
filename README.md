# SimPyROS - Python Robot Simulation Framework

A comprehensive robotics simulation framework built on SimPy with interactive 3D visualization powered by PyVista and advanced URDF robot support.

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

# Load and visualize URDF robots
python examples/pyvista/urdf_robot_demo.py 10 examples/robots/mobile_robot.urdf
```

## ✨ Key Features

- **🤖 URDF Robot Support**: Load and visualize robots from URDF files with material colors
- **🎮 Interactive 3D Visualization**: Real-time robot visualization with mouse controls
- **⚙️ Advanced Movement System**: Individual link coloring and proper coordinate transformations
- **🔧 Flexible Object System**: Bidirectional connections, static constraints, dynamic relationships
- **⚡ Real-time Simulation**: SimPy-based discrete event simulation with real-time capabilities
- **🎯 Educational Focus**: Progressive learning path from basic concepts to advanced 3D interaction
- **🖼️ Headless Support**: Generate images and data without GUI requirements
- **📊 Multiple Modes**: Interactive, headless, screenshot capture modes

## 🏗️ Project Structure

```
SimPyROS/
├── simulation_object.py       # Core simulation framework
├── pyvista_visualizer.py      # 3D visualization system
├── advanced_urdf_loader.py    # URDF robot loading with colors
├── requirements.txt           # Dependencies
├── README.md                  # This file
├── examples/                  # Demonstrations
│   ├── basic/                 # Basic learning demos
│   ├── pyvista/              # Advanced 3D demos  
│   └── robots/               # URDF robot models
├── legacy/                    # Deprecated/reference code
│   ├── loaders/              # Old URDF loaders
│   ├── debug/                # Development debugging scripts
│   ├── examples/             # Legacy matplotlib demos
│   └── backends/             # Alternative visualization backends
├── docs/                      # Documentation
│   ├── CLAUDE.md             # Complete development history
│   ├── Appendix.md           # Technical implementation details
│   └── visualization_comparison.md  # Backend comparisons
├── tests/                     # Test suite
├── output/                    # Generated files
└── unused/                    # Deprecated files
```

## 🎮 Usage Examples

### Basic Simulation
```bash
# Simple robot concepts
python examples/basic/basic_demo.py
```

### Interactive 3D Visualization  
```bash
# Built-in wheeled robot (10 seconds)
python examples/pyvista/pyvista_robot_demo.py 10

# Load custom URDF robot
python examples/pyvista/urdf_robot_demo.py 15 examples/robots/simple_robot.urdf

# Headless mode without screenshots
python examples/pyvista/urdf_robot_demo.py 10 --headless

# Headless mode with screenshot capture
python examples/pyvista/urdf_robot_demo.py 10 --headless --screenshots
```

### Available Robot Models
```bash
# Simple 3-link arm robot with colors
python examples/pyvista/urdf_robot_demo.py 10 examples/robots/simple_robot.urdf

# Mobile robot with wheels and camera
python examples/pyvista/urdf_robot_demo.py 15 examples/robots/mobile_robot.urdf  

# Multi-color rotation test robot
python examples/pyvista/urdf_robot_demo.py 10 examples/robots/rotation_test.urdf
```

## 🔧 Technical Features

### URDF Robot Loading
- **yourdfpy integration**: Modern URDF parsing with fallback support
- **Material color extraction**: Individual link coloring from URDF materials
- **Coordinate transformations**: Accurate kinematic chain calculations
- **Mesh support**: Geometric primitives (box, cylinder, sphere) with planned mesh file support

### Visualization System
- **PyVista primary backend**: High-quality 3D rendering with interactive controls
- **Individual actor management**: Separate rendering for each robot link
- **Real-time updates**: Smooth animation with proper pose transformations
- **Headless capability**: Off-screen rendering for CI/testing environments

### Simulation Engine
- **SimPy foundation**: Discrete event simulation with real-time synchronization
- **Object relationship system**: Parent-child connections with bidirectional communication
- **Static constraint enforcement**: Prevents movement of connected static objects
- **Extensible architecture**: Easy addition of new robot types and behaviors

## 📚 Documentation

- **[Complete Development History](docs/CLAUDE.md)**: Session-by-session implementation details
- **[Technical Implementation](docs/Appendix.md)**: Deep technical details and comparisons  
- **[Examples Guide](examples/README.md)**: Comprehensive usage examples
- **[Visualization Backends](docs/visualization_comparison.md)**: Backend comparison and selection

## 🧪 Testing

```bash
# Run core functionality tests
python tests/test_static_constraint.py
python tests/test_rotation_accuracy.py

# Test real-time capabilities  
python tests/test_realtime.py
```

## 🤝 Contributing

This project follows an educational development approach:

1. **Examples-first**: New features demonstrated through examples
2. **Progressive complexity**: From basic concepts to advanced 3D interaction
3. **Comprehensive documentation**: Every major change documented in CLAUDE.md
4. **Backward compatibility**: Legacy examples maintained for reference

## 📄 License

[Add your license information here]

## 🎯 Roadmap

- **Physical simulation**: PyBullet integration
- **Advanced sensors**: LiDAR, camera simulation  
- **Multi-robot scenarios**: Swarm and coordination demos
- **ROS integration**: ROS 2 node compatibility
- **Web interface**: Browser-based robot control

---

**Latest Update**: August 2025 - Added comprehensive URDF robot support with individual link coloring and advanced movement system.