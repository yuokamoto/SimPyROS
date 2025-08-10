# SimPyROS - Python Robot Simulation Framework

A comprehensive robotics simulation framework built on SimPy with interactive 3D visualization powered by PyVista and advanced URDF robot support.

## 📋 System Requirements

### Python Version
- **Python 3.8+** (recommended: Python 3.9 or 3.10)
- **Platforms**: Linux, macOS, Windows

### System Dependencies

#### Linux (Ubuntu/Debian)
```bash
# For 3D visualization
sudo apt-get update
sudo apt-get install python3-dev python3-pip

# For headless environments (servers/CI)
sudo apt-get install xvfb

# For mesh processing (optional)
sudo apt-get install libassimp-dev
```

#### macOS
```bash
# Install Python and pip (if not available)
brew install python

# For mesh processing (optional)  
brew install assimp
```

#### Windows
- Install Python 3.8+ from python.org
- Visual Studio Build Tools may be required for some packages
- Consider using conda for easier VTK installation

## 🚀 Quick Start

### Environment Setup (Recommended)

We **strongly recommend** using a virtual environment to avoid dependency conflicts:

#### Option 1: Using Python venv (Built-in)
```bash
# Clone the repository
git clone <repository-url>
cd SimPyROS

# Create virtual environment
python -m venv simpyros-env

# Activate virtual environment
# On Linux/macOS:
source simpyros-env/bin/activate
# On Windows:
# simpyros-env\Scripts\activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Optional: Install legacy support (only if needed)
# pip install -r legacy/requirements-legacy.txt
```

#### Option 2: Using pyenv + pyenv-virtualenv
```bash
# Install Python 3.8+ if not available
pyenv install 3.9.18  # or newer version

# Clone and setup
git clone <repository-url>
cd SimPyROS

# Create virtual environment with pyenv
pyenv virtualenv 3.9.18 simpyros
pyenv local simpyros  # Auto-activates in this directory

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Optional: Install legacy support (only if needed)
# pip install -r legacy/requirements-legacy.txt
```

#### Option 3: Using conda/miniconda
```bash
# Clone the repository
git clone <repository-url>
cd SimPyROS

# Create conda environment
conda create -n simpyros python=3.9
conda activate simpyros

# Install dependencies
pip install -r requirements.txt

# Optional: Install legacy support (only if needed)  
# pip install -r requirements-legacy.txt
```

### Installation Verification
```bash
# Test core functionality
python examples/basic/basic_demo.py

# Test 3D visualization (requires display/X11)
python examples/pyvista/pyvista_simple_demo.py

# Test URDF robot loading
python examples/urdf/urdf_robot_demo.py 5 examples/robots/simple_robot.urdf
```

### Run Your First Simulation
```bash
# Learn the basics
python examples/basic/basic_demo.py

# Experience interactive 3D with built-in robots
python examples/pyvista/pyvista_robot_demo.py 5

# Load and visualize URDF robots
python examples/urdf/urdf_robot_demo.py 10 examples/robots/mobile_robot.urdf
```

### Troubleshooting Installation

#### Common Issues:

**PyVista/VTK Installation Issues:**
```bash
# If PyVista fails to install
pip install --upgrade pip setuptools wheel
pip install pyvista --no-cache-dir

# Alternative: Use conda for VTK
conda install -c conda-forge vtk pyvista
```

**URDF Libraries Issues:**
```bash
# If yourdfpy installation fails
pip install yourdfpy --no-deps
pip install trimesh networkx

# For dependency conflicts with legacy URDF libraries
# Use the clean requirements.txt (without urdfpy/pycollada conflicts)
pip install -r requirements.txt

# Only install legacy support if specifically needed
# pip install -r legacy/requirements-legacy.txt
```

**Headless Environment (no display):**
```bash
# For servers/CI environments, install headless support
sudo apt-get install xvfb  # On Ubuntu/Debian
pip install pyvista[headless]

# Test headless mode
python examples/urdf/urdf_robot_demo.py 5 --headless
```

**Virtual Environment Issues:**
```bash
# If activation doesn't work, try full path
/path/to/simpyros-env/bin/python examples/basic/basic_demo.py

# Or reinstall in environment
pip uninstall -r requirements.txt
pip install -r requirements.txt
```

#### Environment Management Tips:

**Check your current environment:**
```bash
# Verify Python path
which python
python --version

# List installed packages
pip list

# Check if packages are installed correctly
python -c "import simpy, numpy, scipy, pyvista; print('All core dependencies available')"
```

**Deactivate environment when done:**
```bash
# For venv/pyenv
deactivate

# For conda
conda deactivate
```

**Remove environment (if needed):**
```bash
# For venv
rm -rf simpyros-env

# For pyenv
pyenv uninstall simpyros

# For conda
conda env remove -n simpyros
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
├── urdf_loader.py             # URDF robot loading with colors
├── requirements.txt           # Core dependencies (recommended)
├── README.md                  # This file
├── examples/                  # Demonstrations
│   ├── basic/                 # Basic learning demos
│   ├── pyvista/              # PyVista 3D visualization demos
│   ├── urdf/                 # URDF robot loading and joint control demos
│   ├── robot_demo.py         # Robot class fundamentals
│   └── robots/               # URDF robot model files
├── legacy/                    # Deprecated/reference code
│   ├── loaders/              # Old URDF loaders
│   ├── debug/                # Development debugging scripts
│   ├── examples/             # Legacy matplotlib demos
│   ├── requirements-legacy.txt # Legacy/fallback dependencies (optional)
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

**Note**: Make sure your virtual environment is activated before running examples:
```bash
# Activate your environment first
source simpyros-env/bin/activate  # Linux/macOS
# OR: simpyros-env\Scripts\activate  # Windows
# OR: conda activate simpyros  # Conda
```

### Basic Simulation
```bash
# Simple robot concepts (no 3D visualization)
python examples/basic/basic_demo.py
```

### Interactive 3D Visualization  
```bash
# Built-in wheeled robot (10 seconds)
python examples/pyvista/pyvista_robot_demo.py 10

# Load custom URDF robot
python examples/urdf/urdf_robot_demo.py 15 examples/robots/simple_robot.urdf

# Headless mode without screenshots
python examples/urdf/urdf_robot_demo.py 10 --headless

# Headless mode with screenshot capture
python examples/urdf/urdf_robot_demo.py 10 --headless --screenshots
```

### Advanced Robot Control
```bash
# Full robot class demonstration with joint control
python examples/robot_demo.py

# Joint motion with visual feedback
python examples/pyvista/simple_joint_demo.py

# Real-time joint animation
python examples/pyvista/realtime_joint_demo.py 15
```

### Available Robot Models
```bash
# Simple 3-link arm robot with colors
python examples/urdf/urdf_robot_demo.py 10 examples/robots/simple_robot.urdf

# Mobile robot with wheels and camera
python examples/urdf/urdf_robot_demo.py 15 examples/robots/mobile_robot.urdf  

# Multi-color rotation test robot
python examples/urdf/urdf_robot_demo.py 10 examples/robots/rotation_test.urdf
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