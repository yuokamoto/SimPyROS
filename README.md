# SimPyROS - Centralized Discrete Event Simulation Framework

A powerful robotics simulation framework built on **SimPy's centralized architecture** with **unified time management** for efficient robot simulation. Features interactive 3D visualization, URDF robot support, and high-performance simulation with real-time control.

## 🚀 Revolutionary Centralized Architecture

SimPyROS leverages **centralized update management** with a single simulation loop for optimal performance:

```python
# Centralized Update Flow (Latest Architecture)
SimulationManager._simulation_process_loop():
    advance_sim_time() → current_sim_time
    for robot in robots:
        robot.update_joints_if_needed(current_sim_time)  # 100Hz joint control
        robot.update_if_needed(current_sim_time)         # Base motion updates
    for obj in objects:
        obj.update_if_needed(current_sim_time)           # Custom update rates
    for callback in control_callbacks:
        callback.call_if_needed(current_sim_time)        # User control logic
    yield env.timeout(time_step)  # Single yield for optimal performance
```

This enables **superior performance** (1500+ Hz headless), **simplified debugging**, and **precise timing control**.

## ✨ Key Features

- 🤖 **URDF Robot Support**: Load complex robots with proper kinematics
- 🎮 **Interactive 3D Visualization**: Real-time PyVista rendering with controls
- ⚡ **Centralized Architecture**: Single-loop design with optimal performance
- 🔗 **Multi-Robot Support**: Multiple robots with independent update rates
- 🎯 **Simplified Interface**: ~20 lines for complete simulations
- 📊 **Real-Time Control**: Dynamic speed, pause/resume, reset functionality
- ⏱️ **Unified Time Management**: Centralized sim_time access for all components
- 🧪 **High Performance**: 1500+ Hz headless, 60+ Hz with visualization
- 🔧 **Processing Time Compensation**: Accurate real-time synchronization

## 🚀 Quick Start

### 1. Environment Setup

```bash
# Clone and setup
git clone <repository-url>
cd SimPyROS

# Create virtual environment (recommended)
python -m venv simpyros-env
source simpyros-env/bin/activate  # Linux/macOS
# OR: simpyros-env\Scripts\activate  # Windows

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

### 2. Run Your First Simulation

```bash
# Activate environment
source simpyros-env/bin/activate

# Start with basics
python examples/beginner/basic_simulation.py --vis

# Experience advanced multi-robot
python examples/advanced/all_features_demo.py --vis
```

### 3. Simple Code Example

```python
from core.simulation_manager import SimulationManager
import math

# 1. Create simulation
sim = SimulationManager()

# 2. Add robot
robot = sim.add_robot_from_urdf("my_robot", "robots/articulated_arm_robot.urdf")

# 3. Define control
def control(dt):
    sim_time = sim.get_sim_time()
    for joint_name in robot.get_joint_names():
        position = math.sin(sim_time)
        sim.set_robot_joint_position("my_robot", joint_name, position)

# 4. Run
sim.set_robot_control_callback("my_robot", control)
sim.run(duration=10.0, visualization=True)
```

**That's it!** Complete simulation in ~15 lines.

## 📁 Example Structure

Examples are organized by difficulty:

### 🟢 [Beginner](examples/beginner/) - Start Here
- **[basic_simulation.py](examples/beginner/basic_simulation.py)** - Essential first example
  - Single robot with movement
  - Interactive visualization controls
  - Perfect introduction to centralized architecture

### 🟡 [Intermediate](examples/intermediate/) - Build Skills  
- **[link_connections.py](examples/intermediate/link_connections.py)** - Object attachments
- **[mesh_robots.py](examples/intermediate/mesh_robots.py)** - TurtleBot3, UR5 integration
- **[pyvista/](examples/intermediate/pyvista/)** - Direct visualization control

### 🔴 [Advanced](examples/advanced/) - Master the Framework
- **[all_features_demo.py](examples/advanced/all_features_demo.py)** ⭐ **Featured**
  - **Complete feature showcase** with all visualization backends
  - **Multiple robot coordination** demonstrations
  - **Performance comparison** between backends
- **[advanced_simpy_demo.py](examples/advanced/advanced_simpy_demo.py)** - Advanced SimPy patterns

## 🏗️ Architecture Comparison

### Before: Independent Process Architecture (Legacy)
```python
# Multiple independent SimPy processes (legacy approach)
robot.start_joint_control_process()    # 100 Hz joint control
robot.start_sensor_process()          # 30 Hz sensor processing
robot.start_navigation_process()      # 10 Hz autonomous navigation
robot.start_base_motion_process()     # 100 Hz base motion

# Problems: Process synchronization complexity, lower performance
```

### After: Centralized Update Management (Current)
```python
# Single centralized loop with direct method calls
SimulationManager._simulation_process_loop():
    for robot in robots: robot.update_joints_if_needed(sim_time)
    for obj in objects: obj.update_if_needed(sim_time)
    yield env.timeout(time_step)  # Only one yield in entire system

# Benefits: Superior performance, simplified debugging, precise timing
```

## 🎮 Interactive Features

All simulations include:
- **Real-time factor control** (0.1x to 5.0x speed with slider)
- **Play/Pause/Reset** buttons with emoji labels
- **🎯 Axes**: Toggle coordinate axis display
- **🚧 Collision**: Toggle collision geometry display  
- **🕸️ Wire**: Toggle wireframe rendering mode
- **Simulation time display** with frame rate monitoring
- **Mouse-controlled 3D navigation**

## 🤖 Robot Models

### Built-in Robots (examples/robots/)
- **articulated_arm_robot.urdf** - 4-DOF arm for manipulation
- **collision_robot.urdf** - Multi-robot collision scenarios
- **mobile_robot.urdf** - Mobile base with sensors

### External Robots (auto-downloaded)
- **TurtleBot3** - Popular mobile robot (burger, waffle, waffle_pi)
- **UR5** - Industrial robot arms (ur3, ur5, ur10, etc.)

## 🎯 Learning Path

1. **[basic_simulation.py](examples/beginner/basic_simulation.py)** - Learn centralized architecture fundamentals
2. **[link_connections.py](examples/intermediate/link_connections.py)** - Object relationships  
3. **[mesh_robots.py](examples/intermediate/mesh_robots.py)** - External robot models
4. **[all_features_demo.py](examples/advanced/all_features_demo.py)** - Complete feature showcase

## 🔧 System Requirements

- **Python 3.8+** (recommended: 3.9+)
- **Platforms**: Linux, macOS, Windows
- **Dependencies**: SimPy, PyVista, NumPy, SciPy

### Optional System Packages
```bash
# Linux (Ubuntu/Debian)
sudo apt-get install python3-dev xvfb  # For headless support

# macOS  
brew install python

# Windows
# Install Python from python.org
```

## 🆘 Troubleshooting

### Display Issues
```bash
# Check display
echo $DISPLAY

# For headless servers
export DISPLAY=:0
```

### Installation Issues
```bash
# Clean install
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt --no-cache-dir

# Alternative: conda
conda install -c conda-forge vtk pyvista
```

### Performance Issues
- Lower real-time factor if slow
- Use headless mode: `--headless`
- Check system resources

## 📊 Performance Metrics

- **Headless Mode**: 1500+ Hz simulation rates
- **With Visualization**: 60+ Hz with smooth animation
- **Multi-Robot**: Scales efficiently with robot count
- **Timing Accuracy**: <5% error with processing time compensation
- **Memory Usage**: Optimized for multiple robots and objects

## 🧪 Core Architecture Components

### SimulationManager (Centralized Orchestrator)
```python
from core.simulation_manager import SimulationManager

sim = SimulationManager()  # Centralized time management
robot = sim.add_robot_from_urdf(name, urdf_path)
sim.run()  # Single simulation loop
```

### TimeManager (Unified Timing)
```python
from core.time_manager import TimeManager

time_mgr = TimeManager(real_time_factor=1.0)
# Centralized sim_time access for all components
```

### Robot (sim_time Based Updates)
```python
from core.robot import Robot

robot = Robot(env, parameters, time_manager)
# Updates based on simulation time, not real time
```

## 📚 Documentation

- **[Examples Guide](examples/README.md)** - Complete usage examples
- **[CLAUDE.md](CLAUDE.md)** - Development history and detailed architecture
- **[Legacy Code](legacy/)** - Previous implementations for reference

## 🎓 Educational Goals

By using SimPyROS, you'll learn:

1. **Centralized Simulation Architecture** - Single-loop vs multi-process design
2. **Unified Time Management** - sim_time based coordination
3. **Robot Kinematics** - URDF loading and joint control
4. **3D Visualization** - Interactive PyVista rendering
5. **Performance Optimization** - Efficient update management

## 🔮 Advanced Features

- **Autonomous Navigation**: Goal-seeking with obstacle avoidance
- **Sensor Simulation**: LIDAR, camera, IMU processing
- **Multi-Robot Coordination**: Centralized behavior management
- **Real-Time Control**: Dynamic parameter adjustment
- **Headless Operation**: High-performance simulation without GUI
- **Processing Time Compensation**: Accurate real-time synchronization

## 🤝 Contributing

This project emphasizes:
- **Centralized architecture** for optimal performance
- **Educational progression** from simple to complex
- **Real-world applicability** with practical examples
- **Performance optimization** with timing accuracy
- **Comprehensive documentation** of design decisions

## 📄 License

[Add your license information here]

## 🌟 Visualization Backends

SimPyROS supports multiple visualization options:

```bash
# Standard PyVista (recommended for most users)
python examples/beginner/basic_simulation.py --vis --visualization-backend pyvista

# Process-separated PyVista (crash isolation)
python examples/beginner/basic_simulation.py --vis --visualization-backend process_separated_pyvista

# MeshCat web-based (lightweight, browser-based)
python examples/beginner/basic_simulation.py --vis --visualization-backend meshcat

# Optimized PyVista (performance-focused)
python examples/beginner/basic_simulation.py --vis --visualization-backend optimized_pyvista
```

## 🚀 What's Next?

After mastering SimPyROS:
- Build custom robot behaviors with centralized control
- Implement swarm coordination with unified timing
- Integrate with ROS 2 for real robot deployment
- Create autonomous vehicle simulations
- Develop warehouse robotics scenarios

---

**🎯 SimPyROS demonstrates that centralized update management with unified time control is superior to complex multi-process designs. Experience the performance difference!**

*Latest Update: August 2025 - Revolutionary centralized architecture with unified time management and processing time compensation*