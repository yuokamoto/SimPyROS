# SimPyROS - Event-Driven Robot Simulation Framework

A powerful robotics simulation framework built on **SimPy's RealtimeEnvironment** with **independent process architecture** for realistic robot behaviors. Features interactive 3D visualization, URDF robot support, and event-driven multi-robot coordination.

## 🚀 Revolutionary Architecture

SimPyROS leverages **SimPy's true power** with independent processes for each robot subsystem:

```python
# Each robot has independent SimPy processes
robot.start_joint_control_process()    # 100 Hz joint control
robot.start_sensor_process()          # 30 Hz sensor processing
robot.start_navigation_process()      # 10 Hz autonomous navigation
robot.start_base_motion_process()     # 100 Hz base motion

# Compare to single-loop centralized designs!
```

This enables **natural event-driven behaviors**, **scalable performance**, and **clear code architecture**.

## ✨ Key Features

- 🤖 **URDF Robot Support**: Load complex robots with proper kinematics
- 🎮 **Interactive 3D Visualization**: Real-time PyVista rendering with controls
- ⚡ **Event-Driven Architecture**: Independent SimPy processes per robot subsystem
- 🔗 **Multi-Robot Coordination**: Patrol, search, follower behaviors
- 🎯 **Simplified Interface**: ~20 lines for complete simulations
- 📊 **Real-Time Control**: Dynamic speed, pause/resume, reset
- 🧪 **Centralized Time Management**: RealtimeEnvironment-based synchronization
- 🔧 **Headless Support**: High-performance simulation without GUI

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
cd examples/beginner
python basic_simulation.py

# Experience multi-robot coordination
cd ../advanced  
python advanced_simpy_demo.py
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
    for joint_name in robot.get_joint_names():
        position = math.sin(time.time())
        sim.set_robot_joint_position("my_robot", joint_name, position)

# 4. Run
sim.set_robot_control_callback("my_robot", control)
sim.run(duration=10.0)
```

**That's it!** Complete simulation in ~15 lines.

## 📁 Example Structure

Examples are organized by difficulty:

### 🟢 [Beginner](examples/beginner/) - Start Here
- **[basic_simulation.py](examples/beginner/basic_simulation.py)** - Essential first example
  - Single robot with movement
  - Interactive controls
  - Perfect introduction

### 🟡 [Intermediate](examples/intermediate/) - Build Skills  
- **[link_connections.py](examples/intermediate/link_connections.py)** - Object attachments
- **[mesh_robots.py](examples/intermediate/mesh_robots.py)** - TurtleBot3, UR5 integration
- **[pyvista/](examples/intermediate/pyvista/)** - Direct visualization control

### 🔴 [Advanced](examples/advanced/) - Master the Framework
- **[advanced_simpy_demo.py](examples/advanced/advanced_simpy_demo.py)** ⭐ **Featured**
  - **3 autonomous robots** with different behaviors
  - **Independent SimPy processes** demonstration
  - **Event-driven coordination** showcase
- **[all_features_demo.py](examples/advanced/all_features_demo.py)** - Complete feature tour

## 🏗️ Architecture Comparison

### Before: Single-Loop Centralized (Legacy)
```python
while simulation_running:
    for robot in robots:
        robot.update_joints(dt)     # All at same frequency
        robot.update_base(dt)       # Inflexible timing
    for obj in objects:
        obj.update_motion(dt)       # Fixed time step
    yield env.timeout(fixed_dt)     # One yield for everything
```

**Problems:** Inflexible timing, complex coordination, doesn't leverage SimPy

### After: Independent SimPy Processes (Current)
```python
# Joint control process (100 Hz)
def joint_control_process():
    while active:
        process_joint_commands()
        yield env.timeout(1.0 / 100.0)

# Navigation process (10 Hz)  
def navigation_process():
    while active:
        if target_reached():
            break
        update_path_planning()
        yield env.timeout(1.0 / 10.0)

# Each runs independently!
```

**Benefits:** Natural behaviors, efficient resource use, leverages SimPy's strengths

## 🎮 Interactive Features

All simulations include:
- **Real-time factor control** (0.1x to 5.0x speed)
- **Play/Pause/Reset** buttons  
- **Coordinate axis toggle**
- **Wireframe mode**
- **Simulation time display**
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

1. **[basic_simulation.py](examples/beginner/basic_simulation.py)** - Learn fundamentals
2. **[link_connections.py](examples/intermediate/link_connections.py)** - Object relationships  
3. **[mesh_robots.py](examples/intermediate/mesh_robots.py)** - External robot models
4. **[advanced_simpy_demo.py](examples/advanced/advanced_simpy_demo.py)** - Multi-robot coordination

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

## 📊 Performance

- **Beginner examples**: 60+ FPS with visualization, 1000+ FPS headless
- **Advanced multi-robot**: 30+ FPS, scales with robot count
- **Independent processes**: Superior to single-loop designs

## 🧪 Core Architecture

### TimeManager (RealtimeEnvironment-based)
```python
from core.time_manager import TimeManager

time_mgr = TimeManager(real_time_factor=1.0)
env = time_mgr.env  # SimPy RealtimeEnvironment
```

### Robot (Independent Processes)
```python
from core.robot import Robot

robot = Robot(env, parameters, time_manager)
# Automatically starts: joint, sensor, navigation, base processes
```

### SimulationManager (Orchestrator)
```python
from core.simulation_manager import SimulationManager

sim = SimulationManager()  # Uses RealtimeEnvironment
robot = sim.add_robot_from_urdf(name, urdf_path)
sim.run()
```

## 📚 Documentation

- **[Examples Guide](examples/README.md)** - Complete usage examples
- **[CLAUDE.md](CLAUDE.md)** - Development history and architecture details
- **[Legacy Code](legacy/)** - Previous implementations for reference

## 🎓 Educational Goals

By using SimPyROS, you'll learn:

1. **Event-Driven Simulation** - SimPy's cooperative multitasking
2. **Robot Kinematics** - URDF loading and joint control
3. **3D Visualization** - Interactive PyVista rendering
4. **Process Architecture** - Independent vs centralized design
5. **Multi-Robot Coordination** - Autonomous behaviors

## 🔮 Advanced Features

- **Autonomous Navigation**: Goal-seeking with obstacle avoidance
- **Sensor Simulation**: LIDAR, camera, IMU processing
- **Multi-Robot Coordination**: Patrol, search, follow behaviors
- **Real-Time Control**: Dynamic parameter adjustment
- **Headless Operation**: High-performance simulation

## 🤝 Contributing

This project emphasizes:
- **Educational progression** from simple to complex
- **Real-world applicability** with practical examples
- **Architecture clarity** demonstrating SimPy's strengths
- **Comprehensive documentation** of design decisions

## 📄 License

[Add your license information here]

## 🚀 What's Next?

After mastering SimPyROS:
- Build custom robot behaviors
- Implement swarm coordination
- Integrate with ROS 2
- Create autonomous vehicle simulations
- Develop warehouse robotics scenarios

---

**🎯 SimPyROS demonstrates that event-driven simulation with independent processes is superior to single-loop centralized designs. Experience the difference!**

*Latest Update: January 2025 - Revolutionary SimPy architecture with RealtimeEnvironment and independent process design*