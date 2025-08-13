# SimPyROS Examples

Welcome to SimPyROS examples! These are organized by difficulty level to help you learn the framework progressively.

## 🚀 Quick Start

```bash
# 1. Activate environment
source simpyros-env/bin/activate

# 2. Start with beginner examples
cd examples/beginner
python basic_simulation.py

# 3. Explore advanced multi-robot behaviors
cd ../advanced
python advanced_simpy_demo.py
```

## 📁 Directory Structure

### 🟢 [beginner/](beginner/) - Start Here
Perfect for first-time users and learning the basics.

- **[basic_simulation.py](beginner/basic_simulation.py)** - Essential starting point
  - Single robot with movement and visualization
  - Interactive controls (play/pause, speed, reset)
  - ~20 lines of user code for complete simulation

### 🟡 [intermediate/](intermediate/) - Build Skills
Advanced features and external integrations.

- **[link_connections.py](intermediate/link_connections.py)** - Object-robot attachments
- **[mesh_robots.py](intermediate/mesh_robots.py)** - External robot models (TurtleBot3, UR5)
- **[pyvista/](intermediate/pyvista/)** - Direct PyVista visualization examples

### 🔴 [advanced/](advanced/) - Master the Framework
Complex behaviors showcasing SimPy's full power.

- **[advanced_simpy_demo.py](advanced/advanced_simpy_demo.py)** ⭐ **Featured**
  - Multi-robot coordination with independent behaviors
  - Event-driven SimPy processes demonstration
  - Patrol, search, and follower robots
- **[all_features_demo.py](advanced/all_features_demo.py)** - Complete feature showcase

## 🎯 Learning Path

1. **Start**: `beginner/basic_simulation.py` - Learn the fundamentals
2. **Expand**: `intermediate/link_connections.py` - Understand object relationships
3. **Explore**: `intermediate/mesh_robots.py` - Work with real robot models
4. **Master**: `advanced/advanced_simpy_demo.py` - See the architecture's true power

## 🏗️ Architecture Highlights

SimPyROS uses **SimPy's RealtimeEnvironment** with **independent processes** for each robot subsystem:

```python
# Modern SimPy Architecture (Current)
robot.start_joint_control_process()    # 100 Hz joint control
robot.start_sensor_process()          # 30 Hz sensor processing  
robot.start_navigation_process()      # 10 Hz path planning
robot.start_base_motion_process()     # 100 Hz motion control

# Each process runs independently with proper SimPy yielding!
```

This replaces single-loop centralized designs and leverages SimPy's event-driven strengths.

## 🎮 Interactive Features

All examples include:
- **Real-time factor control** (0.1x to 5.0x speed)
- **Play/Pause/Reset** buttons
- **Coordinate axis toggle**
- **Wireframe mode**
- **Simulation time display**

## 🤖 Robot Models

Examples use three main robots from `examples/robots/`:

- **articulated_arm_robot.urdf** - 4-DOF arm for manipulation demos
- **collision_robot.urdf** - Multi-robot collision scenarios
- **mobile_robot.urdf** - Mobile base with sensors

External robots (intermediate examples):
- **TurtleBot3** - Popular mobile robot
- **UR5** - Industrial robot arm

## 🔧 Prerequisites

```bash
# Virtual environment setup
source simpyros-env/bin/activate

# Required packages (should already be installed)
pip install simpy pyvista yourdfpy numpy scipy
```

## 🆘 Troubleshooting

### Display Issues
```bash
# Check DISPLAY variable
echo $DISPLAY

# If empty, visualization will run in headless mode
export DISPLAY=:0  # Adjust as needed
```

### URDF Loading Issues
```bash
# Verify robot files exist
ls examples/robots/*.urdf

# For external mesh examples, ensure git access
git --version
```

### Performance Issues
- Lower real-time factor if simulation is slow
- Use headless mode for maximum performance
- Check available system resources

## 📊 Performance Expectations

- **Beginner**: 60+ FPS with visualization, 1000+ FPS headless
- **Intermediate**: 30+ FPS with complex meshes
- **Advanced**: 30+ FPS with multiple robots, scales with robot count

## 🎓 Learning Objectives

By completing these examples, you'll understand:

1. **SimPy Fundamentals**: Event-driven simulation with RealtimeEnvironment
2. **Robot Control**: Joint-level and base-level motion control
3. **Visualization**: Interactive 3D visualization with PyVista
4. **Architecture**: Independent process design vs single-loop approaches
5. **Coordination**: Multi-robot behaviors and object interactions

## 📚 Next Steps

After mastering these examples:
- Explore the [core/](../core/) module documentation
- Create custom robot behaviors
- Implement your own URDF robots
- Build multi-robot coordination algorithms
- Integrate with ROS 2 systems

Happy simulating! 🚀