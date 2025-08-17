# SimPyROS Architecture Documentation

## Overview
SimPyROS implements a **centralized discrete event simulation framework** built on SimPy with unified time management and optimized performance.

## Core Architecture

### Centralized Update Management
```
SimulationManager._simulation_process_loop()
    ↓ (single while loop with yield)
advance_sim_time() → current_sim_time
    ↓ (centralized time management)
for robot in robots:
    robot.update_joints_if_needed(current_sim_time)  # Joint control at 100Hz
    robot.update_if_needed(current_sim_time)         # Base motion updates
    ↓ (direct method calls, no processes)
for obj in objects:
    obj.update_if_needed(current_sim_time)           # Custom update rates
    ↓ (sim_time based timing)
for callback in control_callbacks:
    callback.call_if_needed(current_sim_time)        # User control logic
    ↓ (single process coordination)
yield env.timeout(time_step)  # Only one yield in entire system
```

### Key Components

#### 1. SimulationManager (`core/simulation_manager.py`)
- **Central orchestrator** for all simulation activities
- **Unified interface** reducing user code from ~100 lines to ~20 lines
- **Real-time factor control** with processing time compensation
- **Multiple visualization backends** (PyVista, MeshCat, process-separated)

#### 2. TimeManager (`core/time_manager.py`)
- **Centralized time management** using SimPy RealtimeEnvironment
- **sim_time based updates** for all components
- **Real-time synchronization** with processing time compensation
- **Global time access** for consistent timing across components

#### 3. Robot (`core/robot.py`)
- **URDF-based robot modeling** with yourdfpy integration
- **Joint control** with forward kinematics
- **Base motion** for mobile robots
- **sim_time based updates** replacing process-based loops

#### 4. Visualization Systems
- **Process-separated PyVista** (`legacy/process_separated_urdf_visualizer.py`) - Crash isolation (default)
- **PyVistaVisualizer** (`core/pyvista_visualizer.py`) - Standard 3D visualization
- **MeshCatVisualizer** (`core/meshcat_visualizer.py`) - Web-based visualization

#### 5. Simulation Objects (`core/simulation_object.py`)
- **Base classes** for all simulated entities
- **Pose and Velocity** management
- **Custom update frequencies** based on sim_time

## Performance Characteristics

### Timing Accuracy
- **Real-time factor control**: Precise speed adjustment (0.1x to 5.0x)
- **Processing time compensation**: Accounts for simulation overhead
- **Timing accuracy**: <5% error in most scenarios
- **High performance**: 1500+ Hz headless, 60+ Hz with visualization

### Process Optimization
- **Single main process** with centralized update loop
- **Frequency grouping** for efficient callback management
- **Direct method calls** instead of inter-process communication
- **Reduced overhead** compared to multi-process architectures

## Visualization Backends

### Standard PyVista (`pyvista`)
- **Recommended for most users**
- **Interactive controls** with real-time factor slider
- **Direct rendering** in main process

### Process-separated PyVista (`process_separated_pyvista`)
- **Crash isolation** between simulation and visualization
- **Shared memory** communication for high performance
- **Non-blocking updates** maintaining SimPy performance

### MeshCat (`meshcat`)
- **Web-based visualization** accessible via browser
- **Lightweight** with good performance
- **Remote access** capabilities


## Directory Structure

```
SimPyROS/
├── core/                          # Active architecture
│   ├── simulation_manager.py     # Central orchestrator
│   ├── time_manager.py           # Unified time management
│   ├── robot.py                  # Robot modeling
│   ├── simulation_object.py      # Base simulation entities
│   ├── pyvista_visualizer.py     # Standard visualization
│   ├── meshcat_visualizer.py     # Web visualization
│   ├── urdf_loader.py            # URDF processing
│   ├── external_mesh_manager.py  # External robot support
│   └── link_connector.py         # Robot attachments
├── examples/                      # Educational progression
│   ├── beginner/                 # Simple introductory examples
│   ├── intermediate/             # Feature exploration
│   └── advanced/                 # Complete demonstrations
├── legacy/                        # Previous implementations
│   ├── process_separated_*.py    # Process-separated systems
│   ├── process_separated_*.py  # Process isolation backends
│   └── unified/                  # Legacy unified backends
└── docs/                         # Documentation
```

## Design Principles

### 1. Centralized Control
- **Single simulation loop** eliminates process synchronization complexity
- **Direct method calls** instead of inter-process communication
- **Unified time management** ensures consistent timing

### 2. Performance Optimization
- **Processing time compensation** for accurate real-time synchronization
- **Frequency grouping** reduces process overhead
- **Efficient update patterns** based on simulation time

### 3. Educational Progression
- **Beginner examples** introduce core concepts
- **Intermediate examples** explore features
- **Advanced examples** demonstrate complete systems

### 4. Backwards Compatibility
- **Legacy backends** available for specialized use cases
- **Gradual migration** path from older architectures
- **Import compatibility** maintained where possible

## Migration from Legacy Systems

### Process-based → Centralized Updates
```python
# OLD: Multiple processes with yields
def robot_joint_control_loop():
    while True:
        update_joints()
        yield env.timeout(joint_update_interval)

def robot_motion_loop():
    while True:
        update_base_motion()
        yield env.timeout(motion_update_interval)

# NEW: Centralized updates
def update_if_needed(self, current_sim_time):
    if current_sim_time >= self._last_update_time + self.update_interval:
        self._update_state()
        self._last_update_time = current_sim_time
```

### Multi-process → Single Loop
```python
# OLD: Multiple SimPy processes
env.process(robot.joint_control_loop())
env.process(robot.motion_loop())
env.process(sensor.update_loop())

# NEW: Single centralized loop
for robot in robots:
    robot.update_joints_if_needed(sim_time)
    robot.update_if_needed(sim_time)
for sensor in sensors:
    sensor.update_if_needed(sim_time)
```

## Future Enhancements

1. **ROS 2 Integration**: Bridge to real robot systems
2. **Advanced Physics**: Integration with physics engines
3. **Distributed Simulation**: Multi-node simulation support
4. **Cloud Deployment**: Scalable cloud-based simulations
5. **Machine Learning**: Integration with ML frameworks

## Performance Metrics

- **Simulation Rate**: 1500+ Hz (headless), 60+ Hz (visualization)
- **Timing Accuracy**: <5% error with processing time compensation
- **Memory Usage**: Optimized for multiple robots and objects
- **Startup Time**: <2 seconds for complex multi-robot scenarios
- **Process Reduction**: Up to 50% fewer processes with frequency grouping

---

*This architecture provides superior performance, simplified debugging, and educational value compared to traditional multi-process simulation designs.*