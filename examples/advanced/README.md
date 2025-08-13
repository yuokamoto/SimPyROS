# Advanced Examples

Complex simulations showcasing the full power of SimPyROS's event-driven architecture.

## Examples

### `advanced_simpy_demo.py` ⭐ **Featured**
**Comprehensive demonstration of SimPy's event-driven architecture** with multiple autonomous robots.

**Features:**
- **3 Independent Robot Behaviors:**
  - Patrol Robot: Autonomous waypoint navigation
  - Search Robot: Random exploration with arm scanning
  - Follower Robot: Leader-following behavior
- **Independent SimPy Processes:** Each robot subsystem runs in parallel
- **Event-Driven Coordination:** Natural robot interactions
- **Real-Time Visualization:** Interactive controls during simulation

**Run:**
```bash
cd examples/advanced
python advanced_simpy_demo.py
```

**Highlights:**
- Demonstrates why SimPy processes > single-loop design
- Complex autonomous behaviors
- Multi-robot coordination
- Real-time parameter adjustment

### `all_features_demo.py`
Comprehensive feature showcase of SimPyROS capabilities.

**Features:**
- All visualization features
- Complete robot control interfaces
- UI control demonstrations
- Performance benchmarking

**Run:**
```bash
cd examples/advanced
python all_features_demo.py
```

## Architecture Showcase

These examples demonstrate the **revolutionary SimPy architecture** that replaces single-loop designs:

### Before: Single Loop (Legacy)
```python
while simulation_running:
    for robot in robots:
        robot.update_joints()
        robot.update_base()
    for obj in objects:
        obj.update_motion()
    yield env.timeout(fixed_timestep)
```

### After: Independent Processes (Current)
```python
# Robot joint control process (100 Hz)
def joint_control_process():
    while active:
        process_joint_commands()
        yield env.timeout(1.0 / 100.0)

# Robot navigation process (10 Hz)
def navigation_process():
    while active:
        update_navigation()
        yield env.timeout(1.0 / 10.0)

# Each runs independently!
```

## Benefits Demonstrated

1. **Natural Event-Driven Behaviors**: Robots react to conditions naturally
2. **Scalable Performance**: Independent frequency control per subsystem
3. **Code Clarity**: Each behavior is a separate, understandable process
4. **SimPy's True Power**: Leverages cooperative multitasking effectively

## Prerequisites

- Completed beginner and intermediate examples
- Understanding of SimPy processes and generators
- Familiarity with event-driven programming
- Experience with multi-robot systems

## Performance Notes

- **advanced_simpy_demo.py**: Optimized for observation (1.5x real-time)
- Multiple independent processes running concurrently
- RealtimeEnvironment provides accurate timing
- Interactive real-time factor adjustment

## Real-World Applications

These patterns are applicable to:
- **Autonomous Vehicle Fleets**: Coordinated navigation
- **Warehouse Robotics**: Task allocation and coordination
- **Search and Rescue**: Multi-robot exploration
- **Manufacturing**: Coordinated assembly operations