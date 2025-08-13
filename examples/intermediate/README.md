# Intermediate Examples

More complex examples demonstrating advanced features and external integrations.

## Examples

### `link_connections.py`
Demonstrates object attachment to robot links with dynamic updates.

**Features:**
- Object-to-robot link connections
- Dynamic attachment/detachment
- Coordinated motion

**Run:**
```bash
cd examples/intermediate
python link_connections.py
```

### `mesh_robots.py`
External robot mesh loading from popular repositories.

**Features:**
- TurtleBot3 and UR5 robot integration
- External repository cloning
- Real 3D mesh visualization
- Complex robot structures

**Run:**
```bash
cd examples/intermediate
python mesh_robots.py --robot turtlebot3
python mesh_robots.py --robot ur5
```

### `pyvista/` Directory
PyVista-specific visualization examples and demos.

**Examples:**
- `pyvista_robot_demo.py` - Direct PyVista robot control
- `pyvista_simple_demo.py` - Simple visualization setup
- `sample_robots.py` - Various robot types

**Run:**
```bash
cd examples/intermediate/pyvista
python pyvista_robot_demo.py 10  # 10 second demo
```

## Prerequisites

- Completed beginner examples
- Understanding of URDF format
- Familiarity with 3D transformations
- For mesh examples: git access for external repositories

## Key Concepts

- **Link Connections**: Objects attached to moving robot parts
- **External Meshes**: Loading complex 3D models from external sources
- **Advanced Visualization**: Custom PyVista rendering and controls
- **Multi-Object Coordination**: Managing multiple simulation objects

## Next Steps

Ready for advanced examples featuring:
- Multi-robot coordination
- Complex autonomous behaviors
- Event-driven SimPy processes