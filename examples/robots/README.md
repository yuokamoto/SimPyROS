# Robot Models Directory

This directory contains URDF files for testing and demonstration purposes.

## Available Robot Models

### 1. `articulated_arm_robot.urdf`
- **Description**: 4-DOF articulated robotic arm with realistic joint structure
- **Links**: 5 (base_link, shoulder_link, elbow_link, wrist_link, end_effector)
- **Joints**: 4 revolute joints with different axes
- **Use case**: Advanced robotics concepts, multi-joint coordination, and arm manipulation

**Structure:**
- Base: Gray cylinder (radius=0.15m, height=0.1m) - stationary base
- Shoulder: Orange cylinder (radius=0.05m, length=0.3m) - first arm segment
- Elbow: Blue cylinder (radius=0.04m, length=0.25m) - second arm segment  
- Wrist: Green cylinder (radius=0.03m, length=0.15m) - third arm segment
- End effector: Red sphere (radius=0.06m) - gripper/tool attachment

**Joint Configuration:**
- base_to_shoulder: Z-axis rotation (full 360°)
- shoulder_to_elbow: X-axis pitch (-2.0 to +2.0 rad)
- elbow_to_wrist: X-axis pitch (-2.5 to +2.5 rad)  
- wrist_to_end: Z-axis rotation (-1.57 to +1.57 rad)

### 2. `collision_robot.urdf`
- **Description**: 3-DOF robot with collision detection elements
- **Links**: 3 (base_link, arm_link, end_effector)
- **Joints**: 2 revolute joints
- **Use case**: Collision detection, multi-robot scenarios, physics simulation

**Structure:**
- Base: Purple box (0.6 × 0.4 × 0.2 m)
- Arm: Cyan cylinder (radius=0.05m, length=0.6m)
- End effector: Magenta sphere (radius=0.08m)

**Features:**
- Includes `<collision>` elements for physics simulation
- Distinctive color scheme (purple/cyan/magenta) for multi-robot visualization
- Optimized for collision detection algorithms

### 3. `mobile_robot.urdf`
- **Description**: Mobile robot with wheels and sensor equipment
- **Links**: 5 (base, 2 wheels, sensor mast, camera)
- **Joints**: 4 (2 continuous wheel joints, 2 revolute sensor joints)
- **Use case**: Mobile robotics and sensor platform demonstration

**Structure:**
- Base: Gray box chassis (0.8 × 0.5 × 0.2 m)
- Wheels: Two black cylinders (radius=0.15m)
- Sensor mast: Blue cylinder (radius=0.03m, height=0.4m)
- Camera: Green box sensor (0.1 × 0.08 × 0.05 m)

## Usage Examples

```bash
# Test with articulated arm robot (main learning robot)
python examples/simple/basic_simulation.py

# Test individual robots
python examples/urdf/urdf_robot_demo.py 15 examples/robots/articulated_arm_robot.urdf
python examples/urdf/urdf_robot_demo.py 10 examples/robots/collision_robot.urdf  
python examples/urdf/urdf_robot_demo.py 12 examples/robots/mobile_robot.urdf

# Multi-robot scenarios
python examples/simple/all_features_demo.py
```

## Robot Selection Guide

| Robot Type | Use Case | Key Features |
|------------|----------|--------------|
| `articulated_arm_robot.urdf` | Learning, single robot demos | 4 joints, realistic arm structure |
| `collision_robot.urdf` | Multi-robot, collision detection | Collision elements, distinct colors |
| `mobile_robot.urdf` | Mobile robotics | Wheels, sensors, navigation |

## URDF Format Notes

These URDF files use basic geometric primitives:
- **Box**: `<box size="x y z"/>`
- **Cylinder**: `<cylinder radius="r" length="h"/>`
- **Sphere**: `<sphere radius="r"/>`

Colors are defined using RGBA values in the material tags.

Joint types supported:
- **revolute**: Rotational joint with limits
- **continuous**: Rotational joint without limits (wheels)
- **fixed**: No movement (not used in these examples)

## Extending These Models

To add mesh files:
1. Create a `meshes/` subdirectory
2. Add STL, DAE, or OBJ files
3. Reference in URDF: `<mesh filename="package://robot_name/meshes/part.stl"/>`

## Dependencies

To use these URDF files, ensure you have installed:
```bash
pip install urdfpy trimesh pycollada
```