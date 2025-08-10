# Robot Models Directory

This directory contains URDF files for testing and demonstration purposes.

## Available Robot Models

### 1. `simple_robot.urdf`
- **Description**: Basic 3-DOF robot with base, arm, and end effector
- **Links**: 3 (base_link, arm_link, end_effector)
- **Joints**: 2 revolute joints
- **Use case**: Basic robotics concepts and joint movement demonstration

**Structure:**
- Base: Orange box (0.6 × 0.4 × 0.2 m)
- Arm: Blue cylinder (radius=0.05m, length=0.6m)
- End effector: Red sphere (radius=0.08m)

### 2. `mobile_robot.urdf`
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
# Test with simple robot
python examples/pyvista/urdf_robot_demo.py 10 examples/robots/simple_robot.urdf

# Test with mobile robot
python examples/pyvista/urdf_robot_demo.py 15 examples/robots/mobile_robot.urdf

# Auto-generate test URDF (default behavior)
python examples/pyvista/urdf_robot_demo.py 5
```

## URDF Format Notes

These URDF files use basic geometric primitives:
- **Box**: `<box size="x y z"/>`
- **Cylinder**: `<cylinder radius="r" length="h"/>`
- **Sphere**: `<sphere radius="r"/>`

Colors are defined using RGBA values in the material tags.

Joint types supported:
- **revolute**: Rotational joint with limits
- **continuous**: Rotational joint without limits
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