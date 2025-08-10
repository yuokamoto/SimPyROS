# PyVista Performance Optimization Notes

## Performance Results

**Benchmark (50 frames, wheeled robot):**
- **Optimized method**: 4,611 FPS (transformation matrix only)
- **Fallback method**: 79 FPS (mesh recreation)
- **Speedup**: 58x faster

## Implementation Details

### Optimized Approach
```python
# Mesh created once during initialization
controller.add_robot("robot", mesh)

# Only update transformation matrix per frame
controller.update_robot_pose("robot", new_pose)  # ✅ Fast
```

**Process:**
1. Store original mesh in memory
2. Update actor's transformation matrix (VTK level)
3. No mesh recreation or actor replacement

### Fallback Approach
```python
# Recreate mesh every frame
def mesh_creator():
    return create_robot_mesh(viz, 'wheeled')

controller.update_robot_pose_fallback("robot", new_pose, mesh_creator)  # ❌ Slow
```

**Process:**
1. Create new mesh from scratch
2. Apply transformation
3. Remove old actor
4. Add new actor

## Usage Guidelines

### Use Optimized Method When:
- ✅ Robot position/rotation changes
- ✅ Robot maintains same shape/structure
- ✅ High frame rate animations
- ✅ Real-time interactive applications

### Use Fallback Method When:
- ⚠️ Robot shape changes (articulated joints)
- ⚠️ Parts added/removed dynamically
- ⚠️ Complex deformations needed
- ⚠️ Infrequent updates (< 1 FPS)

## Code Examples

### Efficient Animation Loop
```python
# Setup (once)
viz = create_interactive_visualizer()
controller = AnimationController(viz.plotter, viz.pv)
robot_mesh = create_robot_mesh(viz, 'wheeled')
controller.add_robot("main_robot", robot_mesh)

# Animation loop (many times per second)
for frame in range(1000):
    pose = calculate_new_pose(frame)
    controller.update_robot_pose("main_robot", pose)  # Fast!
```

### Complex Robot Updates
```python
# When robot structure changes
def create_custom_robot():
    # Complex robot with changing parts
    return custom_mesh

controller.update_robot_pose_fallback("robot", pose, create_custom_robot)
```

## Technical Implementation

### VTK Matrix Update
```python
# Convert SimPyROS pose to VTK matrix
vtk_matrix = vtk.vtkMatrix4x4()
for i in range(4):
    for j in range(4):
        vtk_matrix.SetElement(i, j, transform_matrix[i, j])

# Apply to actor (GPU-level transformation)
actor.SetUserMatrix(vtk_matrix)
```

### Mesh Copy Fallback
```python
# If VTK matrix fails, copy and transform mesh
transformed_mesh = original_mesh.copy()
transformed_mesh.transform(transform_matrix, inplace=True)

# Update mapper data
mapper.SetInputData(transformed_mesh)
mapper.Modified()
```

## Memory Usage

- **Optimized**: Constant memory (1 mesh stored)
- **Fallback**: Linear growth (mesh creation per frame)

## Recommendation

**Default behavior** in `pyvista_visualizer.py` uses the optimized method for best performance. Use fallback only when absolutely necessary for dynamic mesh changes.