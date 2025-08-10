# Unused Files

This directory contains files that are not actively used in the current project but are kept for reference.

## Files

### `test_robot.urdf` - Test URDF File
- **Status**: Superseded by examples/robots/ URDF files
- **Reason**: Better test robots created in examples/robots/
- **Alternatives**: 
  - `examples/robots/simple_robot.urdf` - Basic arm robot
  - `examples/robots/mobile_robot.urdf` - Wheeled robot
  - `examples/robots/rotation_test.urdf` - Multi-color test robot

## Policy

Files are moved here when:
- They're no longer referenced by any active code
- Better alternatives exist
- They were created for temporary testing

## Cleanup

These files can be safely deleted during major cleanups, but are kept for:
- Historical reference
- Potential future use
- Avoiding accidental recreation