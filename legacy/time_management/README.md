# Legacy Time Management Code

This directory contains the original custom time management implementation that was used before migrating to SimPy's RealtimeEnvironment.

## Files

- `custom_time_manager.py` - Original centralized time management system
- `frequency_controller.py` - Custom frequency control logic

## Migration Notes

The custom implementation was replaced with `simpy.rt.RealtimeEnvironment` for:
- Better reliability (official SimPy implementation)
- Reduced maintenance burden
- Standard compliance
- Immediate availability of features

The legacy code is preserved here for reference and potential future custom extensions.