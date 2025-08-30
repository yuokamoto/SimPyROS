# Basic Simulation Optimization Plan

This document captures the proposed optimization actions for `examples/beginner/basic_simulation.py` (a.k.a. basic_example).

## Goals
1. Improve performance for `--vb pyvista` visualization mode.
2. Reduce CPU usage for joint control callbacks and multi‑robot scenarios.
3. Unify logging (avoid `print`) and enhance reproducibility / profiling.
4. Provide a foundation for scalable 100+ robot simulations.

## Quick Wins (Phase 1)
- Replace all `print()` calls in example with structured logging via `logger`.
- Centralize imports (avoid repeated in-function imports of `SimulationConfig`).
- Introduce NumPy vectorization for joint position generation (sinusoidal patterns).
- Add CLI flags:
  - `--offscreen`: sets `PYVISTA_OFF_SCREEN=1` for headless PyVista rendering.
  - `--no-monitor`: disable monitor window regardless of default config.
  - Clarify `--rtf 0.0` meaning (max speed mode using standard SimPy Environment).
- Guard debug logging inside hot loops with `if logger.isEnabledFor(logging.DEBUG)`.
- Minor pause reduction between sequential examples (retain small delay for flushing logs).

## Vectorization Rationale
Current per-joint loop repeatedly calls `math.sin`. For N joints:
```
for i, joint in enumerate(movable_joints):
    pos = amplitude * math.sin(t * freq + i * phase_step)
```
Overhead: Python loop + N C-level calls. Replaced by:
```
phases = cached_phase_array  # shape (N,)
base = t * freq
positions = amplitude * np.sin(base + phases)
for joint, pos in zip(movable_joints, positions):
    set_joint(joint, pos)
```
Benefits: 1 NumPy broadcast op instead of N scalar trig calls. Typical CPU reduction 15–35% for moderate joint counts (>=6) at 10–20 Hz.

## Future (Phase 2+)
- Batch rendering context in visualization loop (ensure single render per frame).
- Frequency grouping auto-enable for higher robot counts (heuristic threshold).
- JSON metrics export (`--metrics-json`).
- Optional callback profiling (`--profile-callback`).
- Joint update delta threshold to skip insignificant transform updates.
- Camera frustum culling for off‑screen robots.

## Advanced (Phase 3)
- Incremental sin/cos recurrence for constant phase steps.
- Instance rendering or composite mappers for static links.
- Numba / Cython acceleration for bulk kinematics.

## Risks / Mitigations
| Risk | Mitigation |
|------|------------|
| Small joint count sees negligible gain | Branch: fallback to scalar path when joint_count < 4 |
| Added NumPy import overhead for tiny runs | Acceptable; warm cost amortized; optional lazy init |
| Logging flood still impacts performance | Use INFO level default; DEBUG only when needed |

## Acceptance Criteria
- Examples run successfully with new flags.
- No regression in existing functionality.
- Measurable callback CPU reduction for multi‑joint scenario (manual profiling note).
- Clear documentation in this plan and updated example help text.

## Implementation Notes
- Cache phase arrays by robot name to avoid reallocation each callback.
- Do not modify core kinematics; optimization is example-level only.
- Keep changes minimal & well-commented for educational clarity.

---
(Generated automatically as a reference plan.)
