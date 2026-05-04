# Robot Motion Editor v3

Adds an **Improvement Pipeline** on top of v2. After (or instead of)
hand-editing curves, the user runs an ordered list of operators that
clamp / smooth / re-derive / diagnose the motion so what gets saved is
joint-limit-safe and FK-consistent. Runs on **port 5002** so v1/v2/v3 coexist.

```bash
cd editor_motion
pip install -r requirements.txt
python app.py                           # http://127.0.0.1:5002
```

## Pipeline overview

The sidebar gains an "Improvement Pipeline" section listing every registered
operator. Each row has an enable checkbox, a name, a `⚙` to edit params, and
a status pill that fills in after a run. Three buttons:

- **Preview** — runs the enabled operators against a *backup of the current
  motion* and swaps the result into the editor (without touching undo). The
  banner at the top of the section indicates preview mode.
- **Apply** — commits the preview onto the undo stack so it becomes a real
  edit. If no preview is active, this is "Preview + Apply" in one click.
- **Reset** — discards the preview, restores the motion you had before.

### Built-in operators (Phase 1)

| Order | Name | What it does |
|------:|------|--------------|
| 1 | `clamp_joint_limits` | Clamps each joint to `[lo + (1−f)/2·range, hi − (1−f)/2·range]` from MJCF, soft factor `f=0.9` (matches `m26_constants.py:188`). |
| 2 | `smooth` | Manifold-aware Savitzky-Golay (default win=11, poly=3) on joints + base position; SO(3) log-map smoothing for `base_quat_w`. Also supports Butterworth `filtfilt` and Gaussian. |
| 3 | `rederive_kinematics` | "Calibrate" step: re-runs `mj_forward` per frame to recompute `body_pos_w`, `body_quat_w`, `body_lin_vel_w`, `body_ang_vel_w` from the edited `joint_pos` + `base_*`. Recomputes `joint_vel` via central finite-difference. Auto-runs at the end of any pipeline that touches `joint_pos` / `base_*`. |
| 4 | `diagnostics` | Read-only quality report — per-joint vel/acc/jerk max & p99, joint-limit violations, base-z range, foot-slip metric. Always runs before & after to populate the diagnostics panel. |

Operators not enabled by default are listed but unchecked — flip the checkbox
and click `⚙` to configure.

### Curve-canvas overlays

When the active channel is a joint, dashed red lines mark the hard MJCF
position limits and lighter orange lines mark the soft-clamp targets so you
can see at a glance whether your edit is in range.

## Public HTTP API

```
GET  /robot/limits?xml=...                 → joint pos/vel/effort limits parsed from MJCF
POST /pipeline/operators                   → operator catalog (schema, defaults, requires/produces)
POST /pipeline/run                         → execute pipeline; body { motion, xml_path, operators, return_diff }
                                            ↳ returns { motion, diagnostics:{before,after}, audit, new_num_frames }
POST /pipeline/diagnostics                 → diagnostics-only fast path
```

## Architecture (backend)

```
motion_pipeline/
├── bundle.py            MotionBundle dataclass, JSON ↔ numpy round-trip
├── _kinematics.py       FK loop, joint/body remap, quaternion helpers (extracted from convert_to_mujoco.py)
├── limits.py            extract_limits(model) reads MJCF + limits_m26.yaml overrides
├── registry.py          @register decorator, Operator ABC, metadata catalog
├── runner.py            PipelineCtx + run(bundle, ops, xml_path); auto-injects rederive_kinematics
├── limits_m26.yaml      vmax / acc_max / effort_max overrides keyed by joint regex
└── operators/
    ├── clamp_joint_limits.py
    ├── smooth.py
    ├── rederive_kinematics.py
    └── diagnostics.py
```

The runner enforces an "auto-rederive" contract: any operator whose `produces`
intersects `{joint_pos, base_pos_w, base_quat_w}` causes `rederive_kinematics`
to run last (unless you explicitly placed it last yourself). This guarantees
`body_*` and `joint_vel` always stay consistent with edited primary fields.

## Workflow (everything from v2 still works)

Original v2 features — channel editing, drag-on-curve, range select, K1/K2
interpolation, copy/paste, floor-contact check, trim, undo/redo — are
unchanged. The pipeline is purely additive.

## Roadmap

- **Phase 2** — `align_origin`, `foot_grounding`, `pad_safe_pose`,
  `boundary_continuity`. Sim-to-real essentials.
- **Phase 3** — `enforce_kinematic_limits` (vel/acc cap with per-joint
  Butterworth fallback), foot-slip metric, jerk diagnostics, tick overlays
  on the curve canvas at violating frames.
- **Phase 4** — `mirror` data augmentation, drag-reorder operators,
  save/load named pipeline presets, A/B compare overlay.
