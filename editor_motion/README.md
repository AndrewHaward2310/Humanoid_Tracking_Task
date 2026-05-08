# editor_motion v1

Web-based motion editor for humanoid robot motion data. Visualize, fix, smooth, trim, and crop motion files in `.npz` / `.csv` / `.pkl` formats — all in one tool, no install of GPU stack required.

> **Pipeline role:** sits between [`vm_retargeting`](../vm_retargeting) (produces robot motion from human pose) and [`mjlab`](../mjlab) (consumes refined motion for RL training). Use this tool to clean up retarget glitches before training.

![Motion Editor screenshot](docs/screenshot.png)

## Features

- **🎬 3D viewport** — Three.js + URDF loader, sky-gradient backdrop, checkered floor.
- **📈 2D curve editor** — click-and-drag a single channel; range select for batch ops; K1/K2 anchors with linear interpolation.
- **▶️ Playback** — frame slider + time slider (toggleable) + Space to play, ←/→ to step, J/K/L (coming soon).
- **📁 Multi-format I/O** — load and save **`.npz`**, **`.csv`** (vm_soma_retargeter format), **`.pkl`** (vm_retargeting format) interchangeably.
- **📂 Folder Quick-Load** — point the tool at a folder; all `.npz/.csv/.pkl` files appear as one click-to-load list.
- **✂ Trim (single range)** — clip motion in-place or export a single range without touching the editor state.
- **✂✂ Crop (multi-segment)** — mark multiple segments and export them all as one ZIP containing every segment in every selected format (NPZ + CSV + PKL).
- **📥 BVH import** — load a BVH and retarget to the current URDF (requires `soma_retargeter`; gracefully disabled otherwise).
- **🛠 Improvement Pipeline** — chainable operators (`align_origin`, `foot_grounding`, `clamp_joint_limits`, `smooth`, `enforce_kinematic_limits`, `boundary_continuity`, `mirror`, `rederive_kinematics`, `diagnostics`).
- **📊 Stats panel** — render FPS, current frame, time in seconds, motion fps, joint/body counts.
- **🤖 Robot auto-discovery** — drops a URDF folder into `static/<name>/urdf/` and the tool picks it up.
- **↩ Undo/redo** — Ctrl+Z / Ctrl+Y across all edits.

## Quick start

```bash
cd editor_motion
pip install -r requirements.txt          # one-time
python app.py                             # http://127.0.0.1:5002
```

Open the URL, pick a robot from the dropdown → **Load URDF**, then drop a `.npz/.csv/.pkl` into the file picker (or use Folder Quick-Load).

## Workflow: crop a take into multiple training clips

1. Load the source motion (e.g. a long retargeted `.pkl`).
2. Open **Scene Options → ✂✂ Crop (multi-segment)** at bottom-right.
3. Scrub to the start frame of segment 1 → **Set Start (cur)**. Scrub to the end → **Set End (cur)** → **+ Add**. Repeat for each segment.
4. Tick the format checkboxes (NPZ / CSV / PKL — any subset).
5. Set a filename base (default `<input>_cropped`).
6. **Export ZIP** — downloads `<base>_segments.zip` containing `<base>_seg<N>.<ext>` for each segment × format combination.

The cropped curves render as green bands on the curve canvas so you can see the coverage at a glance.

## Format reference

| Format | Translation | Quat order | Schema details |
|--------|-------------|------------|----------------|
| `.npz` | meters | `wxyz` | Editor canonical: `joint_pos`, `base_pos_w`, `base_quat_w`, `joint_names`, `body_pos_w` (opt), `body_quat_w` (opt), `body_names` (opt), `joint_vel` (opt), `body_lin_vel_w` (opt), `body_ang_vel_w` (opt), `fps`/`framerate`. |
| `.csv` | **centimeters** | **`xyzw`** | vm_soma_retargeter format. Header: `Frame, root_translateX/Y/Z, root_quatX/Y/Z/W, <joint_names...>` (joints in radians). |
| `.pkl` | meters | `wxyz` | Crop-UI schema: `motion_root_pos`, `motion_root_rot` (wxyz), `motion_dof_pos`, `motion_fps`, `motion_local_body_pos`, `motion_link_body_list`, `joint_names`, `source_motion`, `source_frame_range`. **Also accepts** raw vm_retargeting schema (`root_pos`, `root_rot` xyzw, `dof_pos`, `fps`, `joint_names`, `link_body_list`, `local_body_pos`) — auto-detected on load. |

Round-tripping any format through editor_motion → save → reload preserves `joint_pos`, `base_pos_w`, `base_quat_w` byte-perfect (modulo CSV's missing fps metadata; CSVs default to 30fps on reload).

## Keyboard shortcuts

| Key | Action |
|-----|--------|
| `Space` | Play / pause |
| `←` `→` | Step ±1 frame |
| `↑` `↓` | Nudge selected channel value |
| `Shift + drag` on curve | Range select |
| `Ctrl + Z` / `Ctrl + Y` | Undo / redo |

## API endpoints

All endpoints are under `http://127.0.0.1:5002`:

| Endpoint | Purpose |
|----------|---------|
| `GET /robots` | Auto-discover URDFs under `static/`. |
| `POST /upload_motion` | Load NPZ via multipart upload. |
| `POST /upload_csv` | Load CSV. |
| `POST /upload_pkl` | Load PKL. |
| `POST /upload_bvh` | Load BVH and retarget (requires `soma_retargeter`). |
| `GET /list_motions?dir=<path>&exts=npz,csv,pkl` | List motion files in a server-side folder. |
| `POST /load_motion_by_path` | Load `.npz/.csv/.pkl` from a server-side path. |
| `POST /save_motion` | Save current motion as NPZ. |
| `POST /save_csv` | Save current motion as CSV. |
| `POST /save_pkl` | Save current motion as PKL. |
| `POST /crop_segments` | Crop multiple segments → ZIP of `seg<N>.{npz,csv,pkl}`. |
| `POST /pipeline/run` | Apply a pipeline of improvement operators. |
| `GET /pipeline/operators` | List available operators with their parameter schemas. |
| `POST /pipeline/diagnostics` | Read-only quality report on a motion. |
| `GET /robot/limits?xml=<MJCF>` | Joint pos/vel/effort limits for curve overlays. |

## Adding a new robot

1. Drop the URDF + meshes under `static/<robot_name>/urdf/<robot>.urdf`.
2. (Optional) Add MJCF at `static/<robot_name>/<robot>.xml` for joint-limit overlays + pipeline FK.
3. Reload the page. The robot appears in the dropdown automatically.

## Adding a new improvement operator

1. Add a class under `motion_pipeline/operators/` implementing the operator protocol (see existing operators).
2. Register it in `motion_pipeline/registry.py`.
3. The frontend Improvement Pipeline section picks it up via `/pipeline/operators`.

## Project structure

```
editor_motion/
├── app.py                       # Flask backend, all endpoints
├── templates/index.html         # Single-file ES6 frontend (3D + 2D + UI)
├── motion_pipeline/             # Modular processing pipeline
│   ├── registry.py              # Operator registry
│   ├── runner.py                # Pipeline executor
│   ├── limits.py                # MJCF joint-limit extractor
│   ├── kinematics.py            # FK helpers (rederive_kinematics)
│   └── operators/*.py           # smooth, foot_grounding, clamp, ...
├── static/<robot>/urdf/         # Robot URDF + meshes (auto-discovered)
├── convert_to_mujoco.py         # Isaac-Lab NPZ → MuJoCo NPZ remap
└── csv_to_npz_m23_edit.py       # CSV motion → NPZ via Isaac Lab
```

## Tech

Backend: Flask, NumPy, MuJoCo (for FK + joint limits). Optional: SciPy (advanced smoothing), `soma_retargeter` (BVH retarget).

Frontend: Three.js v0.160 + urdf-loader v0.12 via CDN importmap. No build step — pure ES6 module.
