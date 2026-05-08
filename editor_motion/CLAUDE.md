# editor_motion v1 — Web-based robot motion editor (Flask + Three.js)

Pipeline role: **upstream** = `vm_retargeting` (produces robot motion `.pkl/.npz`); **downstream** = `mjlab` (consumes refined NPZ for training).

## Quick commands

```bash
cd ~/Documents/Humanoid_Tracking_Task/editor_motion
pip install -r requirements.txt          # one-time
python app.py                             # Flask server on http://127.0.0.1:5002
```

## Frequently-used files

- `app.py` — Flask server. Endpoints: `/` UI, `/robots`, `/upload_motion|csv|pkl|bvh`, `/load_motion_by_path`, `/list_motions`, `/save_motion|csv|pkl`, `/crop_segments`, `/pipeline/*`, `/robot/limits`.
- `templates/index.html` — single-file ES6 frontend (3D viewport + 2D curve editor + sidebar + Scene Options panel + Stats panel + playback bar).
- `motion_pipeline/{limits,registry,runner,kinematics}.py` — modular processing pipeline (smooth, foot_grounding, clamp_joint_limits, ...).
- `static/<robot>/urdf/*.urdf` — auto-discovered robot URDFs.
- `static/M2v6/M2v6.xml` — MJCF used for joint-limit overlay + pipeline FK.
- `convert_to_mujoco.py` — Isaac-Lab NPZ → MuJoCo NPZ (joint/body remap + FK recompute).

## Supported motion formats

| Format | Translation | Quat | Notes |
|--------|------|------|-------|
| `.npz` | `base_pos_w` (m) | `base_quat_w` (wxyz) | Editor's canonical schema. |
| `.csv` | cm | xyzw | vm_soma_retargeter format (`Frame, root_translateXYZ, root_quatXYZW, joints`). |
| `.pkl` | m | wxyz on load (xyzw on disk for raw vm_retargeting / wxyz for crop-UI style) | Two schemas detected automatically. |
| `.bvh` | — | — | Loaded via `soma_retargeter` retargeter (optional dep). |

## NPZ canonical schema

| Key | Shape | Notes |
|-----|-------|-------|
| `joint_pos` | `(F, J)` | joint angles, radians |
| `base_pos_w` | `(F, 3)` | world XYZ, meters |
| `base_quat_w` | `(F, 4)` | quaternion `[w, x, y, z]` |
| `joint_names` | list | URDF joint names |
| `fps` / `framerate` | float | default 30 |

Optional: `joint_vel`, `body_pos_w`, `body_quat_w`, `body_lin_vel_w`, `body_ang_vel_w`, `body_names`.

## Cross-refs

- Input: `.pkl` from `../vm_retargeting/output/`, `.csv` from `../vm_soma_retargeter/`, `.npz` from any prior session.
- Output: refined NPZ → `../mjlab/scripts/train_*.sh` (`MOTION_FILE=...`).

## Architecture

**Backend** (`app.py`):
- Loads NPZ/CSV/PKL into a unified JSON payload via `_build_motion_response()`.
- `_load_pkl_to_motion()` auto-detects raw vm_retargeting vs crop_robot_motion_ui schema.
- `_motion_to_pkl_bytes() / _to_csv_bytes() / _to_npz_bytes()` produce identical schemas across formats.
- `/crop_segments` zips multi-segment exports in all 3 formats.

**Frontend** (`templates/index.html`):
1. **3D viewport** — Three.js + urdf-loader; pose-per-frame; sky gradient + checkered floor.
2. **Curve editor** — HTML5 canvas for click-and-drag editing; range select, K1/K2 keyframes, smooth.
3. **Sidebar** — Robot dropdown · Unified file input (3 formats) · Folder Quick-Load · Channels · Channel Editor · Improvement Pipeline.
4. **Stats panel (top-right)** — Render FPS · Frame · Time · Motion FPS · Joints · Bodies.
5. **Scene Options (bottom-right)** — Save NPZ/CSV/PKL · Crop (single segment → file, multi-segment → ZIP) · Import BVH · Visibility toggles.

## Frontend dependencies

Three.js v0.160.0 + urdf-loader v0.12.1 via CDN importmap. No build step.
