# editor_motion — Web-based robot motion editor (Flask + Three.js)

Pipeline role: **upstream** = `vm_retargeting` (produces robot NPZ); **downstream** = `mjlab` (consumes refined NPZ for training).
Folder was previously `robot-motion-editor-v3`, renamed during the 2026-05-04 reorg.

## Quick commands

```bash
cd ~/Documents/Humanoid_Tracking_Task/editor_motion
pip install flask numpy scipy           # one-time
python app.py                            # Flask server on http://127.0.0.1:5000
```

Convert / fix utilities (run from this folder):

```bash
python convert_to_mujoco.py --npz <isaac.npz> --xml static/M2v6/M2v6.xml --out <mujoco.npz>
python csv_to_npz_m23_edit.py            # CSV → NPZ via Isaac Lab simulator
```

## Frequently-used files

- `app.py` — Flask server, routes `/`, `/upload_motion`, `/save_motion`
- `convert_to_mujoco.py` — Isaac Lab NPZ → MuJoCo NPZ (joint/body remap + FK recompute)
- `csv_to_npz_m23_edit.py` — CSV motion → NPZ
- `motion_pipeline/{limits,registry,runner,kinematics}.py` — modular processing pipeline
- `templates/index.html` — single-file ES6 frontend (3D + 2D editor)
- `static/M2v6/m26_constants.py` — M26 robot config (actuators, joint groups, collisions)
- `static/M2v6/M2v6.xml` — primary URDF/XML used by `convert_to_mujoco.py`

## NPZ format (must match)

| Key | Shape | Notes |
|-----|-------|-------|
| `joint_pos` | `(F, J)` | joint angles |
| `base_pos_w` | `(F, 3)` | global `[x, y, z]` |
| `base_quat_w` | `(F, 4)` | quaternion `[w, x, y, z]` |
| `joint_names` | list | match URDF |
| `framerate` | int | default 30 |

Optional: `joint_vel`, `body_pos_w`, `body_quat_w`, `body_lin_vel_w`, `body_ang_vel_w`, `body_names`.

## Cross-refs

- Input: NPZ from `../vm_retargeting/output/*.pkl` (after pkl→npz) or any prior `.npz`.
- Output: refined NPZ consumed by `../mjlab/scripts/train_*.sh` (`MOTION_FILE=...`).

---

## Project Overview (legacy)

Robot Motion Editor is a web-based tool for visualizing, editing, and smoothing robot motion data. It combines a Three.js 3D robot visualizer with a 2D curve editor to modify joint angles, base positions, and base rotations frame-by-frame. Developed by Project Instinct group.

## Running the Application

```bash
pip install flask numpy          # install dependencies (scipy optional, for advanced smoothing)
python app.py                    # starts Flask server on http://127.0.0.1:5000
```

Robot URDF and mesh files must be placed under `static/` before loading.

## Architecture

**Backend** (`app.py`): Flask server with three routes:
- `GET /` — serves the editor UI
- `POST /upload_motion` — parses uploaded `.npz` files, extracts motion arrays (joint_pos, base_pos_w, base_quat_w, etc.), converts NumPy arrays to JSON
- `POST /save_motion` — reconstructs NumPy arrays from edited JSON data, streams `.npz` file back to browser

**Frontend** (`templates/index.html`): Single-file ES6 module application with three integrated systems:
1. **3D Visualization** — Three.js scene with urdf-loader for robot model rendering; updates joint values and base pose per frame
2. **2D Curve Editor** — HTML5 canvas for visualizing/editing motion curves via click-and-drag; unified `getChannelValue()`/`setChannelValue()` API handles joints, base position (XYZ), and base rotation (RPY converted to/from quaternions)
3. **UI Controls** — sidebar for robot/motion loading, timeline slider, channel list, keyboard controls (arrow keys for frame stepping and value adjustment), smooth and save buttons

**Data flow**: NPZ file → Flask parses to JSON → frontend renders 3D + 2D → user edits curves → Flask reconstructs NPZ → download

## NPZ Data Format

Expected keys in `.npz` motion files:
- `joint_pos`: `(num_frames, num_joints)` — joint angles
- `base_pos_w`: `(num_frames, 3)` — global position `[x, y, z]`
- `base_quat_w`: `(num_frames, 4)` — quaternion `[w, x, y, z]`
- `joint_names`: list of strings matching URDF joint names
- `framerate`: int (defaults to 30 if missing)

Optional: `joint_vel`, `body_pos_w`, `body_quat_w`, `body_lin_vel_w`, `body_ang_vel_w`, `body_names`

## Additional Tools

- `convert_to_mujoco.py` — converts edited NPZ (Isaac Lab format) to MuJoCo-compatible format with joint/body name remapping and forward kinematics recomputation
- `csv_to_npz_m23_edit.py` — converts CSV motion files to NPZ using Isaac Lab simulator
- `static/M2v6/m26_constants.py` — M26 robot configuration (actuator params, joint groupings, collision config)

## Frontend Dependencies

Three.js v0.160.0 and urdf-loader v0.12.1 are loaded via CDN importmap in `index.html` — no npm/node build step required.
