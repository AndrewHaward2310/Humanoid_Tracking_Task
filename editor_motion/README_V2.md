# Robot Motion Editor v2

Forked from v1 with productivity features. Runs on **port 5001** so both can
run side-by-side.

```bash
cd archive/robot-motion-editor-v2
pip install flask numpy mujoco scipy   # mujoco needed for floor-check
python app.py                           # http://127.0.0.1:5001
```

## Workflow

1. **Load URDF**: default `M2v6/urdf/Motion2_v2-2.urdf` (under `static/`). Click
   "1. Load URDF".
2. **Load motion**: pick `.npz`. Channels listed in left sidebar; right-side
   **Control panel** appears.
3. **Select channel** (joint or base pos/rot). Curve editor at bottom.
4. **Edit** via any of:
   - Drag on curve (left-click = set value at that frame)
   - Right-side panel: type number, or nudge ± `0.001 / 0.01 / 0.1`
   - Arrow keys: ←/→ step frame · ↑/↓ hold to nudge
5. **Download**: click "Download NPZ".

## Power features

### Range selection (Shift + drag on curve)

Shift+drag across frames to select a range. A toolbar appears above:
- **Smooth**: 5-tap gaussian on the selected range only
- **Interp 2-end**: linear interpolation between first and last frame of range
- **±0.05**: add constant offset to all frames in range
- **Clear**: deselect

### Keyframes (K1 / K2)

1. Navigate to a frame, click **Set K1**
2. Navigate to another frame, click **Set K2**
3. Click **Interpolate K1 → K2 (linear)** — fills all frames between with
   linear interpolation on the current channel

Perfect for "frame 8 should be X, frame 11 should be Y, fill the middle"
workflows without touching every frame.

### Copy / Paste value

- **Copy**: captures the current-frame's value on the current channel
- **Paste**: writes it into the current frame, or into the selected range if
  any is active

### Floor contact check (⚠ red highlight)

1. Ensure `scene_M2v6_with_floor.xml` is set in the "Floor check" field
2. Click **Check Floor Contact** — runs MuJoCo FK on all frames server-side
3. Frames where left/right hand_collision geom is below floor show as a red
   band on the curve. Navigate to those frames and fix.
4. Re-click to re-check after edits.

### Undo / Redo

- `Ctrl+Z` — undo (up to 30 levels)
- `Ctrl+Y` — redo

Every edit operation pushes a snapshot of `joint_pos / base_pos / base_quat`
onto the stack.

## Implementation notes

- Backend: Flask + NumPy + MuJoCo (optional; only needed for floor-check)
- Frontend: Three.js + urdf-loader + raw canvas 2D
- `app.py` caches MuJoCo models by path so repeated floor checks are fast
