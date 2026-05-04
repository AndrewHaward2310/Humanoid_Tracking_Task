"""
Convert an edited_motion.npz (Isaac Lab convention) to a MuJoCo-compatible
qpos/qvel trajectory and optionally play it back in the viewer.

Isaac Lab conventions vs MuJoCo:
  - Quaternion order: both use (w, x, y, z) — no reorder needed.
  - Joint ordering: NPZ stores joints in Isaac Lab enum order which may differ
    from MuJoCo's qpos column order.  We remap by name.
  - body_pos_w order: NPZ body rows follow Isaac Lab body_names order, which
    may differ from MuJoCo body indices.  We remap by name.

Usage:
    python convert_to_mujoco.py \
        --npz edited_motion.npz \
        --xml static/M2v6/M2v6.xml \
        [--out mujoco_motion.npz] \
        [--play]                     # opens MuJoCo passive viewer
"""

import argparse
import sys
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def gaussian_kernel(size, sigma=1):
    x = np.linspace(-size // 2, size // 2, size)
    kernel = np.exp(-0.5 * (x / sigma) ** 2)
    return kernel / kernel.sum()

def gaussian_filter(data, sigma=2.0, size=11):
    kernel = gaussian_kernel(size, sigma)
    pad_size = size // 2
    padded = np.pad(data, pad_size, mode='edge')
    return np.convolve(padded, kernel, mode='valid')

def make_quaternion_continuous(q):
    q_cont = q.copy()
    for i in range(1, len(q_cont)):
        if np.dot(q_cont[i], q_cont[i-1]) < 0:
            q_cont[i] = -q_cont[i]
    return q_cont

def smooth_quaternion(q, sigma=2.0, size=11):
    q_cont = make_quaternion_continuous(q)
    q_smooth = np.zeros_like(q_cont)
    for i in range(4):
        q_smooth[:, i] = gaussian_filter(q_cont[:, i], sigma=sigma, size=size)
    norms = np.linalg.norm(q_smooth, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    return q_smooth / norms


def load_npz(path: str) -> dict:
    raw = np.load(path, allow_pickle=False)
    return {k: raw[k] for k in raw.files}


def get_mujoco_joint_order(model) -> list[str]:
    """Return actuated 1-DOF joint names in MuJoCo qpos order (skips freejoint)."""
    import mujoco
    names = []
    for j in range(model.njnt):
        jtype = model.jnt_type[j]
        if jtype == mujoco.mjtJoint.mjJNT_FREE:
            continue  # floating base — handled separately
        if jtype == mujoco.mjtJoint.mjJNT_HINGE or jtype == mujoco.mjtJoint.mjJNT_SLIDE:
            names.append(model.joint(j).name)
    return names


def get_mujoco_body_order(model) -> list[str]:
    """Return body names in MuJoCo body index order (index 0 = world, skip it)."""
    return [model.body(b).name for b in range(1, model.nbody)]


def remap_joints(npz_joint_names: list[str],
                 mj_joint_names: list[str],
                 joint_pos: np.ndarray,
                 joint_vel: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Reorder joint_pos/vel columns from NPZ order → MuJoCo qpos order.
    Joints present in MuJoCo but missing from NPZ are filled with 0.
    """
    n_frames = joint_pos.shape[0]
    n_mj = len(mj_joint_names)
    out_pos = np.zeros((n_frames, n_mj), dtype=np.float64)
    out_vel = np.zeros((n_frames, n_mj), dtype=np.float64)

    npz_idx = {name: i for i, name in enumerate(npz_joint_names)}
    missing = []
    for mj_i, name in enumerate(mj_joint_names):
        if name in npz_idx:
            npz_i = npz_idx[name]
            out_pos[:, mj_i] = joint_pos[:, npz_i]
            out_vel[:, mj_i] = joint_vel[:, npz_i]
        else:
            missing.append(name)

    if missing:
        print(f"[WARN] {len(missing)} MuJoCo joints not found in NPZ (filled with 0): {missing}")

    extra = [n for n in npz_joint_names if n not in set(mj_joint_names)]
    if extra:
        print(f"[INFO] {len(extra)} NPZ joints not in MuJoCo model (ignored): {extra}")

    return out_pos, out_vel


def remap_bodies(npz_body_names: list[str],
                 mj_body_names: list[str],
                 body_pos_w: np.ndarray,
                 body_quat_w: np.ndarray,
                 body_lin_vel_w: np.ndarray,
                 body_ang_vel_w: np.ndarray
                 ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Reorder body arrays from NPZ order → MuJoCo body index order.
    Missing bodies are filled with zeros.
    body_pos_w  : (T, n_bodies_npz, 3)
    body_quat_w : (T, n_bodies_npz, 4)
    body_lin_vel_w : (T, n_bodies_npz, 3)
    body_ang_vel_w : (T, n_bodies_npz, 3)
    """
    T = body_pos_w.shape[0]
    n_mj = len(mj_body_names)

    out_pos  = np.zeros((T, n_mj, 3),  dtype=np.float64)
    out_quat = np.zeros((T, n_mj, 4),  dtype=np.float64)
    out_quat[..., 0] = 1.0  # identity quaternion default
    out_lvel = np.zeros((T, n_mj, 3),  dtype=np.float64)
    out_avel = np.zeros((T, n_mj, 3),  dtype=np.float64)

    npz_idx = {name: i for i, name in enumerate(npz_body_names)}
    missing = []
    for mj_i, name in enumerate(mj_body_names):
        if name in npz_idx:
            npz_i = npz_idx[name]
            out_pos [:, mj_i] = body_pos_w [:, npz_i]
            out_quat[:, mj_i] = body_quat_w[:, npz_i]
            out_lvel[:, mj_i] = body_lin_vel_w[:, npz_i]
            out_avel[:, mj_i] = body_ang_vel_w[:, npz_i]
        else:
            missing.append(name)

    if missing:
        print(f"[WARN] {len(missing)} MuJoCo bodies not found in NPZ (filled with zeros): {missing}")

    return out_pos, out_quat, out_lvel, out_avel


def build_qpos(base_pos_w: np.ndarray,
               base_quat_w: np.ndarray,
               joint_pos_mj: np.ndarray) -> np.ndarray:
    """
    Assemble MuJoCo qpos = [base_pos(3), base_quat_wxyz(4), joint_pos_mj_order(n)].
    base_quat_w is expected in (w, x, y, z) order (Isaac Lab convention matches MuJoCo).
    """
    return np.concatenate([base_pos_w, base_quat_w, joint_pos_mj], axis=1)


def build_qvel(base_lin_vel_w: np.ndarray,
               base_ang_vel_w: np.ndarray,
               joint_vel_mj: np.ndarray) -> np.ndarray:
    """
    Assemble MuJoCo qvel = [base_lin_vel(3), base_ang_vel(3), joint_vel_mj_order(n)].
    """
    return np.concatenate([base_lin_vel_w, base_ang_vel_w, joint_vel_mj], axis=1)


def recompute_bodies_fk(
    model,
    qpos: np.ndarray,
    qvel: np.ndarray,
    il_body_names: list[str],
    mj_body_names: list[str],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Run mj_forward for every frame and extract body pos/quat/vel from MuJoCo,
    then reorder to Isaac Lab body order (il_body_names).

    Returns arrays in Isaac Lab body order:
      body_pos_w   (T, n_il_bodies, 3)   world-frame position
      body_quat_w  (T, n_il_bodies, 4)   world-frame quaternion (w, x, y, z)
      body_lin_vel (T, n_il_bodies, 3)   world-frame linear velocity
      body_ang_vel (T, n_il_bodies, 3)   world-frame angular velocity
    """
    import mujoco

    T = qpos.shape[0]
    n_il = len(il_body_names)

    out_pos  = np.zeros((T, n_il, 3), dtype=np.float64)
    out_quat = np.zeros((T, n_il, 4), dtype=np.float64)
    out_quat[..., 0] = 1.0
    out_lvel = np.zeros((T, n_il, 3), dtype=np.float64)
    out_avel = np.zeros((T, n_il, 3), dtype=np.float64)

    # Build look-up: Isaac Lab body name → 0-based index into mj_body_names
    # mj_data.xpos[mj_i+1] corresponds to mj_body_names[mj_i]  (xpos[0]=world)
    mj_idx = {name: i for i, name in enumerate(mj_body_names)}
    il_to_mj = [mj_idx.get(name, None) for name in il_body_names]

    missing = [il_body_names[i] for i, v in enumerate(il_to_mj) if v is None]
    if missing:
        print(f"[WARN] FK: {len(missing)} Isaac Lab bodies not in MuJoCo model: {missing}")

    mj_data = mujoco.MjData(model)
    print(f"Running forward kinematics on {T} frames…")
    for t in range(T):
        if t % 100 == 0:
            print(f"  frame {t}/{T}", end="\r", flush=True)
        mj_data.qpos[:] = qpos[t]
        mj_data.qvel[:] = qvel[t]
        mujoco.mj_forward(model, mj_data)
        for il_i, mj_i in enumerate(il_to_mj):
            if mj_i is None:
                continue
            bi = mj_i + 1  # xpos/xquat index (+1 skips world body)
            out_pos [t, il_i] = mj_data.xpos [bi]
            out_quat[t, il_i] = mj_data.xquat[bi]   # MuJoCo stores (w, x, y, z)
            # cvel[bi] = [ang_vel(3), lin_vel(3)] in world/com frame
            out_avel[t, il_i] = mj_data.cvel[bi, 0:3]
            out_lvel[t, il_i] = mj_data.cvel[bi, 3:6]
    print(f"  FK complete.          ")
    return out_pos, out_quat, out_lvel, out_avel


# ---------------------------------------------------------------------------
# Main conversion
# ---------------------------------------------------------------------------

def convert(npz_path: str, xml_path: str, out_path: str | None, 
            smooth_sigma: float = 0.0, 
            smooth_window: int = 11, 
            smooth_base: bool = False, 
            smooth_joints: bool = False) -> dict:
    import mujoco

    print(f"Loading NPZ: {npz_path}")
    data = load_npz(npz_path)

    print(f"Loading MuJoCo model: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)

    mj_joint_names = get_mujoco_joint_order(model)
    mj_body_names  = get_mujoco_body_order(model)

    print(f"  MuJoCo joints ({len(mj_joint_names)}): {mj_joint_names}")
    print(f"  NPZ joints    ({len(data['joint_names'])}): {data['joint_names'].tolist()}")
    print(f"  MuJoCo bodies ({len(mj_body_names)}): {mj_body_names}")
    print(f"  NPZ bodies    ({len(data['body_names'])}): {data['body_names'].tolist()}")

    # -- Remap joints
    joint_pos_mj, joint_vel_mj = remap_joints(
        data['joint_names'].tolist(),
        mj_joint_names,
        np.array(data['joint_pos'], dtype=np.float64),
        np.array(data['joint_vel'], dtype=np.float64),
    )

    base_pos_w  = np.array(data['base_pos_w'],  dtype=np.float64)  # (T, 3)
    base_quat_w = np.array(data['base_quat_w'], dtype=np.float64)  # (T, 4) wxyz

    if smooth_base and smooth_sigma > 0:
        print(f"[INFO] Smoothing base (pos & quat) with sigma={smooth_sigma}, window={smooth_window}...")
        # Smooth base position (X, Y, Z)
        for i in range(3):
            base_pos_w[:, i] = gaussian_filter(base_pos_w[:, i], sigma=smooth_sigma, size=smooth_window)
        
        # Smooth base rotation using proper quaternion smoothing
        base_quat_w = smooth_quaternion(base_quat_w, sigma=smooth_sigma, size=smooth_window)

    if smooth_joints and smooth_sigma > 0:
        print(f"[INFO] Smoothing joints with sigma={smooth_sigma}, window={smooth_window}...")
        # Smooth joint positions and velocities
        for i in range(joint_pos_mj.shape[1]):
            joint_pos_mj[:, i] = gaussian_filter(joint_pos_mj[:, i], sigma=smooth_sigma, size=smooth_window)
            joint_vel_mj[:, i] = gaussian_filter(joint_vel_mj[:, i], sigma=smooth_sigma, size=smooth_window)

    # -- Remap bodies NPZ order → MuJoCo order (needed for pelvis vel extraction)
    body_pos_mj, body_quat_mj, body_lvel_mj, body_avel_mj = remap_bodies(
        data['body_names'].tolist(),
        mj_body_names,
        np.array(data['body_pos_w'],     dtype=np.float64),
        np.array(data['body_quat_w'],    dtype=np.float64),
        np.array(data['body_lin_vel_w'], dtype=np.float64),
        np.array(data['body_ang_vel_w'], dtype=np.float64),
    )

    # Pelvis = index 0 in MuJoCo body order
    base_lin_vel = body_lvel_mj[:, 0]
    base_ang_vel = body_avel_mj[:, 0]

    # -- Build qpos / qvel (for shape validation and playback)
    qpos = build_qpos(base_pos_w, base_quat_w, joint_pos_mj)          # (T, 7+n_joints)
    qvel = build_qvel(base_lin_vel, base_ang_vel, joint_vel_mj)        # (T, 6+n_joints)

    print(f"\nqpos shape: {qpos.shape}  (expected T × {model.nq})")
    print(f"qvel shape: {qvel.shape}  (expected T × {model.nv})")
    if qpos.shape[1] != model.nq:
        print(f"[ERROR] qpos columns {qpos.shape[1]} != model.nq {model.nq}  — check joint count!")
    if qvel.shape[1] != model.nv:
        print(f"[ERROR] qvel columns {qvel.shape[1]} != model.nv {model.nv}  — check joint count!")

    # -- FK recompute: overwrite body arrays with values consistent with edited joint_pos.
    #    recompute_bodies_fk returns arrays in MuJoCo body order (il_body_names=mj_body_names).
    fk_body_pos, fk_body_quat, fk_body_lvel, fk_body_avel = recompute_bodies_fk(
        model, qpos, qvel, mj_body_names, mj_body_names
    )

    # Everything saved in MuJoCo order with original field names.
    result = {
        'joint_names':      np.array(mj_joint_names),   # MuJoCo joint order
        'body_names':       np.array(mj_body_names),    # MuJoCo body order
        'joint_pos':        joint_pos_mj,               # MuJoCo joint order
        'joint_vel':        joint_vel_mj,               # MuJoCo joint order
        'base_pos_w':       base_pos_w,
        'base_quat_w':      base_quat_w,
        'body_pos_w':       fk_body_pos,                # FK-recomputed, MuJoCo body order
        'body_quat_w':      fk_body_quat,               # FK-recomputed, MuJoCo body order
        'body_lin_vel_w':   fk_body_lvel,               # FK-recomputed, MuJoCo body order
        'body_ang_vel_w':   fk_body_avel,               # FK-recomputed, MuJoCo body order
        'fps':              data.get('fps', np.array(30.0)),
        'framerate':        data.get('framerate', np.array(30.0)),
        # Internal only (not saved): for --play
        '_qpos': qpos,
        '_qvel': qvel,
    }

    if out_path:
        # Save only the public (non-underscore) keys — same names as the input
        public = {k: v for k, v in result.items() if not k.startswith('_')}
        np.savez(out_path, **public)
        print(f"\nSaved MuJoCo-compatible NPZ → {out_path}")

    return result, model


# ---------------------------------------------------------------------------
# Optional playback
# ---------------------------------------------------------------------------

def play(result: dict, model) -> None:
    try:
        import mujoco.viewer
    except ImportError:
        print("[ERROR] mujoco.viewer not available — install mujoco>=3.0")
        return

    import mujoco
    import time

    qpos = result['_qpos']
    qvel = result['_qvel']
    fps  = float(np.array(result['fps']).flat[0])
    dt   = 1.0 / fps

    mj_data = mujoco.MjData(model)

    print(f"\nPlaying back {len(qpos)} frames at {fps:.1f} FPS …  (close viewer to exit)")

    with mujoco.viewer.launch_passive(model, mj_data) as viewer:
        for t in range(len(qpos)):
            t_start = time.perf_counter()

            mj_data.qpos[:] = qpos[t]
            mj_data.qvel[:] = qvel[t]
            mujoco.mj_forward(model, mj_data)
            viewer.sync()

            elapsed = time.perf_counter() - t_start
            sleep   = dt - elapsed
            if sleep > 0:
                time.sleep(sleep)

            if not viewer.is_running():
                break

    print("Playback finished.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert Isaac Lab NPZ → MuJoCo qpos trajectory')
    parser.add_argument('--npz',  required=True, help='Input NPZ file (edited_motion.npz)')
    parser.add_argument('--xml',  default='static/M2v6/M2v6.xml', help='MuJoCo XML model path')
    parser.add_argument('--out',  default='mujoco_motion.npz', help='Output NPZ path')
    parser.add_argument('--play', action='store_true', help='Play back in MuJoCo passive viewer after conversion')
    
    # Smoothing arguments
    parser.add_argument('--smooth-sigma', type=float, default=2.0, help='Smoothing sigma (0 to disable)')
    parser.add_argument('--smooth-window', type=int, default=11, help='Smoothing window size')
    parser.add_argument('--no-smooth', action='store_true', help='Disable all smoothing')
    parser.add_argument('--smooth-base', action='store_true', default=None, help='Explicitly enable base smoothing')
    parser.add_argument('--smooth-joints', action='store_true', default=None, help='Explicitly enable joint smoothing')
    
    args = parser.parse_args()

    # Determine what to smooth
    if args.no_smooth or args.smooth_sigma <= 0:
        do_smooth_base = False
        do_smooth_joints = False
    else:
        # If no specific flags are set, smooth everything by default
        if args.smooth_base is None and args.smooth_joints is None:
            do_smooth_base = True
            do_smooth_joints = True
        else:
            do_smooth_base = bool(args.smooth_base)
            do_smooth_joints = bool(args.smooth_joints)

    result, model = convert(
        args.npz, args.xml, args.out, 
        smooth_sigma=args.smooth_sigma, 
        smooth_window=args.smooth_window,
        smooth_base=do_smooth_base,
        smooth_joints=do_smooth_joints
    )

    if args.play:
        play(result, model)
