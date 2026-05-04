"""Balance left/right asymmetry in a motion NPZ.

Symmetrizes joint pairs (sagittal joints averaged, lateral joints anti-averaged),
zeros the lateral base offset (Y), and removes the residual base roll bias.
Recomputes FK so all body arrays stay consistent.
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

PROJECT_ROOT = Path(__file__).resolve().parent.parent
M2V6_XML = PROJECT_ROOT / "vm_retargeting" / "assets" / "M2v6" / "M2v6.xml"

# Joints that act in the sagittal plane: L and R should have the SAME value
SAGITTAL = {"hip_pitch", "knee", "ankle_pitch",
            "shoulder_pitch", "elbow", "wrist_pitch"}
# Joints that act laterally: L and R should be OPPOSITE (L = -R)
LATERAL  = {"hip_roll", "hip_yaw", "ankle_roll",
            "shoulder_roll", "shoulder_yaw", "wrist_yaw", "wrist_roll"}


def symmetrize_joints(joint_pos, joint_names):
    out = joint_pos.copy()
    for jn in SAGITTAL | LATERAL:
        l_name = f"left_{jn}_joint"
        r_name = f"right_{jn}_joint"
        if l_name not in joint_names or r_name not in joint_names:
            continue
        li = joint_names.index(l_name)
        ri = joint_names.index(r_name)
        L = joint_pos[:, li]
        R = joint_pos[:, ri]
        if jn in SAGITTAL:
            avg = 0.5 * (L + R)
            out[:, li] = avg
            out[:, ri] = avg
        else:  # LATERAL: mirror — L should equal -R
            half = 0.5 * (L - R)
            out[:, li] = half
            out[:, ri] = -half
    return out


def center_base(base_pos_w, base_quat_w):
    """Zero the lateral (Y) offset and remove residual roll bias."""
    new_pos = base_pos_w.copy()
    y_mean = new_pos[:, 1].mean()
    new_pos[:, 1] -= y_mean

    # base_quat_w is wxyz; convert to xyzw for scipy
    q_xyzw = np.column_stack([base_quat_w[:, 1], base_quat_w[:, 2],
                              base_quat_w[:, 3], base_quat_w[:, 0]])
    eul = R.from_quat(q_xyzw).as_euler("xyz", degrees=False)
    roll_mean = eul[:, 0].mean()
    eul[:, 0] -= roll_mean
    q_new_xyzw = R.from_euler("xyz", eul, degrees=False).as_quat()
    new_quat = np.column_stack([q_new_xyzw[:, 3], q_new_xyzw[:, 0],
                                q_new_xyzw[:, 1], q_new_xyzw[:, 2]])
    print(f"[BAL] base Y offset removed: {y_mean:+.4f} m")
    print(f"[BAL] base roll bias removed: {np.degrees(roll_mean):+.2f}°")
    return new_pos, new_quat


def recompute_fk(model, base_pos, base_quat, joint_pos, joint_names, body_names):
    T = base_pos.shape[0]
    n_bodies = len(body_names)

    mj_jnames = []
    for j in range(model.njnt):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
            mj_jnames.append(model.joint(j).name)
    npz_idx = {n: i for i, n in enumerate(joint_names)}
    jperm = [npz_idx.get(n, -1) for n in mj_jnames]

    mj_bnames = [model.body(b).name for b in range(1, model.nbody)]
    mj_bidx = {n: i for i, n in enumerate(mj_bnames)}
    b_map = [mj_bidx.get(n, -1) for n in body_names]

    out_pos = np.zeros((T, n_bodies, 3))
    out_quat = np.zeros((T, n_bodies, 4))
    out_quat[..., 0] = 1.0
    out_lvel = np.zeros((T, n_bodies, 3))
    out_avel = np.zeros((T, n_bodies, 3))

    data = mujoco.MjData(model)
    for t in range(T):
        if t % 200 == 0:
            print(f"  FK: {t}/{T}", end="\r", flush=True)
        qpos = np.zeros(model.nq)
        qpos[0:3] = base_pos[t]
        qpos[3:7] = base_quat[t]
        for mi, ni in enumerate(jperm):
            if ni >= 0:
                qpos[7 + mi] = joint_pos[t, ni]
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        for npz_i, mj_i in enumerate(b_map):
            if mj_i < 0:
                continue
            bi = mj_i + 1
            out_pos[t, npz_i] = data.xpos[bi]
            out_quat[t, npz_i] = data.xquat[bi]
            out_avel[t, npz_i] = data.cvel[bi, :3]
            out_lvel[t, npz_i] = data.cvel[bi, 3:]
    print(f"  FK: done ({T} frames)     ")
    return out_pos, out_quat, out_lvel, out_avel


def report_asymmetry(jpos, jnames, label):
    print(f"\n--- {label} ---")
    print(f'{"joint":>15} | {"L_mean":>8} {"R_mean":>8} | {"sym_err":>8}')
    for jn in sorted(SAGITTAL | LATERAL):
        l_name = f"left_{jn}_joint"
        r_name = f"right_{jn}_joint"
        if l_name not in jnames:
            continue
        L = jpos[:, jnames.index(l_name)]
        R = jpos[:, jnames.index(r_name)]
        if jn in SAGITTAL:
            err = np.mean(L - R)
        else:
            err = np.mean(L + R)
        print(f"{jn:>15} | {L.mean():8.3f} {R.mean():8.3f} | {err:+8.4f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--xml", default=str(M2V6_XML))
    ap.add_argument("--play", action="store_true")
    args = ap.parse_args()

    print(f"Loading: {args.input}")
    raw = np.load(args.input, allow_pickle=True)
    d = {k: raw[k].copy() for k in raw.files}

    jnames = list(d["joint_names"])
    bnames = list(d["body_names"])
    jpos = d["joint_pos"].astype(np.float64)
    bpos = d["base_pos_w"].astype(np.float64)
    bquat = d["base_quat_w"].astype(np.float64)
    fps = int(d["fps"])

    report_asymmetry(jpos, jnames, "BEFORE")
    print(f"\nBase Y mean: {bpos[:,1].mean():+.4f} m  (should be 0)")

    print("\n[1] Symmetrizing joint pairs...")
    jpos = symmetrize_joints(jpos, jnames)

    print("\n[2] Centering base lateral position and roll...")
    bpos, bquat = center_base(bpos, bquat)

    report_asymmetry(jpos, jnames, "AFTER")
    print(f"\nBase Y mean: {bpos[:,1].mean():+.4f} m")

    model = mujoco.MjModel.from_xml_path(args.xml)
    print("\n[3] Recomputing FK + velocities...")
    bp, bq, blv, bav = recompute_fk(model, bpos, bquat, jpos, jnames, bnames)
    jvel = np.gradient(jpos, 1.0 / fps, axis=0)

    out = {
        "joint_pos": jpos, "joint_vel": jvel,
        "joint_names": d["joint_names"], "body_names": d["body_names"],
        "base_pos_w": bpos, "base_quat_w": bquat,
        "body_pos_w": bp, "body_quat_w": bq,
        "body_lin_vel_w": blv, "body_ang_vel_w": bav,
        "fps": d["fps"], "framerate": d["framerate"],
    }
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.output, **out)
    print(f"\n✓ Saved: {args.output}")

    if args.play:
        play(model, bpos, bquat, jpos, jnames, fps)


def play(model, base_pos, base_quat, joint_pos, joint_names, fps):
    import time, mujoco.viewer
    mj_jnames = []
    for j in range(model.njnt):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
            mj_jnames.append(model.joint(j).name)
    npz_idx = {n: i for i, n in enumerate(joint_names)}
    jperm = [npz_idx.get(n, -1) for n in mj_jnames]
    dt = 1.0 / fps
    T = base_pos.shape[0]
    data = mujoco.MjData(model)
    print(f"\nPlaying {T} frames @ {fps} FPS...")
    with mujoco.viewer.launch_passive(model, data) as v:
        f = 0
        while v.is_running():
            t0 = time.perf_counter()
            qpos = np.zeros(model.nq)
            qpos[0:3] = base_pos[f]
            qpos[3:7] = base_quat[f]
            for mi, ni in enumerate(jperm):
                if ni >= 0:
                    qpos[7 + mi] = joint_pos[f, ni]
            data.qpos[:] = qpos
            mujoco.mj_forward(model, data)
            v.sync()
            s = dt - (time.perf_counter() - t0)
            if s > 0:
                time.sleep(s)
            f = (f + 1) % T


if __name__ == "__main__":
    main()
