"""Reroute arms to behind+below pelvis during frames 510-650.

User requirement (motion: edited_10_balanced.npz):
  Frames 510-650 are the "leaning back to sit" transition. Arms must move
  BEHIND the pelvis and LOWER than hips so that, on continuing the lean,
  hands touch the ground first, then butt, then legs straighten.

Key constraints:
  1. Smooth trajectory — no snap; cosine ease-in-out for target Z.
  2. Avoid arm-torso self-intersection — route arms VIA THE SIDE first
     (target_y offset opens up before pulling Z down), so arms don't sweep
     through the body.
  3. Boundary blends — pre-window (480-510) and post-window (650-680) blend
     joint angles linearly so the rest of the motion is undisturbed.
  4. Per-frame IK is warm-started from previous frame's solution so the
     trajectory in joint space is continuous.
  5. Final Gaussian smoothing on modified arm joints to remove residual
     IK jitter.
"""

import argparse
import math
from pathlib import Path

import mink
import mujoco
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation as R

PROJECT_ROOT = Path(__file__).resolve().parent.parent
M2V6_XML = PROJECT_ROOT / "vm_retargeting" / "assets" / "M2v6" / "M2v6.xml"

ARM_JOINT_NAMES = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
]

# Bodies kept fixed during arm IK (legs + pelvis + torso unchanged)
KEEP_FIXED_BODIES = [
    "pelvis_link", "torso_link",
    "left_hip_roll_link", "right_hip_roll_link",
    "left_knee_link", "right_knee_link",
    "left_ankle_roll_link", "right_ankle_roll_link",
]


def smoothstep(x):
    """Cosine ease-in-out, x in [0,1]."""
    x = max(0.0, min(1.0, x))
    return 0.5 - 0.5 * math.cos(math.pi * x)


def build_qpos(model, base_pos, base_quat, joint_pos, joint_names):
    mj_jnames = []
    for j in range(model.njnt):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
            mj_jnames.append(model.joint(j).name)
    npz_idx = {n: i for i, n in enumerate(joint_names)}
    qpos = np.zeros(model.nq)
    qpos[0:3] = base_pos
    qpos[3:7] = base_quat
    for mi, n in enumerate(mj_jnames):
        ni = npz_idx.get(n, -1)
        if ni >= 0:
            qpos[7 + mi] = joint_pos[ni]
    return qpos, mj_jnames, npz_idx


def solve_arms_behind(
    model, base_pos_w, base_quat_w, joint_pos, joint_names, body_pos_w, body_names,
    fix_start=510, fix_end=650, blend_pre=30, blend_post=30,
    final_dx=-0.30, final_dy=0.25, final_z=0.05,
    max_ik_iter=80, ik_dt=0.02,
):
    """Solve arm IK to move wrists to behind+below pelvis for frames
    [fix_start, fix_end], with blend windows on either side.

    Targets sweep smoothly:
      * Y opens FIRST (cos-ease), then X moves back, then Z drops last.
        This is what avoids torso clipping — arms swing OUT before going
        BEHIND and DOWN.
    """
    n = joint_pos.shape[0]
    fix_start = max(0, fix_start)
    fix_end = min(n, fix_end)
    pre0 = max(0, fix_start - blend_pre)
    post1 = min(n, fix_end + blend_post)

    print(f"[INFO] fix window=[{fix_start},{fix_end}); "
          f"pre-blend=[{pre0},{fix_start}); post-blend=[{fix_end},{post1})")

    # Capture original arm joint values at the boundaries for blending
    orig_arm_at_pre0 = {jn: joint_pos[pre0, joint_names.index(jn)]
                        for jn in ARM_JOINT_NAMES}
    orig_arm_at_post1 = {jn: joint_pos[post1 - 1, joint_names.index(jn)]
                          for jn in ARM_JOINT_NAMES}

    pelvis_i = body_names.index("pelvis_link")
    lwrist_i = body_names.index("left_wrist_roll_link")
    rwrist_i = body_names.index("right_wrist_roll_link")

    # Reference start (frame fix_start) wrist offsets — used for trajectory start
    ref_pelvis = body_pos_w[fix_start, pelvis_i]
    ref_lwrist = body_pos_w[fix_start, lwrist_i]
    ref_rwrist = body_pos_w[fix_start, rwrist_i]
    start_lw_dx = ref_lwrist[0] - ref_pelvis[0]
    start_rw_dx = ref_rwrist[0] - ref_pelvis[0]
    start_lw_dy = ref_lwrist[1] - ref_pelvis[1]  # negative (left=-Y)
    start_rw_dy = ref_rwrist[1] - ref_pelvis[1]  # positive
    start_lw_z  = ref_lwrist[2]
    start_rw_z  = ref_rwrist[2]
    print(f"[INFO] start wrist L: dx={start_lw_dx:+.2f} dy={start_lw_dy:+.2f} z={start_lw_z:.2f}")
    print(f"[INFO] start wrist R: dx={start_rw_dx:+.2f} dy={start_rw_dy:+.2f} z={start_rw_z:.2f}")

    joint_pos_new = joint_pos.copy()

    # ----- 1. Per-frame IK over [fix_start, fix_end) -----
    qpos_warm = None  # carried between frames for continuity
    for t in range(fix_start, fix_end):
        if t % 20 == 0:
            print(f"  IK frame {t}/{fix_end}", end="\r", flush=True)

        # Use previous solved arm joints as warm start (continuity in joint space)
        if qpos_warm is None:
            qpos, _, _ = build_qpos(model, base_pos_w[t], base_quat_w[t],
                                     joint_pos_new[t], joint_names)
        else:
            # Re-build qpos with previous solved joint angles for warmth
            qpos, _, _ = build_qpos(model, base_pos_w[t], base_quat_w[t],
                                     joint_pos_new[t], joint_names)
            # overwrite arm joints with warm values
            mj_jnames = []
            for j in range(model.njnt):
                if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
                    mj_jnames.append(model.joint(j).name)
            for mi, mj_name in enumerate(mj_jnames):
                if mj_name in ARM_JOINT_NAMES:
                    qpos[7 + mi] = qpos_warm[7 + mi]

        # ---- Compute progress alpha ∈ [0,1] across fix window ----
        if fix_end - fix_start <= 1:
            alpha = 1.0
        else:
            alpha = (t - fix_start) / (fix_end - fix_start)

        # Phased schedules: Y opens first, then X back, then Z down.
        #   alpha_y peaks early (full spread by alpha=0.40)
        #   alpha_x ramps mid (full back by alpha=0.75)
        #   alpha_z ramps late (full down by alpha=1.00)
        alpha_y = smoothstep(min(1.0, alpha / 0.40))
        alpha_x = smoothstep(min(1.0, max(0.0, (alpha - 0.10) / 0.65)))
        alpha_z = smoothstep(min(1.0, max(0.0, (alpha - 0.30) / 0.70)))

        pelv = body_pos_w[t, pelvis_i]
        # Left wrist target
        lw_dx = start_lw_dx + (final_dx - start_lw_dx) * alpha_x
        lw_dy = start_lw_dy + (-final_dy - start_lw_dy) * alpha_y  # -Y for left
        lw_z  = start_lw_z + (final_z - start_lw_z) * alpha_z
        l_target = np.array([pelv[0] + lw_dx, pelv[1] + lw_dy, lw_z])
        # Right wrist target
        rw_dx = start_rw_dx + (final_dx - start_rw_dx) * alpha_x
        rw_dy = start_rw_dy + (final_dy - start_rw_dy) * alpha_y  # +Y for right
        rw_z  = start_rw_z + (final_z - start_rw_z) * alpha_z
        r_target = np.array([pelv[0] + rw_dx, pelv[1] + rw_dy, rw_z])

        # ---- Build mink IK ----
        config = mink.Configuration(model)
        config.update(qpos)

        tasks = []
        # Wrist position tasks (orientation low priority — let IK pick natural)
        for body_name, target in [("left_wrist_roll_link", l_target),
                                   ("right_wrist_roll_link", r_target)]:
            bi = model.body(body_name).id
            cur_quat = config.data.xquat[bi].copy()
            wt = mink.FrameTask(
                frame_name=body_name, frame_type="body",
                position_cost=600.0, orientation_cost=5.0, lm_damping=1.0,
            )
            wt.set_target(mink.SE3.from_rotation_and_translation(
                mink.SO3(cur_quat), target,
            ))
            tasks.append(wt)

        # Lock everything else (pelvis, torso, legs)
        for bn in KEEP_FIXED_BODIES:
            bi = model.body(bn).id
            kt = mink.FrameTask(
                frame_name=bn, frame_type="body",
                position_cost=1500.0, orientation_cost=800.0, lm_damping=1.0,
            )
            kt.set_target(mink.SE3.from_rotation_and_translation(
                mink.SO3(config.data.xquat[bi].copy()),
                config.data.xpos[bi].copy(),
            ))
            tasks.append(kt)

        limits = [mink.ConfigurationLimit(model)]
        for _ in range(max_ik_iter):
            vel = mink.solve_ik(config, tasks, ik_dt, "daqp", 5e-1, limits)
            config.integrate_inplace(vel, ik_dt)

        # Extract arm joints, write back
        solved = config.data.qpos.copy()
        mj_jnames = []
        for j in range(model.njnt):
            if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
                mj_jnames.append(model.joint(j).name)
        for mi, mj_name in enumerate(mj_jnames):
            if mj_name in ARM_JOINT_NAMES:
                ni = joint_names.index(mj_name)
                joint_pos_new[t, ni] = solved[7 + mi]
        qpos_warm = solved

    print(f"  IK frames done ({fix_end - fix_start} frames).         ")

    # ----- 1b. Enforce L/R symmetry inside fix window -----
    # Pose is intentionally symmetric ("ngả về đằng sau, hai tay chống đất").
    # Average L/R IK results to remove asymmetry from solver convergence.
    SAGITTAL = {"shoulder_pitch", "elbow", "wrist_pitch"}
    LATERAL  = {"shoulder_roll", "shoulder_yaw", "wrist_yaw", "wrist_roll"}
    for jn in SAGITTAL | LATERAL:
        l_idx = joint_names.index(f"left_{jn}_joint")
        r_idx = joint_names.index(f"right_{jn}_joint")
        L = joint_pos_new[fix_start:fix_end, l_idx].copy()
        R = joint_pos_new[fix_start:fix_end, r_idx].copy()
        if jn in SAGITTAL:
            avg = 0.5 * (L + R)
            joint_pos_new[fix_start:fix_end, l_idx] = avg
            joint_pos_new[fix_start:fix_end, r_idx] = avg
        else:  # LATERAL: L = -R
            half = 0.5 * (L - R)
            joint_pos_new[fix_start:fix_end, l_idx] = half
            joint_pos_new[fix_start:fix_end, r_idx] = -half
    print(f"  Symmetrized {len(SAGITTAL | LATERAL)} arm joint pairs in window.")

    # NOTE: We tested forcing wrist_pitch = -1.2 rad ("flexed") but it raised
    # the wrist Z by ~7cm because curling the wrist tucks wrist_roll_link upward.
    # That moved wrists ABOVE pelvis, opposite of what user wants. So we leave
    # wrist_pitch alone (IK chooses ~0) and let the wrist segment hang naturally.
    # If "back-of-wrist contact" surface needs adjustment, that's a separate fix
    # via the wrist contact-point geometry rather than joint angle.

    # ----- 2. Pre-blend: linear interp from orig at pre0 to solved at fix_start -----
    if blend_pre > 0:
        for t in range(pre0, fix_start):
            a = (t - pre0) / max(1, fix_start - pre0)
            a = smoothstep(a)
            for jn in ARM_JOINT_NAMES:
                ji = joint_names.index(jn)
                joint_pos_new[t, ji] = (1 - a) * orig_arm_at_pre0[jn] + a * joint_pos_new[fix_start, ji]

    # ----- 3. Post-blend: from solved at fix_end-1 back to orig at post1 -----
    if blend_post > 0:
        for t in range(fix_end, post1):
            a = (t - fix_end) / max(1, post1 - fix_end)
            a = smoothstep(a)
            for jn in ARM_JOINT_NAMES:
                ji = joint_names.index(jn)
                joint_pos_new[t, ji] = (1 - a) * joint_pos_new[fix_end - 1, ji] + a * orig_arm_at_post1[jn]

    # ----- 4. Final Gaussian smoothing of arm joints over full affected range -----
    smooth_lo = max(0, pre0 - 5)
    smooth_hi = min(n, post1 + 5)
    for jn in ARM_JOINT_NAMES:
        ji = joint_names.index(jn)
        seg = joint_pos_new[smooth_lo:smooth_hi, ji].copy()
        joint_pos_new[smooth_lo:smooth_hi, ji] = gaussian_filter1d(seg, sigma=1.5, mode="nearest")

    return joint_pos_new


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
    out_quat = np.zeros((T, n_bodies, 4)); out_quat[..., 0] = 1.0
    out_lvel = np.zeros((T, n_bodies, 3))
    out_avel = np.zeros((T, n_bodies, 3))

    data = mujoco.MjData(model)
    for t in range(T):
        if t % 200 == 0:
            print(f"  FK: {t}/{T}", end="\r", flush=True)
        qpos = np.zeros(model.nq)
        qpos[0:3] = base_pos[t]; qpos[3:7] = base_quat[t]
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


def report(label, joint_pos, body_pos_w, body_names, frames=(480, 510, 540, 580, 620, 650, 680)):
    pi = body_names.index("pelvis_link")
    lw = body_names.index("left_wrist_roll_link")
    rw = body_names.index("right_wrist_roll_link")
    ti = body_names.index("torso_link")
    print(f"\n--- {label} ---")
    print(f"{'frame':>6} | {'pelv_z':>6} | {'lW dx':>6} {'lW dy':>6} {'lW z':>6} | "
          f"{'rW dx':>6} {'rW dy':>6} {'rW z':>6} | {'lW-tor_y':>9} {'rW-tor_y':>9}")
    for f in frames:
        if f >= body_pos_w.shape[0]:
            continue
        p = body_pos_w[f, pi]
        l = body_pos_w[f, lw]
        r = body_pos_w[f, rw]
        ty = body_pos_w[f, ti, 1]
        print(f"{f:>6} | {p[2]:>6.3f} | "
              f"{l[0]-p[0]:>+6.2f} {l[1]-p[1]:>+6.2f} {l[2]:>6.3f} | "
              f"{r[0]-p[0]:>+6.2f} {r[1]-p[1]:>+6.2f} {r[2]:>6.3f} | "
              f"{l[1]-ty:>+9.3f} {r[1]-ty:>+9.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--xml", default=str(M2V6_XML))
    ap.add_argument("--fix-start", type=int, default=510)
    ap.add_argument("--fix-end", type=int, default=650)
    ap.add_argument("--blend-pre", type=int, default=30)
    ap.add_argument("--blend-post", type=int, default=30)
    ap.add_argument("--final-dx", type=float, default=-0.30,
                    help="Final wrist X offset relative to pelvis (negative=behind)")
    ap.add_argument("--final-dy", type=float, default=0.25,
                    help="Final wrist Y spread magnitude (each side)")
    ap.add_argument("--final-z", type=float, default=0.05,
                    help="Final wrist world Z height")
    ap.add_argument("--waist-peak", type=float, default=0.55,
                    help="Peak forward waist flexion (rad) in fix window")
    ap.add_argument("--play", action="store_true")
    args = ap.parse_args()

    print(f"Loading: {args.input}")
    raw = np.load(args.input, allow_pickle=True)
    d = {k: raw[k].copy() for k in raw.files}

    jnames = list(d["joint_names"])
    bnames = list(d["body_names"])
    bpos = d["body_pos_w"].astype(np.float64)
    jpos = d["joint_pos"].astype(np.float64)
    base_pos = d["base_pos_w"].astype(np.float64)
    base_quat = d["base_quat_w"].astype(np.float64)
    fps = int(d["fps"])

    model = mujoco.MjModel.from_xml_path(args.xml)

    report("BEFORE", jpos, bpos, bnames)

    # ---- Zero base roll inside fix window (with smooth boundary blends) ----
    # Even with symmetric joints, a roll bias on the pelvis tilts L vs R wrist
    # heights. For a deliberately symmetric pose ("ngả về sau, hai tay chống"),
    # the base roll should be 0 over the window.
    pre0 = max(0, args.fix_start - args.blend_pre)
    post1 = min(jpos.shape[0], args.fix_end + args.blend_post)
    bq_xyzw = np.column_stack([base_quat[:, 1], base_quat[:, 2],
                               base_quat[:, 3], base_quat[:, 0]])
    eul = R.from_quat(bq_xyzw).as_euler("xyz", degrees=False)
    new_eul = eul.copy()
    print(f"\n[BASE-ROLL] window roll mean before: "
          f"{np.degrees(eul[args.fix_start:args.fix_end, 0].mean()):+.2f}°")
    # In window: zero roll
    new_eul[args.fix_start:args.fix_end, 0] = 0.0
    # Pre-blend: smooth from original to 0
    for t in range(pre0, args.fix_start):
        a = (t - pre0) / max(1, args.fix_start - pre0)
        a = smoothstep(a)
        new_eul[t, 0] = (1 - a) * eul[t, 0] + a * 0.0
    # Post-blend: smooth from 0 back to original
    for t in range(args.fix_end, post1):
        a = (t - args.fix_end) / max(1, post1 - args.fix_end)
        a = smoothstep(a)
        new_eul[t, 0] = (1 - a) * 0.0 + a * eul[t, 0]
    new_q_xyzw = R.from_euler("xyz", new_eul, degrees=False).as_quat()
    base_quat = np.column_stack([new_q_xyzw[:, 3], new_q_xyzw[:, 0],
                                  new_q_xyzw[:, 1], new_q_xyzw[:, 2]])
    print(f"[BASE-ROLL] window roll mean after:  "
          f"{np.degrees(new_eul[args.fix_start:args.fix_end, 0].mean()):+.2f}°")

    # NOTE: M2v6 ``waist_joint`` is YAW around Z (axis "0 0 1"), NOT a
    # forward-bend hinge. So the upper body cannot truly "lean forward" via
    # joint changes alone — the only forward tilt option is to pitch the
    # whole base quat (which would also tilt the legs, breaking the seated
    # pose). We therefore SKIP a torso forward-lean here and rely on arm IK
    # alone to get wrists behind+below the pelvis.

    print("\n[1] Solving arm IK with phased target schedule...")
    jpos_new = solve_arms_behind(
        model, base_pos, base_quat, jpos, jnames, bpos, bnames,
        fix_start=args.fix_start, fix_end=args.fix_end,
        blend_pre=args.blend_pre, blend_post=args.blend_post,
        final_dx=args.final_dx, final_dy=args.final_dy, final_z=args.final_z,
    )

    print("\n[2] Recomputing FK...")
    bp, bq, blv, bav = recompute_fk(model, base_pos, base_quat, jpos_new,
                                     jnames, bnames)

    report("AFTER", jpos_new, bp, bnames)

    jvel = np.gradient(jpos_new, 1.0 / fps, axis=0)
    out = {
        "joint_pos": jpos_new, "joint_vel": jvel,
        "joint_names": d["joint_names"], "body_names": d["body_names"],
        "base_pos_w": base_pos, "base_quat_w": base_quat,
        "body_pos_w": bp, "body_quat_w": bq,
        "body_lin_vel_w": blv, "body_ang_vel_w": bav,
        "fps": d["fps"], "framerate": d["framerate"],
    }
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.output, **out)
    print(f"\n✓ Saved: {args.output}")

    if args.play:
        play(model, base_pos, base_quat, jpos_new, jnames, fps)


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
            qpos[0:3] = base_pos[f]; qpos[3:7] = base_quat[f]
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
