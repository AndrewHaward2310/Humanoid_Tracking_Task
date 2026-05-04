"""Fix lying-down motion for M2v6 using per-frame IK with ground constraints.

Uses mink IK solver to re-solve leg joint angles so feet stay on ground,
re-route arm trajectory to avoid torso penetration, and place wrists on
ground for support phase.
"""

import argparse
from pathlib import Path

import mink
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
M2V6_XML = PROJECT_ROOT / "vm_retargeting" / "assets" / "M2v6" / "M2v6.xml"

LEG_JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]

UPPER_BODY_LINKS = [
    "torso_link",
    "left_shoulder_pitch_link", "left_shoulder_roll_link",
    "left_elbow_link", "left_wrist_roll_link",
    "right_shoulder_pitch_link", "right_shoulder_roll_link",
    "right_elbow_link", "right_wrist_roll_link",
]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def gaussian_smooth_1d(data, sigma=2.0, size=11):
    x = np.linspace(-size // 2, size // 2, size)
    k = np.exp(-0.5 * (x / sigma) ** 2)
    k /= k.sum()
    return np.convolve(np.pad(data, size // 2, mode="edge"), k, mode="valid")


def build_qpos(model, base_pos, base_quat, joint_pos, joint_names):
    """Build MuJoCo qpos from NPZ data for one frame."""
    mj_jnames = []
    for j in range(model.njnt):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
            mj_jnames.append(model.joint(j).name)
    npz_idx = {n: i for i, n in enumerate(joint_names)}

    qpos = np.zeros(model.nq)
    qpos[0:3] = base_pos
    qpos[3:7] = base_quat
    for mi, mj_name in enumerate(mj_jnames):
        ni = npz_idx.get(mj_name, -1)
        if ni >= 0:
            qpos[7 + mi] = joint_pos[ni]
    return qpos, mj_jnames, npz_idx


def extract_leg_joints(qpos, mj_jnames, npz_idx, joint_names):
    """Extract only leg joint values from solved qpos back to NPZ order."""
    result = {}
    for mi, mj_name in enumerate(mj_jnames):
        if mj_name in LEG_JOINT_NAMES:
            ni = npz_idx.get(mj_name, -1)
            if ni >= 0:
                result[ni] = qpos[7 + mi]
    return result


# ---------------------------------------------------------------------------
# 0. Lower pelvis trajectory
# ---------------------------------------------------------------------------

def lower_pelvis_for_sitting(
    base_pos_w, fps,
    sit_start_s=5.0, butt_ground_s=10.0, lying_start_s=17.0,
    sitting_pelvis_z=0.18,
):
    """Create a pelvis Z trajectory where the butt actually touches the ground.

    The original motion has pelvis too high (0.43m at t=8s). We ramp it down
    to sitting_pelvis_z (~0.10m) so the butt contacts the ground, then
    transition to lying_pelvis_z for the lying phase.
    """
    n = base_pos_w.shape[0]
    new_base = base_pos_w.copy()

    sit_start = int(sit_start_s * fps)
    butt_ground = int(butt_ground_s * fps)
    lying_start = int(lying_start_s * fps)

    for t in range(sit_start, lying_start):
        orig_z = base_pos_w[t, 2]

        if t < butt_ground:
            # Ramp from original to sitting_pelvis_z
            alpha = (t - sit_start) / max(butt_ground - sit_start, 1)
            alpha = 0.5 * (1 - np.cos(np.pi * alpha))  # cosine ease
            target_z = orig_z * (1 - alpha) + sitting_pelvis_z * alpha
            new_base[t, 2] = min(orig_z, target_z)  # only lower, never raise
        else:
            # Hold at sitting height, tracking original if it goes lower
            new_base[t, 2] = min(orig_z, sitting_pelvis_z)
    # Frames >= lying_start: keep original pelvis trajectory unchanged.
    # The original motion already has feet on the ground in the lying phase.

    # Smooth the Z trajectory across the full array so the lying_start
    # boundary blends gradually from capped → original.
    new_base[:, 2] = gaussian_smooth_1d(new_base[:, 2], sigma=4.0, size=31)

    print(f"[FIX] Pelvis Z: sit→{sitting_pelvis_z}m @ t={butt_ground_s}s, "
          f"natural trajectory restored at t={lying_start_s}s")
    return new_base


# ---------------------------------------------------------------------------
# 1. Leg IK with ground constraint
# ---------------------------------------------------------------------------

def solve_legs_ground_contact(
    model, base_pos_w, base_quat_w, joint_pos, joint_names,
    fps, fix_start_s=5.0, fix_end_s=None, ground_z=0.02, max_ik_iter=80,
    ik_dt=0.02,
):
    """Re-solve leg joints per-frame so feet stay flat on ground.

    For each frame in [fix_start, fix_end]:
      1. Set foot targets at (original X, original Y, ground_z) with flat orientation
      2. Use mink IK to solve for leg joints
      3. Keep upper body joints fixed
    """
    n = joint_pos.shape[0]
    fix_start = int(fix_start_s * fps)
    fix_end = n if fix_end_s is None else min(int(fix_end_s * fps), n)

    # Get standing foot orientation from frame 0 (flat on ground)
    qpos0, mj_jnames, npz_idx = build_qpos(
        model, base_pos_w[0], base_quat_w[0], joint_pos[0], joint_names
    )
    data0 = mujoco.MjData(model)
    data0.qpos[:] = qpos0
    mujoco.mj_forward(model, data0)

    l_foot_id = model.body("left_ankle_roll_link").id
    r_foot_id = model.body("right_ankle_roll_link").id
    flat_quat_l = data0.xquat[l_foot_id].copy()  # standing = flat
    flat_quat_r = data0.xquat[r_foot_id].copy()

    modified_count = 0
    joint_pos_new = joint_pos.copy()

    for t in range(fix_start, fix_end):
        if t % 50 == 0:
            print(f"  Leg IK: frame {t}/{fix_end}", end="\r", flush=True)

        # Build current qpos
        qpos, _, _ = build_qpos(
            model, base_pos_w[t], base_quat_w[t], joint_pos[t], joint_names
        )

        # Check if feet are lifted
        data_check = mujoco.MjData(model)
        data_check.qpos[:] = qpos
        mujoco.mj_forward(model, data_check)
        lf_z = data_check.xpos[l_foot_id, 2]
        rf_z = data_check.xpos[r_foot_id, 2]

        # Skip only when both feet are already close to target ground height.
        # Must NOT skip when feet are underground (negative Z after pelvis lowering).
        if abs(lf_z - ground_z) < 0.015 and abs(rf_z - ground_z) < 0.015:
            continue  # Both feet already at ground level

        # Set up mink configuration
        config = mink.Configuration(model)
        config.update(qpos)

        # Foot targets: current XY, Z = ground, flat orientation
        l_foot_pos = data_check.xpos[l_foot_id].copy()
        r_foot_pos = data_check.xpos[r_foot_id].copy()
        l_foot_pos[2] = ground_z
        r_foot_pos[2] = ground_z

        tasks = []

        # Left foot task (HIGH priority)
        lt = mink.FrameTask(
            frame_name="left_ankle_roll_link", frame_type="body",
            position_cost=1000.0, orientation_cost=500.0, lm_damping=1.0,
        )
        lt.set_target(mink.SE3.from_rotation_and_translation(
            mink.SO3(flat_quat_l), l_foot_pos
        ))
        tasks.append(lt)

        # Right foot task (HIGH priority)
        rt = mink.FrameTask(
            frame_name="right_ankle_roll_link", frame_type="body",
            position_cost=1000.0, orientation_cost=500.0, lm_damping=1.0,
        )
        rt.set_target(mink.SE3.from_rotation_and_translation(
            mink.SO3(flat_quat_r), r_foot_pos
        ))
        tasks.append(rt)

        # Pelvis task (keep position, medium priority)
        pt = mink.FrameTask(
            frame_name="pelvis_link", frame_type="body",
            position_cost=200.0, orientation_cost=100.0, lm_damping=1.0,
        )
        pt.set_target(mink.SE3.from_rotation_and_translation(
            mink.SO3(data_check.xquat[model.body("pelvis_link").id]),
            data_check.xpos[model.body("pelvis_link").id],
        ))
        tasks.append(pt)

        # Upper body tasks (keep in place)
        for body_name in UPPER_BODY_LINKS:
            bi = model.body(body_name).id
            ut = mink.FrameTask(
                frame_name=body_name, frame_type="body",
                position_cost=500.0, orientation_cost=200.0, lm_damping=1.0,
            )
            ut.set_target(mink.SE3.from_rotation_and_translation(
                mink.SO3(data_check.xquat[bi]), data_check.xpos[bi],
            ))
            tasks.append(ut)

        # Solve IK — use ik_dt (larger than model.opt.timestep) for faster convergence
        limits = [mink.ConfigurationLimit(model)]
        for _ in range(max_ik_iter):
            vel = mink.solve_ik(config, tasks, ik_dt, "daqp", 5e-1, limits)
            config.integrate_inplace(vel, ik_dt)

        # Extract ONLY leg joints (keep original upper body)
        solved_qpos = config.data.qpos.copy()
        leg_updates = extract_leg_joints(solved_qpos, mj_jnames, npz_idx, joint_names)
        for ni, val in leg_updates.items():
            joint_pos_new[t, ni] = val

        modified_count += 1

    print(f"  Leg IK: modified {modified_count}/{fix_end - fix_start} frames")
    return joint_pos_new


# ---------------------------------------------------------------------------
# 2. Arm collision avoidance
# ---------------------------------------------------------------------------

def fix_arm_clearance(
    joint_pos, joint_names, body_pos_w, body_names,
    fps, min_clearance=0.28, fix_start_s=4.5, fix_end_s=17.0,
):
    """Push arms away from torso by increasing shoulder_roll AND elbow when too close."""
    n = joint_pos.shape[0]
    fix_start = int(fix_start_s * fps)
    fix_end = min(int(fix_end_s * fps), n)

    torso_idx = body_names.index("torso_link")
    l_wrist_idx = body_names.index("left_wrist_roll_link")
    r_wrist_idx = body_names.index("right_wrist_roll_link")
    l_elbow_idx = body_names.index("left_elbow_link")
    r_elbow_idx = body_names.index("right_elbow_link")
    l_sr_idx = joint_names.index("left_shoulder_roll_joint")
    r_sr_idx = joint_names.index("right_shoulder_roll_joint")

    joint_pos_new = joint_pos.copy()
    push_factor = 5.0  # how strongly shoulder_roll responds to deficit

    for t in range(fix_start, fix_end):
        torso_y = body_pos_w[t, torso_idx, 1]

        # Left wrist+elbow clearance — take the worst (smallest |Y|) of the two
        lw_y = body_pos_w[t, l_wrist_idx, 1] - torso_y
        le_y = body_pos_w[t, l_elbow_idx, 1] - torso_y
        l_min = min(abs(lw_y), abs(le_y))
        if l_min < min_clearance:
            deficit = min_clearance - l_min
            joint_pos_new[t, l_sr_idx] += deficit * push_factor

        # Right side
        rw_y = body_pos_w[t, r_wrist_idx, 1] - torso_y
        re_y = body_pos_w[t, r_elbow_idx, 1] - torso_y
        r_min = min(abs(rw_y), abs(re_y))
        if r_min < min_clearance:
            deficit = min_clearance - r_min
            joint_pos_new[t, r_sr_idx] -= deficit * push_factor

    # Smooth shoulder_roll transitions
    for idx in [l_sr_idx, r_sr_idx]:
        joint_pos_new[:, idx] = gaussian_smooth_1d(
            joint_pos_new[:, idx], sigma=3.0, size=21
        )

    modified = np.sum(np.any(joint_pos_new[:, [l_sr_idx, r_sr_idx]] != joint_pos[:, [l_sr_idx, r_sr_idx]], axis=1))
    print(f"[FIX] Arm clearance: modified {modified} frames (t={fix_start_s}→{fix_end_s}s, "
          f"min_clearance={min_clearance}m, push={push_factor})")
    return joint_pos_new


# ---------------------------------------------------------------------------
# 2b. Arm IK for wrist ground placement
# ---------------------------------------------------------------------------

def solve_wrist_ground_ik(
    model, base_pos_w, base_quat_w, joint_pos, joint_names,
    fps, support_start_s=8.0, support_end_s=16.0,
    wrist_ground_z=0.03, max_ik_iter=60, ik_dt=0.02,
):
    """Use mink IK to solve arm joints so wrists reach ground for support.

    Sets wrist targets on the ground (behind pelvis) and solves for
    shoulder/elbow angles. Keeps leg joints and pelvis fixed.
    """
    n = joint_pos.shape[0]
    start = int(support_start_s * fps)
    end = min(int(support_end_s * fps), n)

    ARM_JOINT_NAMES = [
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint", "left_elbow_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint", "right_elbow_joint",
    ]

    # Get standing wrist orientation
    qpos0, mj_jnames, npz_idx = build_qpos(
        model, base_pos_w[0], base_quat_w[0], joint_pos[0], joint_names
    )
    data0 = mujoco.MjData(model)
    data0.qpos[:] = qpos0
    mujoco.mj_forward(model, data0)
    l_wrist_id = model.body("left_wrist_roll_link").id
    r_wrist_id = model.body("right_wrist_roll_link").id

    # Bodies to keep fixed (legs + pelvis + torso)
    KEEP_FIXED = [
        "pelvis_link",
        "left_ankle_roll_link", "right_ankle_roll_link",
        "left_knee_link", "right_knee_link",
    ]

    joint_pos_new = joint_pos.copy()
    modified = 0

    for t in range(start, end):
        if t % 50 == 0:
            print(f"  Arm IK: frame {t}/{end}", end="\r", flush=True)

        qpos, _, _ = build_qpos(
            model, base_pos_w[t], base_quat_w[t], joint_pos_new[t], joint_names
        )

        # Check current wrist Z
        data_chk = mujoco.MjData(model)
        data_chk.qpos[:] = qpos
        mujoco.mj_forward(model, data_chk)
        lw_z = data_chk.xpos[l_wrist_id, 2]
        rw_z = data_chk.xpos[r_wrist_id, 2]

        if max(lw_z, rw_z) < wrist_ground_z + 0.05:
            continue  # Already on ground

        config = mink.Configuration(model)
        config.update(qpos)

        tasks = []

        # Wrist ground targets
        for wid, name in [(l_wrist_id, "left_wrist_roll_link"),
                          (r_wrist_id, "right_wrist_roll_link")]:
            wrist_pos = data_chk.xpos[wid].copy()
            wrist_pos[2] = wrist_ground_z
            wt = mink.FrameTask(
                frame_name=name, frame_type="body",
                position_cost=800.0, orientation_cost=50.0, lm_damping=1.0,
            )
            wt.set_target(mink.SE3.from_rotation_and_translation(
                mink.SO3(data_chk.xquat[wid]), wrist_pos
            ))
            tasks.append(wt)

        # Keep legs + pelvis fixed
        for body_name in KEEP_FIXED:
            bi = model.body(body_name).id
            kt = mink.FrameTask(
                frame_name=body_name, frame_type="body",
                position_cost=1000.0, orientation_cost=500.0, lm_damping=1.0,
            )
            kt.set_target(mink.SE3.from_rotation_and_translation(
                mink.SO3(data_chk.xquat[bi]), data_chk.xpos[bi],
            ))
            tasks.append(kt)

        # Solve — use ik_dt for faster convergence
        limits = [mink.ConfigurationLimit(model)]
        for _ in range(max_ik_iter):
            vel = mink.solve_ik(config, tasks, ik_dt, "daqp", 5e-1, limits)
            config.integrate_inplace(vel, ik_dt)

        # Extract ONLY arm joints
        solved = config.data.qpos.copy()
        for mi, mj_name in enumerate(mj_jnames):
            if mj_name in ARM_JOINT_NAMES:
                ni = npz_idx.get(mj_name, -1)
                if ni >= 0:
                    joint_pos_new[t, ni] = solved[7 + mi]
        modified += 1

    print(f"  Arm IK: modified {modified}/{end - start} frames")
    return joint_pos_new


# ---------------------------------------------------------------------------
# 3. Wrist ground placement
# ---------------------------------------------------------------------------

def fix_wrist_support(
    joint_pos, joint_names, fps,
    wrist_pitch_target=-1.2,
    ramp_in_s=7.5, hold_start_s=9.0, hold_end_s=16.0, ramp_out_end_s=18.0,
):
    """Flex wrist_pitch for wrist-only ground contact during support phase."""
    n = joint_pos.shape[0]
    joint_pos_new = joint_pos.copy()
    l_idx = joint_names.index("left_wrist_pitch_joint")
    r_idx = joint_names.index("right_wrist_pitch_joint")

    ramp_in = int(ramp_in_s * fps)
    hold_start = int(hold_start_s * fps)
    hold_end = int(hold_end_s * fps)
    ramp_out = min(int(ramp_out_end_s * fps), n)

    for idx in [l_idx, r_idx]:
        orig = joint_pos[:, idx].copy()

        # Ramp in
        n_in = hold_start - ramp_in
        if n_in > 0:
            blend = 0.5 * (1 - np.cos(np.pi * np.arange(n_in) / max(n_in - 1, 1)))
            for i, f in enumerate(range(ramp_in, hold_start)):
                joint_pos_new[f, idx] = orig[f] * (1 - blend[i]) + wrist_pitch_target * blend[i]

        # Hold
        joint_pos_new[hold_start:hold_end, idx] = wrist_pitch_target

        # Ramp out
        n_out = ramp_out - hold_end
        if n_out > 0:
            blend = 0.5 * (1 - np.cos(np.pi * np.arange(n_out) / max(n_out - 1, 1)))
            end_val = orig[min(ramp_out, n - 1)]
            for i, f in enumerate(range(hold_end, ramp_out)):
                joint_pos_new[f, idx] = wrist_pitch_target * (1 - blend[i]) + end_val * blend[i]

    print(f"[FIX] Wrist pitch: {wrist_pitch_target:.2f} rad (t={ramp_in_s}→{ramp_out_end_s}s)")
    return joint_pos_new


# ---------------------------------------------------------------------------
# FK recompute
# ---------------------------------------------------------------------------

def recompute_fk(model, base_pos, base_quat, joint_pos, joint_vel, joint_names, body_names):
    """Run mj_forward per frame, return body arrays in NPZ body order."""
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


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

def analyse(joint_pos, body_pos, joint_names, body_names, base_pos, fps, label=""):
    n = joint_pos.shape[0]
    li = body_names.index("left_ankle_roll_link")
    ri = body_names.index("right_ankle_roll_link")
    lwi = body_names.index("left_wrist_roll_link")
    rwi = body_names.index("right_wrist_roll_link")
    ti = body_names.index("torso_link")

    print(f"\n{'='*65}\n  {label}\n{'='*65}")
    print(f"{'t':>5} | {'plv_z':>6} | {'Lft_z':>6} {'Rft_z':>6} | {'Lwst_z':>6} {'Rwst_z':>6}")
    for s in range(0, int(n / fps) + 1, 2):
        f = min(s * fps, n - 1)
        print(f"{s:4d}s | {base_pos[f,2]:6.3f} | {body_pos[f,li,2]:6.3f} {body_pos[f,ri,2]:6.3f} "
              f"| {body_pos[f,lwi,2]:6.3f} {body_pos[f,rwi,2]:6.3f}")

    max_lf = body_pos[:, li, 2].max()
    max_rf = body_pos[:, ri, 2].max()
    lifted = np.where((body_pos[:, li, 2] > 0.10) | (body_pos[:, ri, 2] > 0.10))[0]
    if len(lifted):
        print(f"\n  ⚠ Foot >10cm: {lifted[0]}-{lifted[-1]} (t={lifted[0]/fps:.1f}-{lifted[-1]/fps:.1f}s)")
    else:
        print(f"\n  ✓ No foot lifting >10cm")
    print(f"  Max foot Z: L={max_lf:.4f} R={max_rf:.4f}")

    clearances = []
    for f in range(n):
        ty = body_pos[f, ti, 1]
        c = min(abs(body_pos[f, lwi, 1] - ty), abs(body_pos[f, rwi, 1] - ty))
        clearances.append(c)
    print(f"  Min wrist-torso clearance: {min(clearances):.4f}m")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

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
    jvel = d["joint_vel"].astype(np.float64)
    bpos_w = d["base_pos_w"].astype(np.float64)
    bquat_w = d["base_quat_w"].astype(np.float64)
    body_pos = d["body_pos_w"].astype(np.float64)
    fps = int(d["fps"])
    n = jpos.shape[0]
    print(f"Frames: {n}, FPS: {fps}, Duration: {n/fps:.1f}s")

    model = mujoco.MjModel.from_xml_path(args.xml)

    # Analyse before
    analyse(jpos, body_pos, jnames, bnames, bpos_w, fps, "BEFORE")

    # Step 0: Lower pelvis trajectory (butt touches ground)
    print("\n[Step 0] Lowering pelvis for sitting...")
    bpos_w = lower_pelvis_for_sitting(bpos_w, fps)

    # Step 1: Leg IK with ground constraint (only over the modified phase)
    print("\n[Step 1] Solving leg IK with ground constraint...")
    jpos = solve_legs_ground_contact(
        model, bpos_w, bquat_w, jpos, jnames, fps, ground_z=0.025,
        fix_end_s=17.0,  # Stop at lying_start — original motion handles the lying phase
    )

    # Step 2a: Arm collision avoidance
    print("\n[Step 2a] Fixing arm clearance...")
    print("  Computing intermediate FK...")
    body_pos_tmp, _, _, _ = recompute_fk(
        model, bpos_w, bquat_w, jpos, jvel, jnames, bnames
    )
    jpos = fix_arm_clearance(jpos, jnames, body_pos_tmp, bnames, fps)

    # Step 2b: Arm IK for wrist ground placement
    print("\n[Step 2b] Solving arm IK for wrist ground contact...")
    jpos = solve_wrist_ground_ik(model, bpos_w, bquat_w, jpos, jnames, fps)

    # Step 3: Wrist pitch for proper contact
    print("\n[Step 3] Fixing wrist pitch...")
    jpos = fix_wrist_support(jpos, jnames, fps)

    # Step 4: Clamp to joint limits
    print("\n[Step 4] Clamping joints...")
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        name = model.joint(j).name
        if name not in jnames:
            continue
        idx = jnames.index(name)
        lo, hi = model.jnt_range[j]
        jpos[:, idx] = np.clip(jpos[:, idx], lo, hi)

    # Step 5: Smooth modified joints
    print("\n[Step 5] Smoothing...")
    for name in LEG_JOINT_NAMES:
        if name in jnames:
            idx = jnames.index(name)
            jpos[:, idx] = gaussian_smooth_1d(jpos[:, idx], sigma=2.0, size=11)

    # Step 6: Recompute velocities + FK
    print("\n[Step 6] Recomputing velocities and FK...")
    jvel = np.gradient(jpos, 1.0 / fps, axis=0)
    bp, bq, blv, bav = recompute_fk(model, bpos_w, bquat_w, jpos, jvel, jnames, bnames)

    # Analyse after
    analyse(jpos, bp, jnames, bnames, bpos_w, fps, "AFTER")

    # Save
    out = {
        "joint_pos": jpos, "joint_vel": jvel,
        "joint_names": d["joint_names"], "body_names": d["body_names"],
        "base_pos_w": bpos_w, "base_quat_w": bquat_w,
        "body_pos_w": bp, "body_quat_w": bq,
        "body_lin_vel_w": blv, "body_ang_vel_w": bav,
        "fps": d["fps"], "framerate": d["framerate"],
    }
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.output, **out)
    print(f"\n✓ Saved: {args.output}")

    if args.play:
        play_motion(model, bpos_w, bquat_w, jpos, jnames, fps)


def play_motion(model, base_pos, base_quat, joint_pos, joint_names, fps):
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
