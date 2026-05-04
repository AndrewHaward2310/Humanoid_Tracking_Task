"""
Retarget SMPL body_pose from GVHMR output to M2v6 robot CSV format.

SMPL body_pose (axis-angle, 21 joints × 3) → M2v6 joint angles (27 DOF)

CSV format per row:
  [base_pos(3), base_rot_xyzw(4), joint_angles(27)]

SMPL joints → M2v6 mapping:
  SMPL uses axis-angle (3D) per joint.
  M2v6 uses individual 1-DOF revolute joints (pitch/roll/yaw).
  We decompose SMPL 3D rotations into Euler angles matching M2v6 joint axes.
"""

import argparse
import numpy as np
import torch
from scipy.spatial.transform import Rotation as R


# SMPL body_pose indices (0-indexed, each joint = 3 values)
SMPL_JOINT_MAP = {
    "left_hip": 0,       # body_pose[0:3]
    "right_hip": 1,      # body_pose[3:6]
    "spine1": 2,         # body_pose[6:9]
    "left_knee": 3,      # body_pose[9:12]
    "right_knee": 4,     # body_pose[12:15]
    "spine2": 5,         # body_pose[15:18]
    "left_ankle": 6,     # body_pose[18:21]
    "right_ankle": 7,    # body_pose[21:24]
    "spine3": 8,         # body_pose[24:27]
    "left_foot": 9,      # body_pose[27:30]
    "right_foot": 10,    # body_pose[30:33]
    "neck": 11,          # body_pose[33:36]
    "left_collar": 12,   # body_pose[36:39]
    "right_collar": 13,  # body_pose[39:42]
    "head": 14,          # body_pose[42:45]
    "left_shoulder": 15, # body_pose[45:48]
    "right_shoulder": 16,# body_pose[48:51]
    "left_elbow": 17,    # body_pose[51:54]
    "right_elbow": 18,   # body_pose[54:57]
    "left_wrist": 19,    # body_pose[57:60]
    "right_wrist": 20,   # body_pose[60:63]
}

# M2v6 joint order (27 DOF) - must match csv_to_npz_m2v6.py
M2V6_JOINTS = [
    "left_hip_pitch",      # 0
    "left_hip_roll",       # 1
    "left_hip_yaw",        # 2
    "left_knee",           # 3
    "left_ankle_pitch",    # 4
    "left_ankle_roll",     # 5
    "right_hip_pitch",     # 6
    "right_hip_roll",      # 7
    "right_hip_yaw",       # 8
    "right_knee",          # 9
    "right_ankle_pitch",   # 10
    "right_ankle_roll",    # 11
    "waist",               # 12
    "left_shoulder_pitch",  # 13
    "left_shoulder_roll",   # 14
    "left_shoulder_yaw",    # 15
    "left_elbow",           # 16
    "left_wrist_yaw",       # 17
    "left_wrist_pitch",     # 18
    "left_wrist_roll",      # 19
    "right_shoulder_pitch", # 20
    "right_shoulder_roll",  # 21
    "right_shoulder_yaw",   # 22
    "right_elbow",          # 23
    "right_wrist_yaw",      # 24
    "right_wrist_pitch",    # 25
    "right_wrist_roll",     # 26
]


def axis_angle_to_euler(aa, order="YXZ"):
    """Convert axis-angle (3,) to Euler angles with given order."""
    rot = R.from_rotvec(aa)
    return rot.as_euler(order, degrees=False)


def decompose_smpl_joint(body_pose, joint_name):
    """Extract axis-angle for a SMPL joint from body_pose."""
    idx = SMPL_JOINT_MAP[joint_name]
    return body_pose[idx * 3 : idx * 3 + 3]


def retarget_frame(body_pose, global_orient, transl):
    """
    Retarget one frame of SMPL params to M2v6 joint angles.

    Args:
        body_pose: (63,) SMPL body_pose axis-angle
        global_orient: (3,) global orientation axis-angle
        transl: (3,) global translation

    Returns:
        base_pos: (3,) base position
        base_rot_xyzw: (4,) base rotation quaternion (xyzw)
        joint_angles: (27,) M2v6 joint angles
    """
    # Base position: SMPL transl (adjust coordinate frame)
    # SMPL: Y-up, M2v6/MuJoCo: Z-up
    base_pos = np.array([transl[0], transl[1], transl[2] + 0.85])

    # Base rotation: global_orient → quaternion (xyzw)
    rot = R.from_rotvec(global_orient)
    quat_wxyz = rot.as_quat(scalar_first=True)  # wxyz
    base_rot_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])

    joint_angles = np.zeros(27)

    # --- Left hip (pitch/roll/yaw) ---
    lh = axis_angle_to_euler(decompose_smpl_joint(body_pose, "left_hip"), "YXZ")
    joint_angles[0] = lh[0]   # pitch (Y)
    joint_angles[1] = lh[1]   # roll (X)
    joint_angles[2] = lh[2]   # yaw (Z)

    # --- Left knee ---
    lk = decompose_smpl_joint(body_pose, "left_knee")
    lk_euler = axis_angle_to_euler(lk, "YXZ")
    joint_angles[3] = lk_euler[0]  # pitch

    # --- Left ankle (pitch/roll) ---
    la = axis_angle_to_euler(decompose_smpl_joint(body_pose, "left_ankle"), "YXZ")
    joint_angles[4] = la[0]   # pitch
    joint_angles[5] = la[1]   # roll

    # --- Right hip (pitch/roll/yaw) ---
    rh = axis_angle_to_euler(decompose_smpl_joint(body_pose, "right_hip"), "YXZ")
    joint_angles[6] = rh[0]   # pitch
    joint_angles[7] = rh[1]   # roll
    joint_angles[8] = rh[2]   # yaw

    # --- Right knee ---
    rk = decompose_smpl_joint(body_pose, "right_knee")
    rk_euler = axis_angle_to_euler(rk, "YXZ")
    joint_angles[9] = rk_euler[0]  # pitch

    # --- Right ankle (pitch/roll) ---
    ra = axis_angle_to_euler(decompose_smpl_joint(body_pose, "right_ankle"), "YXZ")
    joint_angles[10] = ra[0]  # pitch
    joint_angles[11] = ra[1]  # roll

    # --- Waist (combine spine1 + spine2 + spine3 yaw) ---
    s1 = axis_angle_to_euler(decompose_smpl_joint(body_pose, "spine1"), "ZYX")
    s2 = axis_angle_to_euler(decompose_smpl_joint(body_pose, "spine2"), "ZYX")
    s3 = axis_angle_to_euler(decompose_smpl_joint(body_pose, "spine3"), "ZYX")
    joint_angles[12] = (s1[0] + s2[0] + s3[0]) / 3.0  # average yaw

    # --- Left shoulder (pitch/roll/yaw) ---
    # Combine collar + shoulder
    lc = decompose_smpl_joint(body_pose, "left_collar")
    ls = decompose_smpl_joint(body_pose, "left_shoulder")
    ls_combined = R.from_rotvec(lc) * R.from_rotvec(ls)
    ls_euler = ls_combined.as_euler("YXZ", degrees=False)
    joint_angles[13] = ls_euler[0]  # pitch
    joint_angles[14] = ls_euler[1]  # roll
    joint_angles[15] = ls_euler[2]  # yaw

    # --- Left elbow ---
    le = decompose_smpl_joint(body_pose, "left_elbow")
    le_euler = axis_angle_to_euler(le, "YXZ")
    joint_angles[16] = le_euler[0]  # pitch

    # --- Left wrist (yaw/pitch/roll) ---
    lw = axis_angle_to_euler(decompose_smpl_joint(body_pose, "left_wrist"), "ZYX")
    joint_angles[17] = lw[0]  # yaw (Z)
    joint_angles[18] = lw[1]  # pitch (Y)
    joint_angles[19] = lw[2]  # roll (X)

    # --- Right shoulder (pitch/roll/yaw) ---
    rc = decompose_smpl_joint(body_pose, "right_collar")
    rs = decompose_smpl_joint(body_pose, "right_shoulder")
    rs_combined = R.from_rotvec(rc) * R.from_rotvec(rs)
    rs_euler = rs_combined.as_euler("YXZ", degrees=False)
    joint_angles[20] = rs_euler[0]  # pitch
    joint_angles[21] = rs_euler[1]  # roll
    joint_angles[22] = rs_euler[2]  # yaw

    # --- Right elbow ---
    re = decompose_smpl_joint(body_pose, "right_elbow")
    re_euler = axis_angle_to_euler(re, "YXZ")
    joint_angles[23] = re_euler[0]  # pitch

    # --- Right wrist (yaw/pitch/roll) ---
    rw = axis_angle_to_euler(decompose_smpl_joint(body_pose, "right_wrist"), "ZYX")
    joint_angles[24] = rw[0]  # yaw (Z)
    joint_angles[25] = rw[1]  # pitch (Y)
    joint_angles[26] = rw[2]  # roll (X)

    return base_pos, base_rot_xyzw, joint_angles


def main():
    parser = argparse.ArgumentParser(description="Retarget SMPL → M2v6 CSV")
    parser.add_argument("--input", required=True, help="Path to hmr4d_results.pt from GVHMR")
    parser.add_argument("--output", required=True, help="Output CSV file path")
    parser.add_argument("--use-global", action="store_true", default=True,
                        help="Use global SMPL params (default: True)")
    parser.add_argument("--smooth-window", type=int, default=5,
                        help="Smoothing window size (0 to disable)")
    args = parser.parse_args()

    # Load GVHMR results
    print(f"Loading GVHMR results from: {args.input}")
    data = torch.load(args.input, map_location="cpu", weights_only=False)

    key = "smpl_params_global" if args.use_global else "smpl_params_incam"
    smpl = data[key]
    body_pose = smpl["body_pose"].numpy()    # (N, 63)
    global_orient = smpl["global_orient"].numpy()  # (N, 3)
    transl = smpl["transl"].numpy()          # (N, 3)

    num_frames = body_pose.shape[0]
    print(f"Frames: {num_frames}")
    print(f"Using: {key}")

    # Retarget each frame
    all_rows = []
    for i in range(num_frames):
        base_pos, base_rot_xyzw, joint_angles = retarget_frame(
            body_pose[i], global_orient[i], transl[i]
        )
        row = np.concatenate([base_pos, base_rot_xyzw, joint_angles])
        all_rows.append(row)

    csv_data = np.array(all_rows)  # (N, 34)

    # Smoothing
    if args.smooth_window > 1:
        from scipy.ndimage import uniform_filter1d
        print(f"Applying smoothing (window={args.smooth_window})...")
        # Smooth joint angles only (columns 7+), not base pos/rot
        csv_data[:, 7:] = uniform_filter1d(csv_data[:, 7:], size=args.smooth_window, axis=0)

    # Save
    np.savetxt(args.output, csv_data, delimiter=",", fmt="%.8f")
    print(f"Saved CSV: {args.output}")
    print(f"  Shape: {csv_data.shape} (frames × [pos(3) + rot(4) + joints(27)])")
    print(f"  Joint angle range: [{csv_data[:, 7:].min():.3f}, {csv_data[:, 7:].max():.3f}] rad")


if __name__ == "__main__":
    main()
