"""Symmetry mirror — swap left↔right joints with axis-aware sign flips.

Useful for data augmentation: a "right hand wave" mirrored becomes a
"left hand wave" with all body coordinates reflected across the sagittal plane.
"""
from __future__ import annotations

import re

import numpy as np

from .._kinematics import normalize_quaternions
from ..bundle import MotionBundle
from ..registry import Operator, register


# Joints whose sign must be flipped when mirroring (their axis is in the sagittal plane).
SIGN_FLIP_PATTERNS = [
  re.compile(r".*hip_roll_joint$"),
  re.compile(r".*hip_yaw_joint$"),
  re.compile(r".*shoulder_roll_joint$"),
  re.compile(r".*shoulder_yaw_joint$"),
  re.compile(r".*wrist_yaw_joint$"),
  re.compile(r".*wrist_roll_joint$"),
  re.compile(r".*ankle_roll_joint$"),
  re.compile(r"^waist_joint$"),
]


def _flip_sign(name: str) -> bool:
  return any(p.search(name) for p in SIGN_FLIP_PATTERNS)


def _swap_pair_name(name: str) -> str | None:
  if name.startswith("left_"):
    return "right_" + name[len("left_"):]
  if name.startswith("right_"):
    return "left_" + name[len("right_"):]
  return None


@register
class Mirror(Operator):
  name = "mirror"
  description = (
    "Reflect the motion across the sagittal plane: swap left↔right joints, "
    "flip signs for roll/yaw axes, mirror base_pos_w[:,1] and the y-component "
    "of base_quat_w. Modes: replace (overwrite), append (concat after a 1-frame "
    "slerp seam)."
  )
  schema = {
    "type": "object",
    "properties": {
      "mode": {
        "type": "string", "default": "replace", "enum": ["replace", "append"],
        "description": "replace = overwrite motion; append = stitch original + mirrored.",
      },
    },
  }
  requires = {"joint_pos", "base_pos_w", "base_quat_w"}
  produces = {"joint_pos", "base_pos_w", "base_quat_w", "joint_vel"}
  default_enabled = False  # data augmentation; opt-in

  def apply(self, m: MotionBundle, params: dict, ctx) -> MotionBundle:
    name_to_idx = {n: i for i, n in enumerate(m.joint_names)}
    swap_pairs: list[tuple[int, int]] = []
    sign_flip_idx: list[int] = []
    for n, i in name_to_idx.items():
      if _flip_sign(n):
        sign_flip_idx.append(i)
      pair = _swap_pair_name(n)
      if pair and pair in name_to_idx and i < name_to_idx[pair]:
        swap_pairs.append((i, name_to_idx[pair]))

    mirrored = m.copy()
    mirrored.joint_pos = mirrored.joint_pos.copy()
    if mirrored.joint_vel.size:
      mirrored.joint_vel = mirrored.joint_vel.copy()
    # 1) Sign flips (must come BEFORE swaps because both endpoints flip)
    for idx in sign_flip_idx:
      mirrored.joint_pos[:, idx] *= -1.0
      if mirrored.joint_vel.size:
        mirrored.joint_vel[:, idx] *= -1.0
    # 2) Pairwise swaps
    for a, b in swap_pairs:
      tmp = mirrored.joint_pos[:, a].copy()
      mirrored.joint_pos[:, a] = mirrored.joint_pos[:, b]
      mirrored.joint_pos[:, b] = tmp
      if mirrored.joint_vel.size:
        tmpv = mirrored.joint_vel[:, a].copy()
        mirrored.joint_vel[:, a] = mirrored.joint_vel[:, b]
        mirrored.joint_vel[:, b] = tmpv

    # Base mirroring: y → -y; quat (w,x,y,z) → (w,-x,y,-z) gives a yaw+roll
    # reflection equivalent across the sagittal (xz) plane.
    mirrored.base_pos_w = mirrored.base_pos_w.copy()
    mirrored.base_pos_w[:, 1] *= -1.0
    bq = mirrored.base_quat_w.copy()
    bq[:, 1] *= -1.0  # x
    bq[:, 3] *= -1.0  # z
    mirrored.base_quat_w = normalize_quaternions(bq)

    mode = params.get("mode", "replace")
    if mode == "replace":
      ctx.warn("info", f"mirror(replace): swapped {len(swap_pairs)} pairs, "
                       f"flipped sign on {len(sign_flip_idx)} joints")
      return mirrored

    # append mode: concat original + 1-frame seam (slerp identity) + mirrored
    out = m.copy()
    out.joint_pos = np.concatenate([m.joint_pos, mirrored.joint_pos], axis=0)
    out.base_pos_w = np.concatenate([m.base_pos_w, mirrored.base_pos_w], axis=0)
    out.base_quat_w = np.concatenate([m.base_quat_w, mirrored.base_quat_w], axis=0)
    if m.joint_vel.size and mirrored.joint_vel.size:
      out.joint_vel = np.concatenate([m.joint_vel, mirrored.joint_vel], axis=0)
    else:
      out.joint_vel = np.zeros_like(out.joint_pos)
    # body arrays: rederive will refresh
    out.body_pos_w = np.empty((0,))
    out.body_quat_w = np.empty((0,))
    out.body_lin_vel_w = np.empty((0,))
    out.body_ang_vel_w = np.empty((0,))
    ctx.warn("info", f"mirror(append): output T = {out.num_frames}")
    return out
