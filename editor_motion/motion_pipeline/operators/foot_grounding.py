"""Shift base_z so the lowest foot point sits on the floor (z=0)."""
from __future__ import annotations

import re
from typing import Any

import numpy as np

from .._kinematics import lowest_z_for_geom, run_mj_forward_per_frame
from ..bundle import MotionBundle
from ..registry import Operator, register


def _foot_geom_ids(model: Any, pattern: str) -> list[int]:
  import mujoco
  rx = re.compile(pattern)
  out: list[int] = []
  for gid in range(model.ngeom):
    name = model.geom(gid).name or ""
    if rx.match(name):
      out.append(gid)
  return out


def _per_frame_min_foot_z(model: Any, m: MotionBundle, foot_pattern: str) -> np.ndarray | None:
  import mujoco
  gids = _foot_geom_ids(model, foot_pattern)
  if not gids:
    return None
  data = mujoco.MjData(model)
  T = m.num_frames
  out = np.zeros(T, dtype=np.float64)
  # Build joint_pos in MuJoCo qpos order using the helper
  from .._kinematics import get_mujoco_joint_order, remap_joint_array
  mj_joints = get_mujoco_joint_order(model)
  jp_mj = remap_joint_array(m.joint_names, mj_joints, m.joint_pos)
  qpos_buf = np.zeros(model.nq, dtype=np.float64)
  for t in range(T):
    qpos_buf[0:3] = m.base_pos_w[t]
    qpos_buf[3:7] = m.base_quat_w[t]
    qpos_buf[7:7 + len(mj_joints)] = jp_mj[t]
    data.qpos[:] = qpos_buf
    mujoco.mj_forward(model, data)
    z = min(lowest_z_for_geom(model, data, gid) for gid in gids)
    out[t] = z
  return out


@register
class FootGrounding(Operator):
  name = "foot_grounding"
  description = (
    "Shift base_pos_w[:,2] so the lowest foot collision geom sits on the floor (z=0). "
    "Three modes: first_frame (use frame 0 as reference), min_over_window "
    "(use min over first K frames), per_frame_clamp (apply per-frame correction; "
    "follow this with a base_pos smooth pass to avoid jitter)."
  )
  schema = {
    "type": "object",
    "properties": {
      "mode":    {"type": "string", "default": "min_over_window",
                   "enum": ["first_frame", "min_over_window", "per_frame_clamp"]},
      "window":  {"type": "integer", "default": 5, "minimum": 1, "maximum": 50},
      "target_z": {"type": "number", "default": 0.0,
                   "description": "Floor height the lowest foot should sit on."},
      "foot_geom_pattern": {"type": "string",
                              "default": r"^(left|right)_foot([0-9]+|_collision)?_collision$",
                              "description": "Regex matching foot collision geom names."},
    },
  }
  requires = {"base_pos_w", "joint_pos", "base_quat_w"}
  produces = {"base_pos_w"}

  def apply(self, m: MotionBundle, params: dict, ctx) -> MotionBundle:
    if ctx.model is None:
      ctx.warn("warn", "foot_grounding: no MuJoCo model loaded; skipping")
      return m
    pattern = str(params.get("foot_geom_pattern",
                              r"^(left|right)_foot([0-9]+|_collision)?_collision$"))
    foot_z = _per_frame_min_foot_z(ctx.model, m, pattern)
    if foot_z is None:
      ctx.warn("warn", f"foot_grounding: no geom matched pattern {pattern!r}")
      return m
    target = float(params.get("target_z", 0.0))
    mode = params.get("mode", "min_over_window")
    out = m.copy()
    if mode == "first_frame":
      offset = target - foot_z[0]
      out.base_pos_w[:, 2] = out.base_pos_w[:, 2] + offset
      ctx.warn("info", f"foot_grounding(first_frame): shifted z by {offset:+.4f}")
    elif mode == "min_over_window":
      W = max(1, int(params.get("window", 5)))
      W = min(W, foot_z.size)
      offset = target - foot_z[:W].min()
      out.base_pos_w[:, 2] = out.base_pos_w[:, 2] + offset
      ctx.warn("info", f"foot_grounding(min_over_window={W}): shifted z by {offset:+.4f}")
    elif mode == "per_frame_clamp":
      shift = np.maximum(target - foot_z, 0.0)
      out.base_pos_w[:, 2] = out.base_pos_w[:, 2] + shift
      n_lifted = int((shift > 1e-6).sum())
      ctx.warn("info", f"foot_grounding(per_frame_clamp): lifted {n_lifted} frames; "
                       f"max shift {shift.max():+.4f}")
    else:
      ctx.warn("warn", f"foot_grounding: unknown mode {mode!r}")
      return m
    return out
