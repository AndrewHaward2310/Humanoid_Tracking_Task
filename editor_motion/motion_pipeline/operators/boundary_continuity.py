"""Half-cosine taper of the first/last K frames toward a constant boundary value.

After this runs, joint_vel at frame 0 and T-1 is ≈0, which matches what RL
imitation policies expect at the start/end of a tracked clip.
"""
from __future__ import annotations

import numpy as np

from .._kinematics import normalize_quaternions
from ..bundle import MotionBundle
from ..registry import Operator, register


def _half_cos_window(K: int) -> np.ndarray:
  if K <= 0:
    return np.empty((0,), dtype=np.float64)
  i = np.arange(K, dtype=np.float64)
  return 0.5 * (1.0 + np.cos(np.pi * i / K))


@register
class BoundaryContinuity(Operator):
  name = "boundary_continuity"
  description = (
    "Taper the first / last K frames toward the boundary frame so joint_vel "
    "at the seams is ≈0. Helps PD-tracking policies start and stop cleanly."
  )
  schema = {
    "type": "object",
    "properties": {
      "taper_frames": {"type": "integer", "default": 8, "minimum": 1, "maximum": 60},
      "apply_start":  {"type": "boolean", "default": True},
      "apply_end":    {"type": "boolean", "default": True},
      "include_base": {"type": "boolean", "default": True,
                        "description": "Also taper base_pos_w and base_quat_w."},
    },
  }
  requires = {"joint_pos", "base_pos_w", "base_quat_w"}
  produces = {"joint_pos", "base_pos_w", "base_quat_w"}
  default_enabled = False  # mutually exclusive with pad_safe_pose; opt-in

  def apply(self, m: MotionBundle, params: dict, ctx) -> MotionBundle:
    K = int(params.get("taper_frames", 8))
    do_start = bool(params.get("apply_start", True))
    do_end = bool(params.get("apply_end", True))
    do_base = bool(params.get("include_base", True))
    T = m.num_frames
    if K < 1 or T < 2 * K + 1:
      ctx.warn("warn", f"boundary_continuity: K={K} too large for T={T}; skipping")
      return m

    out = m.copy()
    w = _half_cos_window(K)

    def taper_array(arr: np.ndarray, anchor: np.ndarray, indices: np.ndarray, ww: np.ndarray) -> None:
      # arr[indices] = arr[indices] * ww + anchor * (1 - ww)
      arr[indices] = arr[indices] * ww[:, None] + anchor[None, :] * (1.0 - ww[:, None])

    if do_start:
      idx = np.arange(K)
      ww = w[::-1]  # ramp from 0 → 1 (anchor weighted heavily near i=0)
      taper_array(out.joint_pos, out.joint_pos[0].copy(), idx, ww)
      if do_base:
        taper_array(out.base_pos_w, out.base_pos_w[0].copy(), idx, ww)
        taper_array(out.base_quat_w, out.base_quat_w[0].copy(), idx, ww)

    if do_end:
      idx = np.arange(T - K, T)
      ww = w  # ramp from 1 → 0 (anchor weighted heavily near i=T-1)
      taper_array(out.joint_pos, out.joint_pos[-1].copy(), idx, ww)
      if do_base:
        taper_array(out.base_pos_w, out.base_pos_w[-1].copy(), idx, ww)
        taper_array(out.base_quat_w, out.base_quat_w[-1].copy(), idx, ww)

    if do_base:
      out.base_quat_w = normalize_quaternions(out.base_quat_w)
    ctx.warn("info", f"boundary_continuity: tapered K={K} start={do_start} end={do_end}")
    return out
