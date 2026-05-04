"""Origin & yaw alignment — frame-N base xy → 0, yaw → 0."""
from __future__ import annotations

import numpy as np

from .._kinematics import normalize_quaternions
from ..bundle import MotionBundle
from ..registry import Operator, register


def _yaw_from_quat_wxyz(q: np.ndarray) -> float:
  w, x, y, z = q
  return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _quat_axis_angle_z(angle: float) -> np.ndarray:
  half = 0.5 * angle
  return np.array([np.cos(half), 0.0, 0.0, np.sin(half)], dtype=np.float64)


def _quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
  aw, ax, ay, az = a
  bw, bx, by, bz = b
  return np.array([
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ], dtype=np.float64)


@register
class AlignOrigin(Operator):
  name = "align_origin"
  description = (
    "Translate base_pos_w[:,:2] so the reference frame sits at (0,0); rotate the "
    "whole trajectory about world Z so the reference frame's yaw is 0. "
    "Applies the same rotation/translation to base_quat_w as well."
  )
  schema = {
    "type": "object",
    "properties": {
      "align_xy":         {"type": "boolean", "default": True,
                            "description": "Subtract reference frame's xy from base_pos_w."},
      "align_yaw":        {"type": "boolean", "default": True,
                            "description": "Rotate trajectory so reference frame's yaw = 0."},
      "align_z":          {"type": "boolean", "default": False,
                            "description": "Also subtract reference frame's z (rare; usually let foot_grounding handle Z)."},
      "reference_frame":  {"type": "integer", "default": 0, "minimum": 0,
                            "description": "Which frame to use as the reference."},
    },
  }
  requires = {"base_pos_w", "base_quat_w"}
  produces = {"base_pos_w", "base_quat_w"}

  def apply(self, m: MotionBundle, params: dict, ctx) -> MotionBundle:
    if m.base_pos_w.shape[0] == 0:
      return m
    align_xy = bool(params.get("align_xy", True))
    align_yaw = bool(params.get("align_yaw", True))
    align_z = bool(params.get("align_z", False))
    ref = int(params.get("reference_frame", 0))
    ref = max(0, min(ref, m.num_frames - 1))

    out = m.copy()
    bp = out.base_pos_w.copy()
    bq = out.base_quat_w.copy()
    bq = normalize_quaternions(bq)

    if align_yaw:
      yaw0 = _yaw_from_quat_wxyz(bq[ref])
      q_corr = _quat_axis_angle_z(-yaw0)
      cos_y, sin_y = np.cos(-yaw0), np.sin(-yaw0)
      Rz = np.array([[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]])
      pivot = bp[ref].copy()
      bp = (bp - pivot) @ Rz.T + pivot
      for t in range(bq.shape[0]):
        bq[t] = _quat_mul_wxyz(q_corr, bq[t])
      bq = normalize_quaternions(bq)
      ctx.warn("info", f"align_origin: yaw rotated by {-np.degrees(yaw0):.2f}°", reference_frame=ref)

    offset = np.zeros(3)
    if align_xy:
      offset[0] = bp[ref, 0]
      offset[1] = bp[ref, 1]
    if align_z:
      offset[2] = bp[ref, 2]
    if align_xy or align_z:
      bp = bp - offset
      ctx.warn("info", f"align_origin: translated by {(-offset).tolist()}")

    out.base_pos_w = bp
    out.base_quat_w = bq
    return out
