"""Enforce per-joint velocity / acceleration caps.

Two-pass algorithm:
  1. Detect joints whose 99-th percentile |vel| or |acc| exceeds the limit.
  2. For each offending joint, apply a Butterworth low-pass with cutoff chosen
     to bring the offending statistic under the limit.

This is *not* time-rescaling: the motion's duration is preserved. If a joint
is so spiky that even cutoff=fps/4 can't tame it, we fall back to a tighter
cutoff and surface a `time_rescale_recommended` warning so the user knows
to redesign that segment.
"""
from __future__ import annotations

import numpy as np
from scipy.signal import butter, filtfilt

from ..bundle import MotionBundle
from ..registry import Operator, register


def _butter_lowpass(x: np.ndarray, cutoff_hz: float, fps: float, order: int = 4) -> np.ndarray:
  nyq = fps / 2.0
  Wn = max(min(cutoff_hz / nyq, 0.99), 0.01)
  T = x.shape[0]
  if T < 3 * order * 2 + 1:
    return x.copy()
  b, a = butter(order, Wn, btype="low")
  return filtfilt(b, a, x)


def _bisect_cutoff(
  x: np.ndarray, fps: float, vmax: float, amax: float,
  hi_cutoff: float, lo_cutoff: float, order: int, max_iter: int = 8,
) -> tuple[np.ndarray, float, bool]:
  """Bisect Butterworth cutoff to bring vel_p99 under vmax (and acc_max under amax)."""
  dt = 1.0 / fps
  best = x.copy()
  best_cutoff = hi_cutoff
  ok = False
  lo, hi = lo_cutoff, hi_cutoff
  for _ in range(max_iter):
    mid = 0.5 * (lo + hi)
    y = _butter_lowpass(x, mid, fps, order)
    v_p99 = float(np.percentile(np.abs(np.gradient(y, dt)), 99))
    a_max = float(np.abs(np.gradient(np.gradient(y, dt), dt)).max())
    if v_p99 <= vmax and a_max <= amax:
      ok = True
      best = y; best_cutoff = mid
      lo = mid  # try a higher cutoff (preserve more detail)
    else:
      hi = mid  # need lower cutoff
  if not ok:
    # final pass at the most aggressive cutoff
    y = _butter_lowpass(x, lo_cutoff, fps, order)
    return y, lo_cutoff, False
  return best, best_cutoff, True


@register
class EnforceKinematicLimits(Operator):
  name = "enforce_kinematic_limits"
  description = (
    "Bring per-joint velocity / acceleration under MJCF or YAML-defined limits "
    "by applying a per-joint Butterworth low-pass with auto-tuned cutoff. "
    "Surfaces a `time_rescale_recommended` warning when even an aggressive "
    "cutoff can't satisfy both limits."
  )
  schema = {
    "type": "object",
    "properties": {
      "vel_scale":   {"type": "number", "default": 1.0, "minimum": 0.1, "maximum": 1.5,
                       "description": "Multiplier on configured vel_max (≤1 → stricter)."},
      "acc_scale":   {"type": "number", "default": 1.0, "minimum": 0.1, "maximum": 1.5,
                       "description": "Multiplier on configured acc_max (≤1 → stricter)."},
      "butter_order": {"type": "integer", "default": 4, "minimum": 1, "maximum": 8},
      "max_cutoff_hz": {"type": "number", "default": 12.0, "minimum": 1.0, "maximum": 30.0},
      "min_cutoff_hz": {"type": "number", "default": 1.0,  "minimum": 0.1, "maximum": 10.0},
    },
  }
  requires = {"joint_pos"}
  produces = {"joint_pos"}

  def apply(self, m: MotionBundle, params: dict, ctx) -> MotionBundle:
    if not ctx.limits or "joints" not in ctx.limits:
      ctx.warn("warn", "enforce_kinematic_limits: no limits available; skipping")
      return m
    vel_scale = float(params.get("vel_scale", 1.0))
    acc_scale = float(params.get("acc_scale", 1.0))
    order = int(params.get("butter_order", 4))
    hi = float(params.get("max_cutoff_hz", 12.0))
    lo = float(params.get("min_cutoff_hz", 1.0))

    out = m.copy()
    fps = float(out.fps)
    dt = 1.0 / fps
    name_to_col = {n: i for i, n in enumerate(out.joint_names)}
    affected: dict[str, dict] = {}
    rescale_recs: list[str] = []

    for jname, lim in ctx.limits["joints"].items():
      if jname not in name_to_col:
        continue
      col = name_to_col[jname]
      vmax = float(lim.get("vel_max", float("inf"))) * vel_scale
      amax = float(lim.get("acc_max", float("inf"))) * acc_scale
      x = out.joint_pos[:, col]
      v = np.gradient(x, dt)
      a = np.gradient(v, dt)
      v_p99 = float(np.percentile(np.abs(v), 99))
      a_max = float(np.abs(a).max())
      if v_p99 <= vmax and a_max <= amax:
        continue
      y, cutoff, ok = _bisect_cutoff(x, fps, vmax, amax, hi, lo, order)
      out.joint_pos[:, col] = y
      vy = np.gradient(y, dt)
      ay = np.gradient(vy, dt)
      affected[jname] = {
        "before": {"vel_p99": v_p99, "acc_max": a_max},
        "after":  {"vel_p99": float(np.percentile(np.abs(vy), 99)), "acc_max": float(np.abs(ay).max())},
        "cutoff_hz": float(cutoff),
        "satisfied": bool(ok),
      }
      if not ok:
        rescale_recs.append(jname)
    if affected:
      ctx.warn("info", f"enforce_kinematic_limits: filtered {len(affected)} joints",
               details=affected)
    if rescale_recs:
      ctx.warn("warn", f"time_rescale_recommended for {len(rescale_recs)} joints — "
                       "spikes too sharp for low-pass alone",
               joints=rescale_recs)
    return out
