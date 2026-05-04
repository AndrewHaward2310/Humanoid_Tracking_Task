"""Hard clamp joint_pos to MJCF-defined ranges (with soft factor)."""
from __future__ import annotations

import numpy as np

from ..bundle import MotionBundle
from ..registry import Operator, register


@register
class ClampJointLimits(Operator):
  name = "clamp_joint_limits"
  description = (
    "Clamp each joint position to [lo + (1-f)/2·range, hi - (1-f)/2·range] using "
    "the soft_factor f from MJCF. Any joint not present in the model is skipped."
  )
  schema = {
    "type": "object",
    "properties": {
      "soft_factor": {
        "type": "number", "default": 0.9,
        "description": "Range multiplier; 1.0 = full hard limit, 0.9 ≈ 5% margin each side.",
        "minimum": 0.5, "maximum": 1.0,
      },
      "report_only": {
        "type": "boolean", "default": False,
        "description": "If true, count violations but don't modify joint_pos.",
      },
    },
  }
  requires = {"joint_pos"}
  produces = {"joint_pos"}

  def apply(self, m: MotionBundle, params: dict, ctx) -> MotionBundle:
    if not ctx.limits or "joints" not in ctx.limits:
      ctx.warn("warn", "clamp_joint_limits: no MJCF limits available; skipping")
      return m
    soft = float(params.get("soft_factor", 0.9))
    report_only = bool(params.get("report_only", False))
    out = m.copy()
    jp = out.joint_pos
    name_to_col = {n: i for i, n in enumerate(out.joint_names)}
    violations: dict[str, dict] = {}
    for jname, lim in ctx.limits["joints"].items():
      if jname not in name_to_col:
        continue
      if not lim.get("limited", True):
        continue
      lo = lim["pos_min"]; hi = lim["pos_max"]
      half = 0.5 * (1.0 - soft) * (hi - lo)
      lo_s, hi_s = lo + half, hi - half
      col = name_to_col[jname]
      vec = jp[:, col]
      n_lo = int(np.sum(vec < lo_s))
      n_hi = int(np.sum(vec > hi_s))
      if n_lo or n_hi:
        violations[jname] = {
          "below": n_lo, "above": n_hi,
          "max_below": float(max(0.0, lo_s - vec.min())),
          "max_above": float(max(0.0, vec.max() - hi_s)),
        }
        if not report_only:
          jp[:, col] = np.clip(vec, lo_s, hi_s)
    if violations:
      ctx.warn(
        "warn" if report_only else "info",
        f"clamp_joint_limits: {len(violations)} joints clipped",
        details=violations,
      )
    if report_only:
      return m
    return out
