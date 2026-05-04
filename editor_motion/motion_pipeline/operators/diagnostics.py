"""Quality report — read-only operator that writes findings to ctx.report."""
from __future__ import annotations

import numpy as np

from ..bundle import MotionBundle
from ..registry import Operator, register


@register
class Diagnostics(Operator):
  name = "diagnostics"
  description = (
    "Compute per-joint vel/acc/jerk stats, joint-limit violation counts, "
    "base height range, and (when MJCF is available) foot-slip metric. "
    "Read-only — never modifies the bundle."
  )
  schema = {
    "type": "object",
    "properties": {
      "vel_p99": {"type": "boolean", "default": True},
      "include_foot_slip": {"type": "boolean", "default": True},
    },
  }
  requires = {"joint_pos"}
  produces: set[str] = set()
  always_run = True

  def apply(self, m: MotionBundle, params: dict, ctx) -> MotionBundle:
    dt = m.dt
    jp = m.joint_pos
    T, nj = jp.shape
    vel = np.gradient(jp, dt, axis=0) if T > 1 else np.zeros_like(jp)
    acc = np.gradient(vel, dt, axis=0) if T > 1 else np.zeros_like(jp)
    jerk = np.gradient(acc, dt, axis=0) if T > 1 else np.zeros_like(jp)

    per_joint: dict[str, dict] = {}
    warnings: list[dict] = []
    limits = (ctx.limits or {}).get("joints", {})
    soft = (ctx.limits or {}).get("soft_factor", 0.9)

    for i, name in enumerate(m.joint_names or [f"joint_{i}" for i in range(nj)]):
      v_abs = np.abs(vel[:, i])
      a_abs = np.abs(acc[:, i])
      j_abs = np.abs(jerk[:, i])
      stats = {
        "pos_min": float(jp[:, i].min()),
        "pos_max": float(jp[:, i].max()),
        "vel_max": float(v_abs.max()) if T > 1 else 0.0,
        "vel_p99": float(np.percentile(v_abs, 99)) if T > 1 else 0.0,
        "acc_max": float(a_abs.max()) if T > 1 else 0.0,
        "jerk_max": float(j_abs.max()) if T > 1 else 0.0,
      }
      lim = limits.get(name)
      if lim and lim.get("limited", True):
        lo = lim["pos_min"]; hi = lim["pos_max"]
        half = 0.5 * (1.0 - soft) * (hi - lo)
        lo_s, hi_s = lo + half, hi - half
        stats["limit_violation_count"] = int(np.sum((jp[:, i] < lo_s) | (jp[:, i] > hi_s)))
        stats["clamp_margin"] = float(min(jp[:, i].min() - lo_s, hi_s - jp[:, i].max()))
        stats["limit_pos_min"] = lo
        stats["limit_pos_max"] = hi
        if stats["vel_max"] > lim.get("vel_max", float("inf")):
          warnings.append({
            "level": "warn", "joint": name, "kind": "vel_exceeds_limit",
            "value": stats["vel_max"], "limit": lim.get("vel_max"),
          })
        if stats["limit_violation_count"]:
          warnings.append({
            "level": "warn", "joint": name, "kind": "pos_limit_violation",
            "count": stats["limit_violation_count"],
          })
      per_joint[name] = stats

    glob: dict = {
      "num_frames": T,
      "fps": m.fps,
      "duration_s": float(T * dt),
    }
    if m.base_pos_w.size:
      glob["base_z_min"] = float(m.base_pos_w[:, 2].min())
      glob["base_z_max"] = float(m.base_pos_w[:, 2].max())

    bad_frames: list[int] = []
    for i, name in enumerate(m.joint_names or []):
      lim = limits.get(name)
      if not lim:
        continue
      vmax = lim.get("vel_max", float("inf"))
      mask = np.abs(vel[:, i]) > vmax
      if mask.any():
        idxs = np.where(mask)[0].tolist()
        bad_frames.extend(idxs)
    glob["vel_violation_frames"] = sorted(set(bad_frames))

    if params.get("include_foot_slip", True) and ctx.model is not None and m.has_bodies():
      slip = _foot_slip_metric(m)
      if slip is not None:
        glob["foot_slip_total_xy"] = float(slip["total"])
        glob["foot_slip_frames"] = slip["frames"]

    ctx.report = {"joints": per_joint, "global": glob, "warnings": warnings}
    return m


def _foot_slip_metric(m: MotionBundle) -> dict | None:
  """Sum of foot xy-velocity magnitudes over frames where the foot is in contact."""
  body_names = m.body_names
  foot_keys = [n for n in body_names if "foot" in n.lower() and "ankle" not in n.lower()]
  if not foot_keys:
    return None
  idx_map = {n: i for i, n in enumerate(body_names)}
  dt = m.dt
  total = 0.0
  frames: list[int] = []
  for fk in foot_keys:
    bi = idx_map[fk]
    pos = m.body_pos_w[:, bi]                      # (T, 3)
    vxy = np.gradient(pos[:, :2], dt, axis=0)
    contact = pos[:, 2] < 0.02                     # 2 cm threshold
    speed = np.linalg.norm(vxy, axis=1)
    contrib = np.where(contact, speed, 0.0) * dt
    total += float(contrib.sum())
    frames.extend(np.where((contact) & (speed > 0.05))[0].tolist())
  return {"total": total, "frames": sorted(set(frames))}
