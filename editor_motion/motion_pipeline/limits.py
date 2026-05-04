"""Joint limit extraction from MuJoCo MJCF files.

Reads pos limits from `model.jnt_range` and velocity/effort limits from
optional sources:
  1. Per-joint XML attributes (model.actuator_forcerange when applicable).
  2. Override file `motion_pipeline/limits_m26.yaml` keyed by joint name regex.
"""
from __future__ import annotations

import os
import re
from typing import Any

import yaml

# ── Model cache (shared with app.py) ─────────────────────────────────────────
_MODEL_CACHE: dict[str, Any] = {}


def get_model(xml_path: str):
  if not os.path.isabs(xml_path):
    xml_path = os.path.abspath(xml_path)
  if xml_path in _MODEL_CACHE:
    return _MODEL_CACHE[xml_path]
  import mujoco
  m = mujoco.MjModel.from_xml_path(xml_path)
  _MODEL_CACHE[xml_path] = m
  return m


# ── YAML overrides ──────────────────────────────────────────────────────────
_OVERRIDES_CACHE: dict[str, dict] = {}


def _load_overrides(path: str) -> dict:
  if path in _OVERRIDES_CACHE:
    return _OVERRIDES_CACHE[path]
  if not os.path.isfile(path):
    _OVERRIDES_CACHE[path] = {}
    return {}
  with open(path) as f:
    data = yaml.safe_load(f) or {}
  _OVERRIDES_CACHE[path] = data
  return data


def _apply_override(joint_name: str, overrides: dict) -> dict:
  """Walk regex keys and merge into a single dict for this joint."""
  out: dict = {}
  for key, val in overrides.get("joint_overrides", {}).items():
    if re.search(key, joint_name):
      out.update(val)
  return out


# ── Public API ──────────────────────────────────────────────────────────────

def extract_limits(model: Any, overrides_path: str | None = None) -> dict:
  """Return a dict keyed by joint name with pos_min/pos_max/vel_max/effort_max.

  Falls back to YAML overrides for vmax/amax when MJCF doesn't expose them.
  """
  import mujoco

  if overrides_path is None:
    overrides_path = os.path.join(
      os.path.dirname(os.path.abspath(__file__)), "limits_m26.yaml"
    )
  ov = _load_overrides(overrides_path)
  defaults = ov.get("defaults", {}) or {}
  default_vmax = float(defaults.get("vel_max", 12.0))
  default_amax = float(defaults.get("acc_max", 50.0))
  default_effort = float(defaults.get("effort_max", 100.0))

  joints: dict[str, dict] = {}
  joint_order: list[str] = []
  for j in range(model.njnt):
    jtype = model.jnt_type[j]
    if jtype == mujoco.mjtJoint.mjJNT_FREE:
      continue
    name = model.joint(j).name
    rng = model.jnt_range[j]
    limited = bool(model.jnt_limited[j])
    pos_min = float(rng[0]) if limited else -float("inf")
    pos_max = float(rng[1]) if limited else float("inf")

    j_over = _apply_override(name, ov)
    vmax = float(j_over.get("vel_max", default_vmax))
    amax = float(j_over.get("acc_max", default_amax))
    effort_max = float(j_over.get("effort_max", default_effort))

    # Try to read effort from the actuator's force range (if any).
    for a in range(model.nu):
      if model.actuator_trnid[a, 0] == j:
        fr = model.actuator_forcerange[a]
        if model.actuator_forcelimited[a]:
          effort_max = float(max(abs(fr[0]), abs(fr[1])))
        break

    joints[name] = {
      "pos_min": pos_min,
      "pos_max": pos_max,
      "vel_max": vmax,
      "acc_max": amax,
      "effort_max": effort_max,
      "limited": limited,
    }
    joint_order.append(name)

  return {
    "joints": joints,
    "joint_order": joint_order,
    "dof": len(joint_order),
    "soft_factor": float(defaults.get("soft_factor", 0.9)),
  }
