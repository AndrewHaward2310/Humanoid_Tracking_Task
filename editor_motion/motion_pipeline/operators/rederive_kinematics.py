"""Recompute body_* arrays via mj_forward and joint_vel via finite-difference.

This is the user's "calibrate" step: after any edit to joint_pos / base_*,
re-derive everything else so the bundle is FK-consistent again.
"""
from __future__ import annotations

import numpy as np

from .._kinematics import run_mj_forward_per_frame
from ..bundle import MotionBundle
from ..registry import Operator, register


@register
class RederiveKinematics(Operator):
  name = "rederive_kinematics"
  description = (
    "Run mj_forward per-frame to recompute body_pos_w/body_quat_w/body_*_vel_w "
    "from base_pos_w + base_quat_w + joint_pos. Also recomputes joint_vel via "
    "central finite-difference. Always runs at the end of the pipeline (auto-injected)."
  )
  schema = {
    "type": "object",
    "properties": {
      "joint_vel_method": {
        "type": "string", "default": "central_diff",
        "enum": ["central_diff", "zero"],
        "description": "How to compute joint_vel from joint_pos.",
      },
      "recompute_body_vel": {
        "type": "boolean", "default": True,
        "description": "If true, body_lin_vel_w/body_ang_vel_w come from MuJoCo cvel.",
      },
      "force_unit_quat": {
        "type": "boolean", "default": True,
        "description": "Renormalize base_quat_w to unit length before FK.",
      },
    },
  }
  requires = {"joint_pos", "base_pos_w", "base_quat_w"}
  produces = {"joint_vel", "body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"}
  always_run = True

  def apply(self, m: MotionBundle, params: dict, ctx) -> MotionBundle:
    if ctx.model is None:
      ctx.warn("warn", "rederive_kinematics: no MuJoCo model loaded; skipping FK")
      return _rederive_joint_vel_only(m, params)

    out = m.copy()
    if params.get("force_unit_quat", True) and out.base_quat_w.size:
      norms = np.linalg.norm(out.base_quat_w, axis=1, keepdims=True)
      norms[norms == 0] = 1.0
      out.base_quat_w = out.base_quat_w / norms

    body_names = out.body_names if out.body_names else None
    pos, quat, lvel, avel = run_mj_forward_per_frame(
      ctx.model,
      out.base_pos_w,
      out.base_quat_w,
      out.joint_pos,
      out.joint_names,
      body_names,
    )
    out.body_pos_w = pos
    out.body_quat_w = quat
    if params.get("recompute_body_vel", True):
      out.body_lin_vel_w = lvel
      out.body_ang_vel_w = avel
    if not out.body_names:
      from .._kinematics import get_mujoco_body_order
      out.body_names = get_mujoco_body_order(ctx.model)

    out.joint_vel = _compute_joint_vel(out.joint_pos, m.dt, params.get("joint_vel_method", "central_diff"))
    return out


def _compute_joint_vel(joint_pos: np.ndarray, dt: float, method: str) -> np.ndarray:
  if method == "zero":
    return np.zeros_like(joint_pos)
  return np.gradient(joint_pos, dt, axis=0)


def _rederive_joint_vel_only(m: MotionBundle, params: dict) -> MotionBundle:
  out = m.copy()
  out.joint_vel = _compute_joint_vel(out.joint_pos, m.dt, params.get("joint_vel_method", "central_diff"))
  return out
