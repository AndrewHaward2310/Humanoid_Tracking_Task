"""Prepend / append "safe" frames that ease in & out from a stable rest pose.

Blending uses a quintic smoothstep (C² continuous → bounded jerk at the seam)
for joints + base position; spherical-linear interpolation (slerp) for
base_quat_w with the same easing curve. Padded frames have joint_vel = 0.
"""
from __future__ import annotations

import numpy as np

from .._kinematics import normalize_quaternions
from ..bundle import MotionBundle
from ..registry import Operator, register


# Joint indices for HOME_KEYFRAME (mirroring static/M2v6/m26_constants.py:117).
# These zeros assume 27-DOF M26; for other robots, fall back to first/last frame.
def _default_rest_joint_pos(joint_names: list[str]) -> np.ndarray:
  out = np.zeros(len(joint_names), dtype=np.float64)
  return out


def _quintic_smoothstep(u: np.ndarray) -> np.ndarray:
  u = np.clip(u, 0.0, 1.0)
  return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def _cosine_step(u: np.ndarray) -> np.ndarray:
  u = np.clip(u, 0.0, 1.0)
  return 0.5 - 0.5 * np.cos(np.pi * u)


def _linear_step(u: np.ndarray) -> np.ndarray:
  return np.clip(u, 0.0, 1.0)


def _slerp(q0: np.ndarray, q1: np.ndarray, t: np.ndarray) -> np.ndarray:
  """Spherical linear interpolation. q0/q1 are length-4 wxyz; t is (N,)."""
  q0 = q0 / max(np.linalg.norm(q0), 1e-12)
  q1 = q1 / max(np.linalg.norm(q1), 1e-12)
  if np.dot(q0, q1) < 0.0:
    q1 = -q1
  dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
  if dot > 0.9995:
    out = (1 - t)[:, None] * q0[None, :] + t[:, None] * q1[None, :]
    return normalize_quaternions(out)
  theta_0 = np.arccos(dot)
  sin_theta_0 = np.sin(theta_0)
  s0 = np.sin((1.0 - t) * theta_0) / sin_theta_0
  s1 = np.sin(t * theta_0) / sin_theta_0
  return s0[:, None] * q0[None, :] + s1[:, None] * q1[None, :]


CURVES = {"quintic": _quintic_smoothstep, "cosine": _cosine_step, "linear": _linear_step}


@register
class PadSafePose(Operator):
  name = "pad_safe_pose"
  description = (
    "Prepend N_pre frames easing IN from a rest pose, append N_post frames "
    "easing OUT to a rest pose. Blend curve defaults to quintic smoothstep "
    "(C² continuous, bounded jerk). All bundle arrays grow by N_pre + N_post."
  )
  schema = {
    "type": "object",
    "properties": {
      "pre_frames":  {"type": "integer", "default": 15, "minimum": 0, "maximum": 200,
                       "description": "Frames inserted at the start (0 disables pre-pad)."},
      "post_frames": {"type": "integer", "default": 15, "minimum": 0, "maximum": 200,
                       "description": "Frames inserted at the end (0 disables post-pad)."},
      "rest_pose":   {"type": "string", "default": "first_frame",
                       "enum": ["first_frame", "zero", "last_frame"],
                       "description": "Source of the rest joint configuration."},
      "rest_base_z": {"type": "number", "default": 1.1,
                       "description": "Rest pose base height (M26 HOME_KEYFRAME ≈ 1.1)."},
      "curve":       {"type": "string", "default": "quintic",
                       "enum": ["quintic", "cosine", "linear"]},
    },
  }
  requires = {"joint_pos", "base_pos_w", "base_quat_w"}
  produces = {"joint_pos", "base_pos_w", "base_quat_w", "joint_vel"}
  default_enabled = False  # changes num_frames; opt-in

  def apply(self, m: MotionBundle, params: dict, ctx) -> MotionBundle:
    pre = int(params.get("pre_frames", 15))
    post = int(params.get("post_frames", 15))
    if pre <= 0 and post <= 0:
      return m
    curve_name = params.get("curve", "quintic")
    step = CURVES.get(curve_name, _quintic_smoothstep)
    rest_kind = params.get("rest_pose", "first_frame")
    rest_base_z = float(params.get("rest_base_z", 1.1))

    # Choose rest joint vector
    if rest_kind == "zero":
      rest_jp = _default_rest_joint_pos(m.joint_names)
    elif rest_kind == "last_frame":
      rest_jp = m.joint_pos[-1].copy()
    else:  # "first_frame"
      rest_jp = m.joint_pos[0].copy()

    # Rest base position: keep xy of nearest endpoint, override z
    first_bp = m.base_pos_w[0].copy()
    last_bp = m.base_pos_w[-1].copy()
    rest_bp_pre = np.array([first_bp[0], first_bp[1], rest_base_z], dtype=np.float64)
    rest_bp_post = np.array([last_bp[0], last_bp[1], rest_base_z], dtype=np.float64)

    # Rest base quat: identity (upright)
    rest_bq = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    out = m.copy()

    # ── Build pre-padding ────────────────────────────────────────────────
    if pre > 0:
      u = np.linspace(0.0, 1.0, pre + 2)[1:-1]   # avoid duplicating endpoints
      s = step(u)
      pre_jp = (1.0 - s)[:, None] * rest_jp[None, :] + s[:, None] * m.joint_pos[0][None, :]
      pre_bp = (1.0 - s)[:, None] * rest_bp_pre[None, :] + s[:, None] * first_bp[None, :]
      pre_bq = _slerp(rest_bq, m.base_quat_w[0], s)
      out.joint_pos = np.concatenate([pre_jp, out.joint_pos], axis=0)
      out.base_pos_w = np.concatenate([pre_bp, out.base_pos_w], axis=0)
      out.base_quat_w = np.concatenate([pre_bq, out.base_quat_w], axis=0)

    # ── Build post-padding ───────────────────────────────────────────────
    if post > 0:
      u = np.linspace(0.0, 1.0, post + 2)[1:-1]
      s = step(u)
      last_jp = m.joint_pos[-1]
      last_bq = m.base_quat_w[-1]
      post_jp = (1.0 - s)[:, None] * last_jp[None, :] + s[:, None] * rest_jp[None, :]
      post_bp = (1.0 - s)[:, None] * last_bp[None, :] + s[:, None] * rest_bp_post[None, :]
      post_bq = _slerp(last_bq, rest_bq, s)
      out.joint_pos = np.concatenate([out.joint_pos, post_jp], axis=0)
      out.base_pos_w = np.concatenate([out.base_pos_w, post_bp], axis=0)
      out.base_quat_w = np.concatenate([out.base_quat_w, post_bq], axis=0)

    out.base_quat_w = normalize_quaternions(out.base_quat_w)

    # joint_vel set to 0 in padded regions; middle gets recomputed by rederive.
    if out.joint_vel.size:
      jv = np.zeros_like(out.joint_pos)
      jv[pre:pre + m.joint_pos.shape[0]] = m.joint_vel if m.joint_vel.size else 0.0
      out.joint_vel = jv
    else:
      out.joint_vel = np.zeros_like(out.joint_pos)

    # body arrays will be grown / refreshed by auto-rederive — but we must
    # at least extend their first dim to match the new T to avoid downstream
    # operators tripping on size mismatch before rederive runs.
    new_T = out.joint_pos.shape[0]
    for fld in ("body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"):
      arr = getattr(out, fld)
      if arr.size and arr.ndim == 3 and arr.shape[0] != new_T:
        # grow with edge-padding so something exists; rederive will overwrite.
        old = arr
        new = np.zeros((new_T,) + old.shape[1:], dtype=old.dtype)
        new[:pre] = old[0]
        new[pre:pre + old.shape[0]] = old
        new[pre + old.shape[0]:] = old[-1]
        if "quat" in fld:
          new[..., 0] = np.where(np.linalg.norm(new, axis=-1) < 1e-9, 1.0, new[..., 0])
        setattr(out, fld, new)

    ctx.warn("info", f"pad_safe_pose: pre={pre} post={post} curve={curve_name} "
                       f"new_T={new_T}")
    return out
