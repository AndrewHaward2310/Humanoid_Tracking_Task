"""MotionBundle — backend mirror of frontend `motionData`.

Round-trips losslessly with /upload_motion and /save_motion JSON shapes.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np


def _as_np(x: Any, dtype=np.float64) -> np.ndarray:
  return np.asarray(x, dtype=dtype) if x is not None and len(x) else np.empty((0,), dtype=dtype)


def _as_float(x: Any, fallback: float = 30.0) -> float:
  if x is None:
    return float(fallback)
  if isinstance(x, (list, tuple)):
    return float(x[0]) if len(x) else float(fallback)
  arr = np.asarray(x).flatten()
  return float(arr[0]) if arr.size else float(fallback)


@dataclass
class MotionBundle:
  joint_pos: np.ndarray            # (T, n_j) float64
  base_pos_w: np.ndarray           # (T, 3)   float64
  base_quat_w: np.ndarray          # (T, 4)   wxyz float64
  joint_names: list[str] = field(default_factory=list)
  joint_vel: np.ndarray = field(default_factory=lambda: np.empty((0,)))     # (T, n_j) or empty
  body_names: list[str] = field(default_factory=list)
  body_pos_w: np.ndarray = field(default_factory=lambda: np.empty((0,)))    # (T, n_b, 3) or empty
  body_quat_w: np.ndarray = field(default_factory=lambda: np.empty((0,)))   # (T, n_b, 4)
  body_lin_vel_w: np.ndarray = field(default_factory=lambda: np.empty((0,))) # (T, n_b, 3)
  body_ang_vel_w: np.ndarray = field(default_factory=lambda: np.empty((0,))) # (T, n_b, 3)
  fps: float = 30.0
  framerate: float = 30.0

  # ── Construction helpers ──────────────────────────────────────────────────
  @classmethod
  def from_json(cls, d: dict) -> "MotionBundle":
    jp = _as_np(d.get("joint_pos", []))
    if jp.ndim != 2:
      raise ValueError(f"joint_pos must be 2-D, got shape {jp.shape}")
    T, nj = jp.shape

    bp = _as_np(d.get("base_pos_w", []))
    if bp.size == 0:
      bp = np.zeros((T, 3))
    bq = _as_np(d.get("base_quat_w", []))
    if bq.size == 0:
      bq = np.tile([1.0, 0.0, 0.0, 0.0], (T, 1))

    jv = _as_np(d.get("joint_vel", []))
    if jv.size == 0:
      jv = np.zeros((T, nj))

    def _maybe_3d(name: str) -> np.ndarray:
      raw = d.get(name)
      if raw is None or len(raw) == 0:
        return np.empty((0,))
      arr = np.asarray(raw, dtype=np.float64)
      return arr

    return cls(
      joint_pos=jp,
      base_pos_w=bp,
      base_quat_w=bq,
      joint_names=list(d.get("joint_names", [])),
      joint_vel=jv,
      body_names=list(d.get("body_names", [])),
      body_pos_w=_maybe_3d("body_pos_w"),
      body_quat_w=_maybe_3d("body_quat_w"),
      body_lin_vel_w=_maybe_3d("body_lin_vel_w"),
      body_ang_vel_w=_maybe_3d("body_ang_vel_w"),
      fps=_as_float(d.get("fps", 30.0)),
      framerate=_as_float(d.get("framerate", d.get("fps", 30.0))),
    )

  # ── Serialization ─────────────────────────────────────────────────────────
  def to_json(self) -> dict:
    def _arr(x: np.ndarray) -> list:
      return x.tolist() if x.size else []
    return {
      "joint_pos": self.joint_pos.tolist(),
      "joint_vel": _arr(self.joint_vel),
      "joint_names": list(self.joint_names),
      "body_names": list(self.body_names),
      "base_pos_w": self.base_pos_w.tolist(),
      "base_quat_w": self.base_quat_w.tolist(),
      "body_pos_w": _arr(self.body_pos_w),
      "body_quat_w": _arr(self.body_quat_w),
      "body_lin_vel_w": _arr(self.body_lin_vel_w),
      "body_ang_vel_w": _arr(self.body_ang_vel_w),
      "fps": self.fps,
      "framerate": self.framerate,
      "num_frames": int(self.joint_pos.shape[0]),
    }

  # ── Convenience ───────────────────────────────────────────────────────────
  @property
  def num_frames(self) -> int:
    return int(self.joint_pos.shape[0])

  @property
  def n_joints(self) -> int:
    return int(self.joint_pos.shape[1])

  @property
  def dt(self) -> float:
    return 1.0 / float(self.fps) if self.fps > 0 else 1.0 / 30.0

  def has_bodies(self) -> bool:
    return self.body_pos_w.size > 0 and len(self.body_names) > 0

  def copy(self) -> "MotionBundle":
    return replace(
      self,
      joint_pos=self.joint_pos.copy(),
      base_pos_w=self.base_pos_w.copy(),
      base_quat_w=self.base_quat_w.copy(),
      joint_vel=self.joint_vel.copy() if self.joint_vel.size else self.joint_vel,
      body_pos_w=self.body_pos_w.copy() if self.body_pos_w.size else self.body_pos_w,
      body_quat_w=self.body_quat_w.copy() if self.body_quat_w.size else self.body_quat_w,
      body_lin_vel_w=self.body_lin_vel_w.copy() if self.body_lin_vel_w.size else self.body_lin_vel_w,
      body_ang_vel_w=self.body_ang_vel_w.copy() if self.body_ang_vel_w.size else self.body_ang_vel_w,
      joint_names=list(self.joint_names),
      body_names=list(self.body_names),
    )

  # Field set used by operator `produces` declarations.
  MUTABLE_FIELDS = {
    "joint_pos", "joint_vel", "base_pos_w", "base_quat_w",
    "body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w",
  }
  # Fields whose change triggers auto-rederive of body_* and joint_vel.
  PRIMARY_FIELDS = {"joint_pos", "base_pos_w", "base_quat_w"}
