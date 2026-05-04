"""Manifold-aware smoothing for joints, base position, and base quaternion.

Default backend: Savitzky-Golay (preserves peaks better than moving average,
gives analytic derivatives). Optional Butterworth low-pass with zero-phase
filtfilt; Gaussian fallback when scipy is unavailable.
"""
from __future__ import annotations

import numpy as np
from scipy.signal import butter, filtfilt, savgol_filter
from scipy.spatial.transform import Rotation as R

from .._kinematics import (
  gaussian_filter_1d,
  make_quaternion_continuous,
  normalize_quaternions,
)
from ..bundle import MotionBundle
from ..registry import Operator, register


def _safe_savgol(x: np.ndarray, window: int, poly: int) -> np.ndarray:
  T = x.shape[0]
  win = min(window, T if T % 2 == 1 else T - 1)
  if win < poly + 2:
    return x.copy()
  return savgol_filter(x, window_length=win, polyorder=poly, mode="nearest", axis=0)


def _safe_butter(x: np.ndarray, cutoff_hz: float, fps: float, order: int) -> np.ndarray:
  T = x.shape[0]
  nyq = fps / 2.0
  Wn = max(min(cutoff_hz / nyq, 0.99), 0.01)
  # filtfilt needs at least 3*(max(len(a),len(b))-1) samples; SOS form reduces this.
  if T < 3 * order * 2 + 1:
    return x.copy()
  b, a = butter(order, Wn, btype="low")
  return filtfilt(b, a, x, axis=0)


def _smooth_quat_log(q: np.ndarray, fn) -> np.ndarray:
  """Smooth a (T,4) quaternion sequence on the SO(3) tangent space.

  Steps: continuity → log map (rotvec) → smooth components → exp map → renorm.
  fn(component_array) -> smoothed component array.
  """
  q_cont = make_quaternion_continuous(q)
  # scipy expects xyzw order; our q is wxyz.
  q_xyzw = q_cont[:, [1, 2, 3, 0]]
  rotvec = R.from_quat(q_xyzw).as_rotvec()  # (T, 3)
  smoothed = np.column_stack([fn(rotvec[:, i]) for i in range(3)])
  q_back_xyzw = R.from_rotvec(smoothed).as_quat()
  q_wxyz = q_back_xyzw[:, [3, 0, 1, 2]]
  return normalize_quaternions(q_wxyz)


@register
class Smooth(Operator):
  name = "smooth"
  description = (
    "Smooth joint_pos, base_pos_w, and (manifold-aware) base_quat_w. Default "
    "method is Savitzky-Golay (window=11, poly=3); butterworth and gaussian "
    "are also available."
  )
  schema = {
    "type": "object",
    "properties": {
      "method": {
        "type": "string", "default": "savgol",
        "enum": ["savgol", "butter", "gauss"],
        "description": "savgol = Savitzky-Golay; butter = Butterworth + filtfilt; gauss = Gaussian convolution.",
      },
      "window": {"type": "integer", "default": 11, "minimum": 3, "maximum": 51},
      "polyorder": {"type": "integer", "default": 3, "minimum": 1, "maximum": 7},
      "cutoff_hz": {"type": "number", "default": 6.0, "minimum": 0.5, "maximum": 30.0},
      "butter_order": {"type": "integer", "default": 4, "minimum": 1, "maximum": 8},
      "gauss_sigma": {"type": "number", "default": 2.0, "minimum": 0.1, "maximum": 10.0},
      "targets": {
        "type": "array", "default": ["joints", "base_pos", "base_quat"],
        "items": {"type": "string", "enum": ["joints", "base_pos", "base_quat"]},
      },
    },
  }
  requires = {"joint_pos"}
  produces = {"joint_pos", "base_pos_w", "base_quat_w"}

  def apply(self, m: MotionBundle, params: dict, ctx) -> MotionBundle:
    method = params.get("method", "savgol")
    window = int(params.get("window", 11))
    poly = int(params.get("polyorder", 3))
    cutoff = float(params.get("cutoff_hz", 6.0))
    border = int(params.get("butter_order", 4))
    sigma = float(params.get("gauss_sigma", 2.0))
    targets = set(params.get("targets", ["joints", "base_pos", "base_quat"]))

    def smooth_axis(x: np.ndarray) -> np.ndarray:
      if method == "savgol":
        return _safe_savgol(x, window, poly)
      if method == "butter":
        return _safe_butter(x, cutoff, m.fps, border)
      if method == "gauss":
        if x.ndim == 1:
          return gaussian_filter_1d(x, sigma=sigma, size=window)
        return np.column_stack([gaussian_filter_1d(x[:, i], sigma=sigma, size=window) for i in range(x.shape[1])])
      raise ValueError(f"smooth: unknown method {method!r}")

    out = m.copy()
    if "joints" in targets and out.joint_pos.size:
      out.joint_pos = smooth_axis(out.joint_pos)
    if "base_pos" in targets and out.base_pos_w.size:
      out.base_pos_w = smooth_axis(out.base_pos_w)
    if "base_quat" in targets and out.base_quat_w.size:
      def fn(arr):  # arr is 1-D
        if method == "savgol":
          return _safe_savgol(arr, window, poly)
        if method == "butter":
          return _safe_butter(arr, cutoff, m.fps, border)
        if method == "gauss":
          return gaussian_filter_1d(arr, sigma=sigma, size=window)
        raise ValueError(method)
      out.base_quat_w = _smooth_quat_log(out.base_quat_w, fn)
    ctx.warn("info", f"smooth: method={method} targets={sorted(targets)}")
    return out
