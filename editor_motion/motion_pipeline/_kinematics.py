"""Kinematics utilities shared between the CLI and the pipeline runner.

Most functions are lifted verbatim from convert_to_mujoco.py so that the CLI
script and Flask backend agree on FK/remap semantics.
"""
from __future__ import annotations

from typing import Any

import numpy as np


# ── Remapping ───────────────────────────────────────────────────────────────

def get_mujoco_joint_order(model: Any) -> list[str]:
  """Return actuated 1-DOF joint names in MuJoCo qpos order (skips freejoint)."""
  import mujoco
  names: list[str] = []
  for j in range(model.njnt):
    jtype = model.jnt_type[j]
    if jtype == mujoco.mjtJoint.mjJNT_FREE:
      continue
    if jtype in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
      names.append(model.joint(j).name)
  return names


def get_mujoco_body_order(model: Any) -> list[str]:
  """Return body names in MuJoCo body index order (skips world body 0)."""
  return [model.body(b).name for b in range(1, model.nbody)]


def remap_joint_array(
  src_names: list[str],
  dst_names: list[str],
  arr: np.ndarray,
  fill: float = 0.0,
) -> np.ndarray:
  """Reorder a (T, n_src) array to dst order; missing dst joints filled with `fill`."""
  T = arr.shape[0]
  out = np.full((T, len(dst_names)), fill, dtype=np.float64)
  src_idx = {n: i for i, n in enumerate(src_names)}
  for di, name in enumerate(dst_names):
    if name in src_idx:
      out[:, di] = arr[:, src_idx[name]]
  return out


# ── Quaternion helpers ──────────────────────────────────────────────────────

def make_quaternion_continuous(q: np.ndarray) -> np.ndarray:
  """Flip signs so consecutive quaternions stay in the same hemisphere."""
  q_cont = q.copy()
  for i in range(1, len(q_cont)):
    if np.dot(q_cont[i], q_cont[i - 1]) < 0:
      q_cont[i] = -q_cont[i]
  return q_cont


def normalize_quaternions(q: np.ndarray) -> np.ndarray:
  norms = np.linalg.norm(q, axis=1, keepdims=True)
  norms[norms == 0] = 1.0
  return q / norms


def gaussian_kernel(size: int, sigma: float = 1.0) -> np.ndarray:
  x = np.linspace(-size // 2, size // 2, size)
  k = np.exp(-0.5 * (x / sigma) ** 2)
  return k / k.sum()


def gaussian_filter_1d(data: np.ndarray, sigma: float = 2.0, size: int = 11) -> np.ndarray:
  kernel = gaussian_kernel(size, sigma)
  pad = size // 2
  padded = np.pad(data, pad, mode="edge")
  return np.convolve(padded, kernel, mode="valid")


def smooth_quaternion_gauss(q: np.ndarray, sigma: float = 2.0, size: int = 11) -> np.ndarray:
  q_cont = make_quaternion_continuous(q)
  q_smooth = np.zeros_like(q_cont)
  for i in range(4):
    q_smooth[:, i] = gaussian_filter_1d(q_cont[:, i], sigma=sigma, size=size)
  return normalize_quaternions(q_smooth)


# ── Forward kinematics ──────────────────────────────────────────────────────

def run_mj_forward_per_frame(
  model: Any,
  base_pos_w: np.ndarray,        # (T, 3)
  base_quat_w: np.ndarray,       # (T, 4) wxyz
  joint_pos_npz: np.ndarray,     # (T, n_npz)
  npz_joint_names: list[str],
  npz_body_names: list[str] | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
  """Run mj_forward for every frame and produce body arrays in NPZ body order.

  If `npz_body_names` is None, returns arrays in MuJoCo body order.
  Output shapes: (T, n_bodies_out, 3 or 4).
  """
  import mujoco

  T = joint_pos_npz.shape[0]
  mj_joints = get_mujoco_joint_order(model)
  mj_bodies = get_mujoco_body_order(model)
  out_body_names = npz_body_names if npz_body_names is not None else mj_bodies
  n_out = len(out_body_names)

  # NPZ joint columns → MuJoCo joint columns
  joint_pos_mj = remap_joint_array(npz_joint_names, mj_joints, joint_pos_npz)

  # MuJoCo body index → output index
  mj_idx = {n: i for i, n in enumerate(mj_bodies)}
  out_to_mj = [mj_idx.get(name) for name in out_body_names]

  out_pos = np.zeros((T, n_out, 3), dtype=np.float64)
  out_quat = np.zeros((T, n_out, 4), dtype=np.float64)
  out_quat[..., 0] = 1.0
  out_lvel = np.zeros((T, n_out, 3), dtype=np.float64)
  out_avel = np.zeros((T, n_out, 3), dtype=np.float64)

  data = mujoco.MjData(model)
  qpos_n = model.nq
  qpos_buf = np.zeros(qpos_n, dtype=np.float64)
  for t in range(T):
    qpos_buf[0:3] = base_pos_w[t]
    qpos_buf[3:7] = base_quat_w[t]
    qpos_buf[7:7 + len(mj_joints)] = joint_pos_mj[t]
    data.qpos[:] = qpos_buf
    mujoco.mj_forward(model, data)
    for out_i, mj_i in enumerate(out_to_mj):
      if mj_i is None:
        continue
      bi = mj_i + 1  # +1 skips world body
      out_pos[t, out_i] = data.xpos[bi]
      out_quat[t, out_i] = data.xquat[bi]
      out_avel[t, out_i] = data.cvel[bi, 0:3]
      out_lvel[t, out_i] = data.cvel[bi, 3:6]
  return out_pos, out_quat, out_lvel, out_avel


# ── Geom Z helpers ──────────────────────────────────────────────────────────

def lowest_z_for_geom(model: Any, data: Any, gid: int) -> float:
  """Approximate world-frame lowest z of a geom (capsule / cylinder / sphere fallback)."""
  import mujoco

  c = data.geom_xpos[gid]
  mat = data.geom_xmat[gid].reshape(3, 3)
  gt = model.geom_type[gid]
  if gt in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
    hl = model.geom_size[gid, 1]
    r = model.geom_size[gid, 0]
    return float(min(c[2] + mat[2, 2] * hl, c[2] - mat[2, 2] * hl) - r)
  if gt == mujoco.mjtGeom.mjGEOM_SPHERE:
    return float(c[2] - model.geom_size[gid, 0])
  if gt == mujoco.mjtGeom.mjGEOM_BOX:
    sz = model.geom_size[gid]
    # Eight corners in local frame, rotate, find min z.
    sx, sy, sz_ = sz
    corners = np.array([
      [sx, sy, sz_], [sx, sy, -sz_], [sx, -sy, sz_], [sx, -sy, -sz_],
      [-sx, sy, sz_], [-sx, sy, -sz_], [-sx, -sy, sz_], [-sx, -sy, -sz_],
    ])
    world = (mat @ corners.T).T + c
    return float(world[:, 2].min())
  return float(c[2])
