"""editor_motion v1 — robot motion NPZ/CSV/PKL editor (Flask backend).

Endpoints:
  • /robots                — list robot URDFs auto-discovered in static/.
  • /upload_motion         — load NPZ via file upload.
  • /upload_csv            — load CSV (vm_soma_retargeter format) via file upload.
  • /upload_pkl            — load PKL (vm_retargeting / crop_robot_motion_ui format).
  • /upload_bvh            — load BVH and retarget via soma_retargeter (optional dep).
  • /load_motion_by_path   — load any of {.npz, .csv, .pkl} from a server-side path.
  • /list_motions          — list motion files in a folder (filterable by extensions).
  • /save_motion           — write current motion to NPZ; stream back as download.
  • /save_csv              — write current motion to CSV.
  • /save_pkl              — write current motion to PKL (crop_robot_motion_ui schema).
  • /crop_segments         — crop multiple segments and zip them in NPZ/CSV/PKL formats.
  • /pipeline/operators    — list registered improvement operators.
  • /pipeline/run          — apply ordered operator list to a motion bundle.
  • /pipeline/diagnostics  — read-only quality report.
  • /robot/limits          — joint pos/vel/effort limits from MJCF for curve overlays.

Runs on port 5002.
"""
from __future__ import annotations

import io
import os

import numpy as np
from flask import Flask, jsonify, render_template, request, send_file

from motion_pipeline import MotionBundle, registry as op_registry, runner as pipeline_runner
from motion_pipeline.limits import extract_limits, get_model as get_pipeline_model

app = Flask(__name__)

# Anchor relative paths to this file's directory so the server works regardless
# of the cwd it was launched from (e.g. `python editor_motion/app.py` from repo
# root vs `cd editor_motion && python app.py`).
_APP_DIR = os.path.dirname(os.path.abspath(__file__))
_STATIC_DIR = os.path.join(_APP_DIR, "static")

if not os.path.exists(_STATIC_DIR):
  os.makedirs(_STATIC_DIR)

# ── Robot joint-name presets ────────────────────────────────────────────────
_M2V6_JOINT_NAMES = [
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
  "waist_joint",
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
  "left_elbow_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint",
  "left_wrist_roll_joint",
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
  "right_elbow_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint",
  "right_wrist_roll_joint",
]
_MINI_M1V1_JOINT_NAMES = [
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
  "waist_joint",
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
  "left_elbow_joint", "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
  "right_elbow_joint", "right_wrist_yaw_joint",
]
_JOINT_PRESETS: dict[int, list[str]] = {
  27: _M2V6_JOINT_NAMES,
  23: _MINI_M1V1_JOINT_NAMES,
}


@app.route("/")
def index():
  return render_template("index.html")


class _FakeNpz:
  """Dict that quacks like np.lib.npyio.NpzFile (`.files` + `__getitem__`).

  Lets us reuse `_build_motion_response()` for PKL / CSV / synthesized inputs.
  """

  def __init__(self, d):
    self._d = d
    self.files = list(d.keys())

  def __getitem__(self, k):
    return self._d[k]

  def __contains__(self, k):
    return k in self._d


def _build_motion_response(data) -> dict:
  """Convert a loaded NPZ (NpzFile) into the editor's JSON payload."""
  keys = set(data.files)
  if "joint_pos" not in keys:
    raise ValueError("npz missing 'joint_pos'")
  jp = np.asarray(data["joint_pos"])
  T = jp.shape[0]
  nj = jp.shape[1]

  def _list(name, shape=None, default=None):
    if name in keys:
      return np.asarray(data[name]).tolist()
    if default is not None:
      return default
    if shape is not None:
      return np.zeros(shape).tolist()
    return []

  auto_extracted: list[str] = []

  if "base_pos_w" in keys:
    base_pos_w = np.asarray(data["base_pos_w"]).tolist()
  elif "body_pos_w" in keys:
    bpw = np.asarray(data["body_pos_w"])
    if bpw.ndim == 3 and bpw.shape[0] == T:
      base_pos_w = bpw[:, 0, :].tolist()
      auto_extracted.append("base_pos_w")
    else:
      base_pos_w = np.zeros((T, 3)).tolist()
  else:
    base_pos_w = np.zeros((T, 3)).tolist()

  if "base_quat_w" in keys:
    base_quat_w = np.asarray(data["base_quat_w"]).tolist()
  elif "body_quat_w" in keys:
    bqw = np.asarray(data["body_quat_w"])
    if bqw.ndim == 3 and bqw.shape[0] == T:
      base_quat_w = bqw[:, 0, :].tolist()
      auto_extracted.append("base_quat_w")
    else:
      base_quat_w = [[1.0, 0.0, 0.0, 0.0]] * T
  else:
    base_quat_w = [[1.0, 0.0, 0.0, 0.0]] * T

  if "joint_names" in keys:
    joint_names = np.asarray(data["joint_names"]).tolist()
  elif nj in _JOINT_PRESETS:
    joint_names = _JOINT_PRESETS[nj]
    auto_extracted.append(f"joint_names(M2v6-{nj}dof)")
  else:
    joint_names = [f"joint_{i}" for i in range(nj)]

  return {
    "joint_pos": jp.tolist(),
    "joint_vel": _list("joint_vel", shape=(T, nj)),
    "joint_names": joint_names,
    "body_names": _list("body_names", default=[]),
    "base_pos_w": base_pos_w,
    "base_quat_w": base_quat_w,
    "body_pos_w": _list("body_pos_w", default=[]),
    "body_quat_w": _list("body_quat_w", default=[]),
    "body_lin_vel_w": _list("body_lin_vel_w", default=[]),
    "body_ang_vel_w": _list("body_ang_vel_w", default=[]),
    "fps": float(np.asarray(data["fps"]).flat[0]) if "fps" in keys else 30.0,
    "framerate": float(np.asarray(data["framerate"]).flat[0]) if "framerate" in keys else 30.0,
    "num_frames": T,
    "_missing_keys": sorted(
      {"joint_names", "body_names", "base_pos_w", "base_quat_w",
       "body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w",
       "fps", "framerate", "joint_vel"} - keys
    ),
    "_auto_extracted": auto_extracted,
  }


@app.route("/upload_motion", methods=["POST"])
def upload_motion():
  if "file" not in request.files:
    return jsonify({"error": "No file part"}), 400
  f = request.files["file"]
  try:
    data = np.load(f, allow_pickle=False)
    return jsonify(_build_motion_response(data))
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


@app.route("/robots", methods=["GET"])
def list_robots():
  """List robots in static/. Each robot dir is expected to contain a top-level
  URDF in `<robot>/urdf/*.urdf` (linkage/sub-URDFs are ignored)."""
  static_root = _STATIC_DIR
  robots = []
  if not os.path.isdir(static_root):
    return jsonify({"robots": robots})
  for name in sorted(os.listdir(static_root)):
    rdir = os.path.join(static_root, name)
    if not os.path.isdir(rdir):
      continue
    candidates = []
    urdf_dir = os.path.join(rdir, "urdf")
    if os.path.isdir(urdf_dir):
      for f in sorted(os.listdir(urdf_dir)):
        if f.lower().endswith(".urdf"):
          candidates.append(f"{name}/urdf/{f}")
    for f in sorted(os.listdir(rdir)):
      if f.lower().endswith(".urdf"):
        candidates.append(f"{name}/{f}")
    if candidates:
      robots.append({"name": name, "urdf": candidates[0], "all": candidates})
  return jsonify({"robots": robots})


_DEFAULT_MOTION_EXTS = ("npz", "csv", "pkl")


def _parse_exts(raw: str | None) -> tuple[str, ...]:
  if not raw:
    return _DEFAULT_MOTION_EXTS
  out = tuple(e.strip().lower().lstrip(".") for e in raw.split(",") if e.strip())
  return out or _DEFAULT_MOTION_EXTS


@app.route("/list_motions", methods=["GET"])
def list_motions():
  """List motion files (.npz/.csv/.pkl by default) in a server-side folder.

  Query params:
    dir   — required, absolute or ~/path folder.
    exts  — optional comma-separated extension filter (e.g. "npz,pkl").
  """
  folder = (request.args.get("dir") or "").strip()
  if not folder:
    return jsonify({"error": "dir is required"}), 400
  folder = os.path.expanduser(folder)
  if not os.path.isdir(folder):
    return jsonify({"error": f"not a directory: {folder}"}), 400
  exts = _parse_exts(request.args.get("exts"))
  files = []
  try:
    for name in sorted(os.listdir(folder)):
      ext = name.rsplit(".", 1)[-1].lower() if "." in name else ""
      if ext not in exts:
        continue
      full = os.path.join(folder, name)
      if not os.path.isfile(full):
        continue
      try:
        st = os.stat(full)
        files.append({
          "name": name,
          "ext": ext,
          "path": os.path.abspath(full),
          "size": st.st_size,
          "mtime": st.st_mtime,
        })
      except OSError:
        continue
  except OSError as e:
    return jsonify({"error": str(e)}), 500
  return jsonify({"dir": os.path.abspath(folder), "files": files, "exts": list(exts)})


def _load_by_path(path: str) -> dict:
  """Dispatch loader by file extension. Returns editor motion JSON."""
  ext = path.rsplit(".", 1)[-1].lower() if "." in path else ""
  if ext == "npz":
    data = np.load(path, allow_pickle=False)
    return _build_motion_response(data)
  if ext == "csv":
    with open(path, "r", encoding="utf-8", errors="replace") as f:
      return _parse_csv_text(f.read())
  if ext == "pkl":
    import pickle
    with open(path, "rb") as f:
      d = pickle.load(f)
    if not isinstance(d, dict):
      raise ValueError("PKL root must be a dict")
    return _parse_pkl_dict(d)
  raise ValueError(f"unsupported extension: {ext or '(none)'}")


@app.route("/load_motion_by_path", methods=["POST"])
def load_motion_by_path():
  body = request.json or {}
  path = (body.get("path") or "").strip()
  if not path:
    return jsonify({"error": "path required"}), 400
  path = os.path.expanduser(path)
  if not os.path.isfile(path):
    return jsonify({"error": f"not a file: {path}"}), 400
  try:
    resp = _load_by_path(path)
    resp["_loaded_path"] = os.path.abspath(path)
    resp["_loaded_name"] = os.path.basename(path)
    return jsonify(resp)
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


@app.route("/save_motion", methods=["POST"])
def save_motion():
  try:
    data = request.json or {}
    out = {}
    for key in (
      "joint_pos",
      "joint_vel",
      "body_pos_w",
      "body_quat_w",
      "body_lin_vel_w",
      "body_ang_vel_w",
      "base_pos_w",
      "base_quat_w",
    ):
      if key in data and data[key]:
        out[key] = np.array(data[key], dtype=np.float64)
    if "joint_names" in data:
      out["joint_names"] = np.array(data["joint_names"])
    if "body_names" in data:
      out["body_names"] = np.array(data["body_names"])
    if "fps" in data:
      try:
        out["fps"] = np.array(data["fps"])
      except Exception:
        out["fps"] = data["fps"]
    if "framerate" in data:
      out["framerate"] = np.array(data["framerate"], dtype=np.float64)

    filename = str(data.get("_filename", "edited_motion")) \
      .replace("/", "_").replace("\\", "_").strip() or "edited_motion"
    if not filename.endswith(".npz"):
      filename += ".npz"

    mem = io.BytesIO()
    np.savez(mem, **out)
    mem.seek(0)
    return send_file(
      mem,
      mimetype="application/octet-stream",
      as_attachment=True,
      download_name=filename,
    )
  except Exception as e:
    print(f"save error: {e}")
    return jsonify({"error": str(e)}), 500


# --------------------------- CSV import / export ----------------------------


def _csv_header(joint_names: list[str]) -> list[str]:
  """vm_soma_retargeter-compatible CSV header.

  Frame, root_translateX/Y/Z (cm), root_quatX/Y/Z/W, <joint names...> (rad)
  """
  return (
    ["Frame",
     "root_translateX", "root_translateY", "root_translateZ",
     "root_quatX", "root_quatY", "root_quatZ", "root_quatW"]
    + list(joint_names)
  )


def _euler_xyz_deg_to_quat_wxyz(rx_deg: float, ry_deg: float, rz_deg: float) -> tuple[float, float, float, float]:
  """Intrinsic XYZ Euler angles in degrees → quaternion [w, x, y, z]."""
  rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
  cx, sx = np.cos(rx / 2), np.sin(rx / 2)
  cy, sy = np.cos(ry / 2), np.sin(ry / 2)
  cz, sz = np.cos(rz / 2), np.sin(rz / 2)
  # XYZ intrinsic: q = qx * qy * qz
  qw = cx * cy * cz - sx * sy * sz
  qx = sx * cy * cz + cx * sy * sz
  qy = cx * sy * cz - sx * cy * sz
  qz = cx * cy * sz + sx * sy * cz
  return float(qw), float(qx), float(qy), float(qz)


def _parse_csv_text(text: str) -> dict:
  """Parse a CSV motion file into the editor's motion JSON.

  Two header variants are auto-detected:

  • vm_soma_retargeter (default):
      Frame, root_translateX/Y/Z (cm), root_quatX/Y/Z/W, <joint_names> (rad)

  • unitree_g1 / LAFAN1 style:
      Frame, root_translateX/Y/Z (cm), root_rotateX/Y/Z (deg, intrinsic XYZ Euler),
      <joint_names ending in `_dof`> (deg)
      → joint suffix `_dof` is stripped so joints match the URDF.
  """
  import csv as csv_mod
  lines = text.splitlines()
  reader = csv_mod.reader(lines)
  rows = list(reader)
  if not rows:
    raise ValueError("empty CSV")
  header = [c.strip() for c in rows[0]]
  body = rows[1:]
  if len(body) < 1:
    raise ValueError("no data rows")

  def col(name):
    try:
      return header.index(name)
    except ValueError:
      return -1

  rt_idx = [col("root_translateX"), col("root_translateY"), col("root_translateZ")]
  if any(i < 0 for i in rt_idx):
    raise ValueError("CSV missing root_translateX/Y/Z columns")

  rq_idx = [col("root_quatX"), col("root_quatY"), col("root_quatZ"), col("root_quatW")]
  rr_idx = [col("root_rotateX"), col("root_rotateY"), col("root_rotateZ")]
  if all(i >= 0 for i in rq_idx):
    root_mode = "quat"
    last_root_col = max(rq_idx)
  elif all(i >= 0 for i in rr_idx):
    root_mode = "euler_xyz_deg"
    last_root_col = max(rr_idx)
  else:
    raise ValueError("CSV missing root rotation columns: need root_quatX/Y/Z/W or root_rotateX/Y/Z")

  raw_joint_names = header[last_root_col + 1:]
  # LAFAN1-style: every joint column ends in `_dof` and values are degrees.
  joints_in_degrees = bool(raw_joint_names) and all(n.endswith("_dof") for n in raw_joint_names)
  joint_names = [n[:-len("_dof")] if joints_in_degrees and n.endswith("_dof") else n
                 for n in raw_joint_names]

  T = len(body)
  nj = len(joint_names)
  jp = np.zeros((T, nj), dtype=np.float64)
  bp = np.zeros((T, 3), dtype=np.float64)
  bq = np.zeros((T, 4), dtype=np.float64)  # editor uses [w,x,y,z]
  for ti, r in enumerate(body):
    bp[ti] = [float(r[rt_idx[0]]) / 100.0,
              float(r[rt_idx[1]]) / 100.0,
              float(r[rt_idx[2]]) / 100.0]
    if root_mode == "quat":
      qx = float(r[rq_idx[0]]); qy = float(r[rq_idx[1]])
      qz = float(r[rq_idx[2]]); qw = float(r[rq_idx[3]])
      bq[ti] = [qw, qx, qy, qz]
    else:  # euler_xyz_deg
      bq[ti] = _euler_xyz_deg_to_quat_wxyz(
        float(r[rr_idx[0]]), float(r[rr_idx[1]]), float(r[rr_idx[2]]))
    for ji in range(nj):
      v = r[last_root_col + 1 + ji].strip() if last_root_col + 1 + ji < len(r) else ""
      jp[ti, ji] = float(v) if v else 0.0
  if joints_in_degrees:
    jp = np.deg2rad(jp)

  return _build_motion_response(_FakeNpz({
    "joint_pos": jp,
    "base_pos_w": bp,
    "base_quat_w": bq,
    "joint_names": np.array(joint_names),
    "fps": np.array([30.0]),
  }))


@app.route("/upload_csv", methods=["POST"])
def upload_csv():
  if "file" not in request.files:
    return jsonify({"error": "No file part"}), 400
  f = request.files["file"]
  try:
    text = f.read().decode("utf-8", errors="replace")
    return jsonify(_parse_csv_text(text))
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


@app.route("/save_csv", methods=["POST"])
def save_csv():
  try:
    import csv as csv_mod
    data = request.json or {}
    if "joint_pos" not in data:
      return jsonify({"error": "joint_pos required"}), 400
    jp = np.asarray(data["joint_pos"])
    bp = np.asarray(data.get("base_pos_w") or np.zeros((jp.shape[0], 3)))
    bq = np.asarray(data.get("base_quat_w") or np.tile([1.0, 0.0, 0.0, 0.0], (jp.shape[0], 1)))
    joint_names = list(data.get("joint_names") or [f"joint_{i}" for i in range(jp.shape[1])])

    mem = io.StringIO()
    w = csv_mod.writer(mem)
    w.writerow(_csv_header(joint_names))
    for i in range(jp.shape[0]):
      px, py, pz = bp[i] * 100.0  # m → cm
      qw, qx, qy, qz = bq[i]
      w.writerow([i, px, py, pz, qx, qy, qz, qw, *jp[i].tolist()])

    out = io.BytesIO(mem.getvalue().encode("utf-8"))
    name = str(data.get("_loaded_name") or "edited_motion").rsplit(".", 1)[0] + ".csv"
    return send_file(out, mimetype="text/csv", as_attachment=True, download_name=name)
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


# --------------------------- PKL import / export ---------------------------
#
# Two on-disk schemas are supported:
#   (a) raw vm_retargeting style (e.g. vm_retargeting/output/motion_clip.pkl):
#         {fps, root_pos:(N,3) m, root_rot:(N,4) xyzw, dof_pos:(N,J) rad,
#          joint_names, local_body_pos, link_body_list}
#   (b) crop_robot_motion_ui.py style (output of that tool):
#         {motion_fps, motion_root_pos, motion_root_rot (wxyz),
#          motion_dof_pos, motion_local_body_pos, motion_link_body_list,
#          motion_data, source_motion, source_frame_range}
# Editor-internal schema is always wxyz + meters.


def _parse_pkl_dict(d: dict) -> dict:
  """Convert a loaded .pkl dict (either schema) into editor motion JSON."""
  if "motion_root_pos" in d:
    # crop-UI schema — wxyz on disk
    rp = np.asarray(d["motion_root_pos"], dtype=np.float64)
    rq = np.asarray(d["motion_root_rot"], dtype=np.float64)  # already wxyz
    dof = np.asarray(d["motion_dof_pos"], dtype=np.float64)
    fps = float(d.get("motion_fps", 30.0))
    body_names = list(d.get("motion_link_body_list") or [])
    local_body = d.get("motion_local_body_pos")
  elif "root_pos" in d:
    # raw vm_retargeting schema — xyzw on disk → convert to wxyz
    rp = np.asarray(d["root_pos"], dtype=np.float64)
    rq_xyzw = np.asarray(d["root_rot"], dtype=np.float64)
    if rq_xyzw.ndim == 2 and rq_xyzw.shape[1] == 4:
      rq = rq_xyzw[:, [3, 0, 1, 2]]
    else:
      raise ValueError(f"root_rot has unexpected shape {rq_xyzw.shape}")
    dof = np.asarray(d["dof_pos"], dtype=np.float64)
    fps = float(d.get("fps", 30.0))
    body_names = list(d.get("link_body_list") or [])
    local_body = d.get("local_body_pos")
  else:
    raise ValueError("PKL missing root_pos / motion_root_pos")

  # joint_names: prefer top-level, else nested motion_data dict
  joint_names = list(d.get("joint_names") or [])
  if not joint_names and isinstance(d.get("motion_data"), dict):
    joint_names = list(d["motion_data"].get("joint_names") or [])

  payload = {
    "joint_pos": dof,
    "base_pos_w": rp,
    "base_quat_w": rq,
    "fps": np.array([fps]),
  }
  if joint_names:
    payload["joint_names"] = np.array(joint_names)
  if body_names:
    payload["body_names"] = np.array(body_names)
  # body_pos_w from local_body_pos if shape matches (T, B, 3)
  if local_body is not None:
    arr = np.asarray(local_body)
    if arr.ndim == 3 and arr.shape[0] == rp.shape[0]:
      payload["body_pos_w"] = arr.astype(np.float64)
  return _build_motion_response(_FakeNpz(payload))


@app.route("/upload_pkl", methods=["POST"])
def upload_pkl():
  if "file" not in request.files:
    return jsonify({"error": "No file part"}), 400
  f = request.files["file"]
  try:
    import pickle
    d = pickle.load(f.stream)
    if not isinstance(d, dict):
      return jsonify({"error": "PKL root must be a dict"}), 400
    resp = _parse_pkl_dict(d)
    resp["_loaded_name"] = os.path.basename(f.filename or "motion.pkl")
    return jsonify(resp)
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


def _motion_to_pkl_dict(motion: dict, source: str = "", frame_range=None) -> dict:
  """Build a pickle-able dict matching crop_robot_motion_ui.py output schema."""
  jp = np.asarray(motion["joint_pos"], dtype=np.float64)
  bp = np.asarray(motion.get("base_pos_w") or np.zeros((jp.shape[0], 3)), dtype=np.float64)
  bq = np.asarray(motion.get("base_quat_w") or np.tile([1.0, 0.0, 0.0, 0.0], (jp.shape[0], 1)),
                  dtype=np.float64)
  out = {
    "motion_root_pos": bp,
    "motion_root_rot": bq,                       # wxyz (matches load_robot_motion convention)
    "motion_dof_pos": jp,
    "motion_fps": float(motion.get("fps", motion.get("framerate", 30.0))),
    "motion_local_body_pos": np.asarray(motion.get("body_pos_w") or [], dtype=np.float64),
    "motion_link_body_list": list(motion.get("body_names") or []),
    "joint_names": list(motion.get("joint_names") or []),
  }
  if source:
    out["source_motion"] = source
  if frame_range is not None:
    out["source_frame_range"] = np.asarray(list(frame_range), dtype=np.int64)
  return out


@app.route("/save_pkl", methods=["POST"])
def save_pkl():
  try:
    import pickle
    data = request.json or {}
    if "joint_pos" not in data:
      return jsonify({"error": "joint_pos required"}), 400
    out = _motion_to_pkl_dict(data, source=str(data.get("_loaded_name") or ""))
    mem = io.BytesIO()
    pickle.dump(out, mem)
    mem.seek(0)
    name = str(data.get("_loaded_name") or "edited_motion").rsplit(".", 1)[0] + ".pkl"
    return send_file(mem, mimetype="application/octet-stream", as_attachment=True, download_name=name)
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


# --------------------------- Crop multi-segment ZIP -------------------------


_FRAME_AXIS_KEYS = (
  "joint_pos", "joint_vel",
  "base_pos_w", "base_quat_w",
  "body_pos_w", "body_quat_w",
  "body_lin_vel_w", "body_ang_vel_w",
)


def _slice_motion_dict(motion: dict, s: int, e: int) -> dict:
  """Return a shallow copy of `motion` with all frame-axis arrays sliced to [s, e).

  Mirrors the JS `_sliceMotionData()` helper. `e` is exclusive (Python convention).
  """
  out = dict(motion)  # shallow copy of metadata
  for k in _FRAME_AXIS_KEYS:
    v = motion.get(k)
    if not v:
      continue
    arr = list(v) if isinstance(v, list) else v
    out[k] = arr[s:e]
  out["num_frames"] = max(0, e - s)
  return out


def _motion_to_npz_bytes(motion: dict) -> bytes:
  """Serialize editor motion JSON into NPZ bytes (same schema as /save_motion)."""
  out_arrays = {}
  for key in _FRAME_AXIS_KEYS:
    v = motion.get(key)
    if v:
      out_arrays[key] = np.asarray(v, dtype=np.float64)
  if motion.get("joint_names"):
    out_arrays["joint_names"] = np.asarray(motion["joint_names"])
  if motion.get("body_names"):
    out_arrays["body_names"] = np.asarray(motion["body_names"])
  if "fps" in motion:
    out_arrays["fps"] = np.array(motion["fps"])
  if "framerate" in motion:
    out_arrays["framerate"] = np.array(motion["framerate"], dtype=np.float64)
  mem = io.BytesIO()
  np.savez_compressed(mem, **out_arrays)
  return mem.getvalue()


def _motion_to_csv_bytes(motion: dict) -> bytes:
  """Serialize editor motion JSON into CSV bytes (vm_soma_retargeter format)."""
  import csv as csv_mod
  jp = np.asarray(motion["joint_pos"])
  bp = np.asarray(motion.get("base_pos_w") or np.zeros((jp.shape[0], 3)))
  bq = np.asarray(motion.get("base_quat_w") or np.tile([1.0, 0.0, 0.0, 0.0], (jp.shape[0], 1)))
  joint_names = list(motion.get("joint_names") or [f"joint_{i}" for i in range(jp.shape[1])])
  mem = io.StringIO()
  w = csv_mod.writer(mem)
  w.writerow(_csv_header(joint_names))
  for i in range(jp.shape[0]):
    px, py, pz = (bp[i] * 100.0).tolist()
    qw, qx, qy, qz = bq[i].tolist()
    w.writerow([i, px, py, pz, qx, qy, qz, qw, *jp[i].tolist()])
  return mem.getvalue().encode("utf-8")


def _motion_to_pkl_bytes(motion: dict, source: str = "", frame_range=None) -> bytes:
  """Serialize editor motion JSON into PKL bytes (crop_robot_motion_ui schema)."""
  import pickle
  d = _motion_to_pkl_dict(motion, source=source, frame_range=frame_range)
  return pickle.dumps(d)


@app.route("/crop_segments", methods=["POST"])
def crop_segments():
  """Crop multiple frame segments from a motion and bundle them as a ZIP.

  POST JSON body:
    {
      "motion": <full editor motion JSON>,
      "segments": [{"start": 100, "end": 250}, ...],   # `end` inclusive
      "formats": ["npz", "csv", "pkl"],                # any subset
      "filename_base": "motion_cropped"
    }
  Returns: ZIP with `<base>_seg<N>.<ext>` for each segment × format combination.
  """
  try:
    import zipfile
    body = request.json or {}
    motion = body.get("motion") or {}
    segments = body.get("segments") or []
    formats = set(body.get("formats") or ["npz", "csv", "pkl"])
    base = (body.get("filename_base") or "motion_cropped").strip() or "motion_cropped"
    base = base.replace("/", "_").replace("\\", "_")

    if "joint_pos" not in motion:
      return jsonify({"error": "motion.joint_pos required"}), 400
    if not segments:
      return jsonify({"error": "no segments provided"}), 400

    N = len(motion["joint_pos"])
    mem = io.BytesIO()
    with zipfile.ZipFile(mem, "w", zipfile.ZIP_DEFLATED) as zf:
      for i, seg in enumerate(segments, 1):
        s = int(seg.get("start", 0))
        e = int(seg.get("end", 0))
        # `end` is inclusive on the wire — convert to exclusive for Python slicing
        s = max(0, min(N - 1, s))
        e_excl = max(s + 1, min(N, e + 1))
        sub = _slice_motion_dict(motion, s, e_excl)
        name = f"{base}_seg{i}"
        if "npz" in formats:
          zf.writestr(f"{name}.npz", _motion_to_npz_bytes(sub))
        if "csv" in formats:
          zf.writestr(f"{name}.csv", _motion_to_csv_bytes(sub))
        if "pkl" in formats:
          zf.writestr(f"{name}.pkl",
                      _motion_to_pkl_bytes(sub, source=base, frame_range=(s, e_excl - 1)))
    mem.seek(0)
    return send_file(mem, mimetype="application/zip", as_attachment=True,
                     download_name=f"{base}_segments.zip")
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


# --------------------------- BVH import (via soma_retargeter) ---------------


_SOMA_AVAILABLE: bool | None = None


def _soma_check() -> tuple[bool, str]:
  global _SOMA_AVAILABLE
  if _SOMA_AVAILABLE is False:
    return False, "soma_retargeter not installed"
  try:
    import soma_retargeter  # noqa: F401
    _SOMA_AVAILABLE = True
    return True, ""
  except ImportError as e:
    _SOMA_AVAILABLE = False
    return False, f"soma_retargeter import failed: {e}"


@app.route("/upload_bvh", methods=["POST"])
def upload_bvh():
  ok, why = _soma_check()
  if not ok:
    return jsonify({"error": why}), 501
  if "file" not in request.files:
    return jsonify({"error": "No file part"}), 400
  target = (request.form.get("target") or "M2v6").strip()
  f = request.files["file"]
  import tempfile
  tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".bvh")
  try:
    f.save(tmp.name)
    tmp.close()
    # Lazy imports — heavy deps (Newton/Warp/torch may pull GPU libs).
    from soma_retargeter.assets import bvh_utils  # type: ignore
    from soma_retargeter.pipelines import NewtonPipeline  # type: ignore

    skeleton, anim_buffer = bvh_utils.load_bvh(tmp.name)
    # Best-effort: NewtonPipeline target is a robot index/name.
    # For minimal scope we surface the error if NewtonPipeline can't build.
    pipeline = NewtonPipeline(skeleton, source="soma", target=target)
    csv_buffer = pipeline.retarget(anim_buffer)
    # csv_buffer has joint_names + per-frame rows (cm + quat xyzw + joints rad).
    joint_names = list(csv_buffer.joint_names)
    rows = csv_buffer.rows  # list/array of dicts or arrays — treat as iterable
    T = len(rows)
    nj = len(joint_names)
    jp = np.zeros((T, nj), dtype=np.float64)
    bp = np.zeros((T, 3), dtype=np.float64)
    bq = np.zeros((T, 4), dtype=np.float64)
    for ti, r in enumerate(rows):
      bp[ti] = [r["root_translateX"] / 100.0, r["root_translateY"] / 100.0, r["root_translateZ"] / 100.0]
      bq[ti] = [r["root_quatW"], r["root_quatX"], r["root_quatY"], r["root_quatZ"]]
      for ji, jn in enumerate(joint_names):
        jp[ti, ji] = r.get(jn, 0.0)

    class _FakeNpz:
      def __init__(self, d):
        self._d = d; self.files = list(d.keys())
      def __getitem__(self, k): return self._d[k]
      def __contains__(self, k): return k in self._d

    fake = _FakeNpz({
      "joint_pos": jp,
      "base_pos_w": bp,
      "base_quat_w": bq,
      "joint_names": np.array(joint_names),
      "fps": np.array([float(getattr(csv_buffer, "fps", 30.0))]),
    })
    resp = _build_motion_response(fake)
    resp["_loaded_name"] = os.path.basename(f.filename or "motion.bvh")
    return jsonify(resp)
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500
  finally:
    try:
      os.unlink(tmp.name)
    except OSError:
      pass


# --------------------------- pipeline -------------------------------------

DEFAULT_XML = os.path.join(_STATIC_DIR, "M2v6", "M2v6.xml")


def _resolve_xml(req_xml: str | None) -> str:
  xml_path = req_xml or DEFAULT_XML
  if not os.path.isabs(xml_path):
    xml_path = os.path.join(_APP_DIR, xml_path)
  return xml_path


@app.route("/pipeline/operators", methods=["GET", "POST"])
def pipeline_operators():
  return jsonify({"operators": op_registry.all_metadata()})


@app.route("/pipeline/run", methods=["POST"])
def pipeline_run():
  try:
    body = request.json or {}
    motion = body.get("motion") or {}
    operators = body.get("operators") or []
    xml_path = _resolve_xml(body.get("xml_path"))
    return_diff = bool(body.get("return_diff", True))

    if "joint_pos" not in motion:
      return jsonify({"error": "motion.joint_pos required"}), 400

    bundle = MotionBundle.from_json(motion)
    result = pipeline_runner.run(bundle, operators, xml_path, return_diff=return_diff)
    return jsonify(result)
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


@app.route("/pipeline/diagnostics", methods=["POST"])
def pipeline_diagnostics():
  try:
    body = request.json or {}
    motion = body.get("motion") or {}
    xml_path = _resolve_xml(body.get("xml_path"))
    bundle = MotionBundle.from_json(motion)
    diag = pipeline_runner.diagnostics_only(bundle, xml_path)
    return jsonify({"diagnostics": diag})
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


@app.route("/robot/limits", methods=["GET"])
def robot_limits():
  try:
    xml_path = _resolve_xml(request.args.get("xml"))
    if not os.path.isfile(xml_path):
      return jsonify({"error": f"xml not found: {xml_path}"}), 400
    model = get_pipeline_model(xml_path)
    limits = extract_limits(model)
    return jsonify(limits)
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


if __name__ == "__main__":
  app.run(debug=True, port=5002)
