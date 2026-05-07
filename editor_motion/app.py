"""Robot Motion Editor v3 — Motion Improvement Pipeline.

Additions over v2:
  • /pipeline/operators — list registered operators (schema for UI form).
  • /pipeline/run        — apply an ordered operator list to a motion bundle.
  • /pipeline/diagnostics — read-only quality report.
  • /robot/limits        — joint pos/vel/effort limits from MJCF for overlays.
  • Pipeline auto-reruns rederive_kinematics whenever joint_pos / base_* change,
    so body_* + joint_vel stay FK-consistent.
  • Runs on port 5002 so v1/v2/v3 can coexist.
"""
from __future__ import annotations

import io
import os

import numpy as np
from flask import Flask, jsonify, render_template, request, send_file

from motion_pipeline import MotionBundle, registry as op_registry, runner as pipeline_runner
from motion_pipeline.limits import extract_limits, get_model as get_pipeline_model

app = Flask(__name__)

if not os.path.exists("static"):
  os.makedirs("static")

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
  static_root = os.path.abspath("static")
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


@app.route("/list_motions", methods=["GET"])
def list_motions():
  """List .npz files in a server-side folder (non-recursive)."""
  folder = (request.args.get("dir") or "").strip()
  if not folder:
    return jsonify({"error": "dir is required"}), 400
  folder = os.path.expanduser(folder)
  if not os.path.isdir(folder):
    return jsonify({"error": f"not a directory: {folder}"}), 400
  files = []
  try:
    for name in sorted(os.listdir(folder)):
      if not name.lower().endswith(".npz"):
        continue
      full = os.path.join(folder, name)
      if not os.path.isfile(full):
        continue
      try:
        st = os.stat(full)
        files.append({
          "name": name,
          "path": os.path.abspath(full),
          "size": st.st_size,
          "mtime": st.st_mtime,
        })
      except OSError:
        continue
  except OSError as e:
    return jsonify({"error": str(e)}), 500
  return jsonify({"dir": os.path.abspath(folder), "files": files})


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
    data = np.load(path, allow_pickle=False)
    resp = _build_motion_response(data)
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


@app.route("/upload_csv", methods=["POST"])
def upload_csv():
  if "file" not in request.files:
    return jsonify({"error": "No file part"}), 400
  f = request.files["file"]
  try:
    import csv as csv_mod
    text = f.read().decode("utf-8", errors="replace").splitlines()
    reader = csv_mod.reader(text)
    rows = list(reader)
    if not rows:
      return jsonify({"error": "empty CSV"}), 400
    header = [c.strip() for c in rows[0]]
    body = rows[1:]
    if len(body) < 1:
      return jsonify({"error": "no data rows"}), 400

    # Locate columns
    def col(name):
      try:
        return header.index(name)
      except ValueError:
        return -1

    has_frame = col("Frame") == 0  # optional first col
    base_off = 1 if has_frame else 0
    rt_idx = [col("root_translateX"), col("root_translateY"), col("root_translateZ")]
    rq_idx = [col("root_quatX"), col("root_quatY"), col("root_quatZ"), col("root_quatW")]
    if any(i < 0 for i in rt_idx + rq_idx):
      return jsonify({"error": "CSV missing root_translate*/root_quat* columns"}), 400

    # Joint columns are everything after root_quatW
    last_root_col = max(rq_idx)
    joint_names = header[last_root_col + 1:]

    T = len(body)
    nj = len(joint_names)
    jp = np.zeros((T, nj), dtype=np.float64)
    bp = np.zeros((T, 3), dtype=np.float64)
    bq = np.zeros((T, 4), dtype=np.float64)  # editor uses [w,x,y,z]
    for ti, r in enumerate(body):
      bp[ti] = [float(r[rt_idx[0]]) / 100.0,
                float(r[rt_idx[1]]) / 100.0,
                float(r[rt_idx[2]]) / 100.0]
      qx = float(r[rq_idx[0]]); qy = float(r[rq_idx[1]])
      qz = float(r[rq_idx[2]]); qw = float(r[rq_idx[3]])
      bq[ti] = [qw, qx, qy, qz]
      for ji in range(nj):
        v = r[last_root_col + 1 + ji].strip() if last_root_col + 1 + ji < len(r) else ""
        jp[ti, ji] = float(v) if v else 0.0

    # Match the schema of _build_motion_response by faking an NpzFile-like dict.
    class _FakeNpz:
      def __init__(self, d):
        self._d = d
        self.files = list(d.keys())
      def __getitem__(self, k): return self._d[k]
      def __contains__(self, k): return k in self._d

    fake = _FakeNpz({
      "joint_pos": jp,
      "base_pos_w": bp,
      "base_quat_w": bq,
      "joint_names": np.array(joint_names),
      "fps": np.array([30.0]),
    })
    resp = _build_motion_response(fake)
    resp["_loaded_path"] = ""
    return jsonify(resp)
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

DEFAULT_XML = "static/M2v6/M2v6.xml"


def _resolve_xml(req_xml: str | None) -> str:
  xml_path = req_xml or DEFAULT_XML
  if not os.path.isabs(xml_path):
    xml_path = os.path.abspath(xml_path)
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
