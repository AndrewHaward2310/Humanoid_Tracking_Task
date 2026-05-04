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
_JOINT_PRESETS: dict[int, list[str]] = {
  27: _M2V6_JOINT_NAMES,
}


@app.route("/")
def index():
  return render_template("index.html")


@app.route("/upload_motion", methods=["POST"])
def upload_motion():
  if "file" not in request.files:
    return jsonify({"error": "No file part"}), 400
  f = request.files["file"]
  try:
    data = np.load(f, allow_pickle=False)
    keys = set(data.files)
    # Required
    if "joint_pos" not in keys:
      return jsonify({"error": "npz missing 'joint_pos'"}), 400
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

    # ── Auto-extract base_pos_w from body_pos_w[:, 0, :] ──────────────────
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

    # ── Auto-extract base_quat_w from body_quat_w[:, 0, :] ────────────────
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

    # ── Auto joint_names from robot preset ────────────────────────────────
    if "joint_names" in keys:
      joint_names = np.asarray(data["joint_names"]).tolist()
    elif nj in _JOINT_PRESETS:
      joint_names = _JOINT_PRESETS[nj]
      auto_extracted.append(f"joint_names(M2v6-{nj}dof)")
    else:
      joint_names = [f"joint_{i}" for i in range(nj)]

    resp = {
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
        {"joint_names","body_names","base_pos_w","base_quat_w",
         "body_pos_w","body_quat_w","body_lin_vel_w","body_ang_vel_w",
         "fps","framerate","joint_vel"} - keys
      ),
      "_auto_extracted": auto_extracted,
    }
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


# --------------------------- floor contact --------------------------------

_model_cache: dict = {}


def _get_model(xml_path: str):
  if xml_path in _model_cache:
    return _model_cache[xml_path]
  import mujoco

  m = mujoco.MjModel.from_xml_path(xml_path)
  _model_cache[xml_path] = m
  return m


@app.route("/check_floor_contact", methods=["POST"])
def check_floor_contact():
  """Per-frame FK → lowest world-z of left/right hand_collision + elbow_collision.

  Expects JSON:
    {
      "xml_path": "<abs or relative path>",
      "joint_names": [...],
      "joint_pos": [[...],...],
      "base_pos_w": [[x,y,z],...], "base_quat_w": [[w,x,y,z],...]
    }
  """
  try:
    import mujoco
  except ImportError:
    return jsonify({"error": "mujoco not installed"}), 500

  try:
    req = request.json or {}
    xml_path = req.get("xml_path", "static/M2v6/scene_M2v6_with_floor.xml")
    if not os.path.isabs(xml_path):
      xml_path = os.path.abspath(xml_path)
    if not os.path.isfile(xml_path):
      return jsonify({"error": f"xml not found: {xml_path}"}), 400

    model = _get_model(xml_path)
    data = mujoco.MjData(model)

    mj_joints = [
      model.joint(j).name
      for j in range(model.njnt)
      if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE
    ]
    src_jn = {n: i for i, n in enumerate(req["joint_names"])}
    jp_in = np.asarray(req["joint_pos"], dtype=np.float64)
    T = jp_in.shape[0]
    jp = np.zeros((T, len(mj_joints)))
    for di, n in enumerate(mj_joints):
      if n in src_jn:
        jp[:, di] = jp_in[:, src_jn[n]]
    bp = np.asarray(req["base_pos_w"], dtype=np.float64)
    bq = np.asarray(req["base_quat_w"], dtype=np.float64)

    checks = {
      "left_hand": "left_hand_collision",
      "right_hand": "right_hand_collision",
      "left_elbow": "left_elbow_collision",
      "right_elbow": "right_elbow_collision",
    }
    gids = {}
    for key, gname in checks.items():
      gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, gname)
      if gid >= 0:
        gids[key] = gid

    def lowest_z(gid):
      c = data.geom_xpos[gid]
      mat = data.geom_xmat[gid].reshape(3, 3)
      gt = model.geom_type[gid]
      if gt in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        hl = model.geom_size[gid, 1]
        r = model.geom_size[gid, 0]
        return float(min(c[2] + mat[2, 2] * hl, c[2] - mat[2, 2] * hl) - r)
      if gt == mujoco.mjtGeom.mjGEOM_SPHERE:
        return float(c[2] - model.geom_size[gid, 0])
      return float(c[2])

    out: dict = {k: [] for k in gids}
    hand_min: list = []
    for t in range(T):
      data.qpos[:] = np.concatenate([bp[t], bq[t], jp[t]])
      mujoco.mj_forward(model, data)
      row = {k: lowest_z(gid) for k, gid in gids.items()}
      for k, v in row.items():
        out[k].append(v)
      hand_min.append(min(row.get("left_hand", 99), row.get("right_hand", 99)))

    return jsonify({"geoms": out, "hand_min_z": hand_min})
  except Exception as e:
    import traceback
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


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
