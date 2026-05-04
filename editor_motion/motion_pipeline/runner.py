"""Pipeline runner — orchestrates operators in order with auto-rederive."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any

from .bundle import MotionBundle
from . import registry as reg
from .limits import extract_limits, get_model


@dataclass
class PipelineCtx:
  xml_path: str
  model: Any = None
  limits: dict = field(default_factory=dict)
  warnings: list[dict] = field(default_factory=list)
  report: dict | None = None  # diagnostics operator writes here

  def warn(self, level: str, msg: str, **extra) -> None:
    self.warnings.append({"level": level, "msg": msg, **extra})

  def pop_warnings(self) -> list[dict]:
    out, self.warnings = self.warnings, []
    return out


def _make_ctx(xml_path: str) -> PipelineCtx:
  ctx = PipelineCtx(xml_path=xml_path)
  if xml_path:
    try:
      ctx.model = get_model(xml_path)
      ctx.limits = extract_limits(ctx.model)
    except Exception as e:
      ctx.warn("warn", f"failed to load mujoco model: {e}", xml_path=xml_path)
  return ctx


def run(
  bundle: MotionBundle,
  operators: list[dict],
  xml_path: str,
  return_diff: bool = True,
) -> dict:
  """Execute the pipeline.

  `operators` is an ordered list: [{name, enabled, params}, ...].
  Returns: { motion, diagnostics: {before, after}, audit, new_num_frames }.
  """
  ctx = _make_ctx(xml_path)
  audit: list[dict] = []
  dirty: set[str] = set()

  diag_before = _run_diag(bundle, ctx) if return_diff else None

  for spec in operators:
    name = spec.get("name")
    if not name:
      continue
    if not spec.get("enabled", True):
      continue
    if not reg.has(name):
      audit.append({
        "name": name, "ms": 0.0, "changed_fields": [],
        "warnings": [{"level": "error", "msg": "unknown operator"}],
      })
      continue
    op = reg.get(name)
    params = {**op.defaults(), **(spec.get("params") or {})}
    missing = op.requires - _present_fields(bundle)
    if missing:
      audit.append({
        "name": name, "ms": 0.0, "changed_fields": [],
        "warnings": [{"level": "error", "msg": f"missing required fields: {sorted(missing)}"}],
      })
      continue
    t0 = time.perf_counter()
    try:
      bundle = op.apply(bundle, params, ctx)
    except Exception as e:
      import traceback
      tb = traceback.format_exc()
      audit.append({
        "name": name, "ms": (time.perf_counter() - t0) * 1000.0,
        "changed_fields": [],
        "warnings": [{"level": "error", "msg": f"{type(e).__name__}: {e}", "traceback": tb}],
      })
      continue
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    audit.append({
      "name": name, "ms": elapsed_ms,
      "changed_fields": sorted(op.produces),
      "warnings": ctx.pop_warnings(),
    })
    dirty |= op.produces

  # Auto-rederive: if any primary kinematic field changed and the last operator
  # wasn't already rederive_kinematics, run it now to ensure FK consistency.
  if dirty & MotionBundle.PRIMARY_FIELDS:
    last_name = next(
      (a["name"] for a in reversed(audit) if not a["name"].endswith("(auto)")),
      None,
    )
    if last_name != "rederive_kinematics" and reg.has("rederive_kinematics"):
      op = reg.get("rederive_kinematics")
      t0 = time.perf_counter()
      try:
        bundle = op.apply(bundle, op.defaults(), ctx)
        audit.append({
          "name": "rederive_kinematics(auto)",
          "ms": (time.perf_counter() - t0) * 1000.0,
          "changed_fields": sorted(op.produces),
          "warnings": ctx.pop_warnings(),
        })
      except Exception as e:
        audit.append({
          "name": "rederive_kinematics(auto)",
          "ms": (time.perf_counter() - t0) * 1000.0,
          "changed_fields": [],
          "warnings": [{"level": "error", "msg": f"auto-rederive failed: {e}"}],
        })

  diag_after = _run_diag(bundle, ctx) if return_diff else None

  return {
    "motion": bundle.to_json(),
    "diagnostics": {"before": diag_before, "after": diag_after} if return_diff else None,
    "audit": audit,
    "new_num_frames": bundle.num_frames,
  }


def diagnostics_only(bundle: MotionBundle, xml_path: str) -> dict:
  ctx = _make_ctx(xml_path)
  return _run_diag(bundle, ctx)


def _run_diag(bundle: MotionBundle, ctx: PipelineCtx) -> dict:
  if not reg.has("diagnostics"):
    return {}
  op = reg.get("diagnostics")
  try:
    ctx.report = None
    op.apply(bundle, op.defaults(), ctx)
    return ctx.report or {}
  except Exception as e:
    return {"error": str(e)}


def _present_fields(b: MotionBundle) -> set[str]:
  out = {"joint_pos", "base_pos_w", "base_quat_w"}
  if b.joint_vel.size:
    out.add("joint_vel")
  if b.body_pos_w.size:
    out.add("body_pos_w")
  if b.body_quat_w.size:
    out.add("body_quat_w")
  if b.body_lin_vel_w.size:
    out.add("body_lin_vel_w")
  if b.body_ang_vel_w.size:
    out.add("body_ang_vel_w")
  return out
