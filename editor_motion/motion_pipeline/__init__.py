"""Motion improvement pipeline.

Public surface used by the Flask backend:
  - MotionBundle.from_json(d) / .to_json()
  - runner.run(bundle, ops, xml_path) → dict
  - runner.diagnostics_only(bundle, xml_path) → dict
  - registry.all_metadata() → list[dict] (operator catalog for the UI)
  - limits.extract_limits(model) → dict
"""
from .bundle import MotionBundle
from . import operators  # noqa: F401  (triggers @register)
from . import registry, runner, limits

__all__ = ["MotionBundle", "registry", "runner", "limits"]
