"""Operator registry & abstract base class."""
from __future__ import annotations

import abc
from typing import Any

from .bundle import MotionBundle


class Operator(abc.ABC):
  name: str = ""
  description: str = ""
  schema: dict = {}                  # JSONSchema for params (drives UI form)
  requires: set[str] = set()         # MotionBundle fields read
  produces: set[str] = set()         # MotionBundle fields written
  always_run: bool = False           # if True, runner adds it even when missing
  default_enabled: bool = True       # initial checkbox state in the UI

  @abc.abstractmethod
  def apply(self, m: MotionBundle, params: dict, ctx: "PipelineCtx") -> MotionBundle:  # noqa: F821
    ...

  # ── Metadata helpers ──────────────────────────────────────────────────────
  def defaults(self) -> dict:
    return _extract_defaults(self.schema)

  def metadata(self) -> dict:
    return {
      "name": self.name,
      "description": self.description,
      "schema": self.schema,
      "requires": sorted(self.requires),
      "produces": sorted(self.produces),
      "defaults": self.defaults(),
      "always_run": self.always_run,
      "default_enabled": self.default_enabled,
    }


def _extract_defaults(schema: dict) -> dict:
  out: dict = {}
  if not isinstance(schema, dict):
    return out
  props = schema.get("properties", {}) or {}
  for k, v in props.items():
    if isinstance(v, dict) and "default" in v:
      out[k] = v["default"]
  return out


# ── Module-level registry ───────────────────────────────────────────────────
_REGISTRY: dict[str, Operator] = {}


def register(op_cls: type[Operator]) -> type[Operator]:
  """Decorator: instantiate operator and add to registry."""
  inst = op_cls()
  if not inst.name:
    raise ValueError(f"Operator {op_cls.__name__} has no name attribute")
  if inst.name in _REGISTRY:
    raise ValueError(f"Operator name collision: {inst.name}")
  _REGISTRY[inst.name] = inst
  return op_cls


def get(name: str) -> Operator:
  if name not in _REGISTRY:
    raise KeyError(f"Unknown operator: {name!r} (have {sorted(_REGISTRY)})")
  return _REGISTRY[name]


def has(name: str) -> bool:
  return name in _REGISTRY


def all_metadata() -> list[dict]:
  return [op.metadata() for op in _REGISTRY.values()]


def names() -> list[str]:
  return list(_REGISTRY.keys())
