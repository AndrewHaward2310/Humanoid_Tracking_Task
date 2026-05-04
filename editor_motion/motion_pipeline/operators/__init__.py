"""Importing this package triggers @register on every operator module."""
# Phase 1
from . import clamp_joint_limits  # noqa: F401
from . import smooth  # noqa: F401
from . import rederive_kinematics  # noqa: F401
from . import diagnostics  # noqa: F401
# Phase 2 (sim-to-real essentials)
from . import align_origin  # noqa: F401
from . import foot_grounding  # noqa: F401
from . import pad_safe_pose  # noqa: F401
from . import boundary_continuity  # noqa: F401
# Phase 3 (physics tightening)
from . import enforce_kinematic_limits  # noqa: F401
# Phase 4 (augmentation)
from . import mirror  # noqa: F401
