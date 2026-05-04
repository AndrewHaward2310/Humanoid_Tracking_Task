"""M26 (M2v6) constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

M26_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "M2v6" / "M2v6.xml"
)
assert M26_XML.exists()


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(M26_XML))
  return spec


##
# Actuator config.
##

# Armature values from M2v6 XML joint defaults.
ARMATURE_HIP_PITCH = 0.063828        # hip_pitch joints
ARMATURE_HIP_ROLL_KNEE = 0.274752    # hip_roll + knee joints (parallel linkage)
ARMATURE_HIP_YAW_WAIST = 0.057648938  # hip_yaw + waist joints
ARMATURE_ANKLE = 0.03875             # ankle_pitch + ankle_roll joints
ARMATURE_SHOULDER_PITCH = 0.031752   # shoulder_pitch joints
ARMATURE_ARM = 0.023328              # shoulder_roll, shoulder_yaw, elbow, wrist_yaw

NATURAL_FREQ = 5 * 2.0 * 3.1415926535  # 5Hz — reduced from 10Hz: large parallel-linkage armature
# (ARMATURE_HIP_ROLL_KNEE=0.274) would make stiffness ~1085 Nm/rad at 10Hz, limiting
# action range to ±0.076 rad. At 5Hz: ~271 Nm/rad → action range ±0.30 rad, comparable to M23.
DAMPING_RATIO = 1.8

STIFFNESS_HIP_PITCH = ARMATURE_HIP_PITCH * NATURAL_FREQ**2
STIFFNESS_HIP_ROLL_KNEE = ARMATURE_HIP_ROLL_KNEE * NATURAL_FREQ**2
STIFFNESS_HIP_YAW_WAIST = ARMATURE_HIP_YAW_WAIST * NATURAL_FREQ**2
STIFFNESS_ANKLE = ARMATURE_ANKLE * NATURAL_FREQ**2
STIFFNESS_SHOULDER_PITCH = ARMATURE_SHOULDER_PITCH * NATURAL_FREQ**2
STIFFNESS_ARM = ARMATURE_ARM * NATURAL_FREQ**2

DAMPING_HIP_PITCH = 2.0 * DAMPING_RATIO * ARMATURE_HIP_PITCH * NATURAL_FREQ
DAMPING_HIP_ROLL_KNEE = 2.0 * DAMPING_RATIO * ARMATURE_HIP_ROLL_KNEE * NATURAL_FREQ
DAMPING_HIP_YAW_WAIST = 2.0 * DAMPING_RATIO * ARMATURE_HIP_YAW_WAIST * NATURAL_FREQ
DAMPING_ANKLE = 2.0 * DAMPING_RATIO * ARMATURE_ANKLE * NATURAL_FREQ
DAMPING_SHOULDER_PITCH = 2.0 * DAMPING_RATIO * ARMATURE_SHOULDER_PITCH * NATURAL_FREQ
DAMPING_ARM = 2.0 * DAMPING_RATIO * ARMATURE_ARM * NATURAL_FREQ

M26_ACTUATOR_HIP_PITCH = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_hip_pitch_joint",),
  stiffness=STIFFNESS_HIP_PITCH,
  damping=DAMPING_HIP_PITCH,
  effort_limit=130.0,
  armature=ARMATURE_HIP_PITCH,
)

M26_ACTUATOR_HIP_ROLL_KNEE = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_hip_roll_joint", ".*_knee_joint"),
  stiffness=STIFFNESS_HIP_ROLL_KNEE,
  damping=DAMPING_HIP_ROLL_KNEE,
  effort_limit=330.0,
  armature=ARMATURE_HIP_ROLL_KNEE,
)

M26_ACTUATOR_HIP_YAW_WAIST = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_hip_yaw_joint", "waist_joint"),
  stiffness=STIFFNESS_HIP_YAW_WAIST,
  damping=DAMPING_HIP_YAW_WAIST,
  effort_limit=70.0,
  armature=ARMATURE_HIP_YAW_WAIST,
)

M26_ACTUATOR_ANKLE = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_ankle_pitch_joint", ".*_ankle_roll_joint"),
  stiffness=STIFFNESS_ANKLE,
  damping=DAMPING_ANKLE,
  effort_limit=60.0,
  armature=ARMATURE_ANKLE,
)

M26_ACTUATOR_SHOULDER_PITCH = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_shoulder_pitch_joint",),
  stiffness=STIFFNESS_SHOULDER_PITCH,
  damping=DAMPING_SHOULDER_PITCH,
  effort_limit=90.0,
  armature=ARMATURE_SHOULDER_PITCH,
)

M26_ACTUATOR_ARM = BuiltinPositionActuatorCfg(
  target_names_expr=(
    ".*_shoulder_roll_joint",
    ".*_shoulder_yaw_joint",
    ".*_elbow_joint",
    ".*_wrist_yaw_joint",
    ".*_wrist_pitch_joint",
    ".*_wrist_roll_joint",
  ),
  stiffness=STIFFNESS_ARM,
  damping=DAMPING_ARM,
  effort_limit=36.0,
  armature=ARMATURE_ARM,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 1.1),
  joint_pos={
    ".*_hip_pitch_joint": -0.0,
    ".*_knee_joint": 0.0,
    ".*_ankle_pitch_joint": -0.0,
    ".*_shoulder_pitch_joint": 0.0,
    ".*_elbow_joint": 0.0,
    "left_shoulder_roll_joint": 0.0,
    "right_shoulder_roll_joint": 0.0,
  },
  joint_vel={".*": 0.0},
)

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 1.1),
  joint_pos={
    ".*_hip_pitch_joint": -0.0,
    ".*_knee_joint": 0.0,
    ".*_ankle_pitch_joint": -0.0,
    ".*_elbow_joint": 0.0,
    "left_shoulder_roll_joint": 0.0,
    "left_shoulder_pitch_joint": 0.0,
    "right_shoulder_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.0,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision",),
  condim={r"^(left|right)_foot[1-11]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1-11]_collision$": 1},
  friction={r"^(left|right)_foot[1-11]_collision$": (0.6,)},
)

FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
  geom_names_expr=(".*_collision",),
  contype=0,
  conaffinity=1,
  condim={r"^(left|right)_foot[1-11]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1-11]_collision$": 1},
  friction={r"^(left|right)_foot[1-11]_collision$": (0.6,)},
)

FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(r"^(left|right)_foot[1-11]_collision$",),
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
)

##
# Final config.
##

M26_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    M26_ACTUATOR_HIP_PITCH,
    M26_ACTUATOR_HIP_ROLL_KNEE,
    M26_ACTUATOR_HIP_YAW_WAIST,
    M26_ACTUATOR_ANKLE,
    M26_ACTUATOR_SHOULDER_PITCH,
    M26_ACTUATOR_ARM,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_m26_robot_cfg() -> EntityCfg:
  """Get a fresh M26 robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=M26_ARTICULATION,
  )


M26_ACTION_SCALE: dict[str, float] = {}
for a in M26_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    M26_ACTION_SCALE[n] = 0.25 * e / s


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_m26_robot_cfg())

  viewer.launch(robot.spec.compile())
