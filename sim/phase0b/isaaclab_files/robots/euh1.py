# Copyright (c) 2026, euro-humanoid Phase 0b.
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the EUH-1 humanoid: Unitree G1 morphology with EUH-1 European actuators.

The robot uses the stock Unitree G1 USD asset and initial state, but replaces all
actuator groups with :class:`DCMotorCfg` models parameterized from EU part datasheets.
Parameter provenance: euro-humanoid ``design/euh1-actuator-params.md`` (v0, 2026-06-10).
(est.) values are +/-30% until bench sysID.

Joint-name notes vs. the euh1_g1_patch.py template (verified against the stock
``G1_CFG`` in this Isaac Lab version, 2.3.2):

* The G1 USD has ``torso_joint`` (not ``waist_yaw_joint``).
* There are no wrist joints; arms end in ``.*_elbow_pitch_joint`` / ``.*_elbow_roll_joint``
  followed by 7 finger joints per hand (``_zero`` .. ``_six``).
* Finger joints are folded into Class C ("the rest") so every articulation joint
  has an actuator group.
"""

from isaaclab.actuators import DCMotorCfg

from .unitree import G1_CFG, G1_MINIMAL_CFG

##
# EUH-1 actuator classes
##

# Class A - TQ ILM-E85x30 + Neugart PLE 10:1 (hip pitch x2, hip roll x2, knee x2)
CLASS_A = dict(
    effort_limit=80.0,  # Nm, conservative cap (peak ~100)
    saturation_effort=100.0,  # Nm, motor peak
    velocity_limit=13.5,  # rad/s @ 48 V
    armature=2.3e-2,  # kg m^2, J_rotor * N^2 (est.)
    stiffness=120.0,
    damping=4.0,
    friction=0.1,  # placeholder until sysID
)

# Class B - Synapticon ACTILINK-JD 10 (hip yaw x2, ankle pitch x2, torso/waist, shoulder pitch x2)
CLASS_B = dict(
    effort_limit=48.0,
    saturation_effort=60.0,
    velocity_limit=20.4,
    armature=6.5e-3,  # (est.)
    stiffness=60.0,
    damping=2.5,
    friction=0.05,
)

# Class C - ILM-E50 + 8:1 (ankle roll x2, shoulder roll/yaw, elbow, fingers)
CLASS_C = dict(
    effort_limit=20.0,
    saturation_effort=25.0,
    velocity_limit=26.0,
    armature=1.3e-3,  # (est.)
    stiffness=30.0,
    damping=1.5,
    friction=0.05,
)

EUH1_ACTUATORS = {
    "class_a_legs": DCMotorCfg(
        joint_names_expr=[
            ".*_hip_pitch_joint",
            ".*_hip_roll_joint",
            ".*_knee_joint",
        ],
        **CLASS_A,
    ),
    "class_b": DCMotorCfg(
        joint_names_expr=[
            ".*_hip_yaw_joint",
            ".*_ankle_pitch_joint",
            "torso_joint",  # G1 USD name for the waist-yaw DoF
            ".*_shoulder_pitch_joint",
        ],
        **CLASS_B,
    ),
    "class_c": DCMotorCfg(
        joint_names_expr=[
            ".*_ankle_roll_joint",
            ".*_shoulder_roll_joint",
            ".*_shoulder_yaw_joint",
            ".*_elbow_pitch_joint",
            ".*_elbow_roll_joint",
            # G1 hand/finger joints - "the rest"
            ".*_zero_joint",
            ".*_one_joint",
            ".*_two_joint",
            ".*_three_joint",
            ".*_four_joint",
            ".*_five_joint",
            ".*_six_joint",
        ],
        **CLASS_C,
    ),
}

##
# Robot configurations (same USD asset + init state as stock G1)
##

EUH1_CFG = G1_CFG.replace(actuators=EUH1_ACTUATORS)
"""EUH-1: Unitree G1 asset with EUH-1 DC-motor actuators."""

EUH1_MINIMAL_CFG = G1_MINIMAL_CFG.replace(actuators=EUH1_ACTUATORS)
"""EUH-1 with the minimal-collision G1 asset (used for velocity tasks)."""
