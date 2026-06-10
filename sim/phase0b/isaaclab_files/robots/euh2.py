# Copyright (c) 2026, euro-humanoid Phase 0b.
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for EUH-1 v1 — our OWN body (R2), not the G1 surrogate.

Asset: ``sim/model/euh1.urdf`` (23 DoF, 31.88 kg, class-B ankle-roll) converted to USD
via Isaac Lab's ``scripts/tools/convert_urdf.py`` (headless, fix_base=False, no
merge-joints — the URDF has no fixed joints). USD lives at ``/workspace/assets/euh1/``.

The "EUH2" name refers to the Phase 0b run/task numbering (R2 = own body), not a new
robot revision. Actuator classes from ``design/euh1-actuator-params.md``:

* Class A (TQ ILM-E85x30 + 10:1): hip_roll, hip_pitch, knee_pitch
* Class B (Synapticon ACTILINK-JD 10): hip_yaw, ankle_pitch, ankle_roll, waist_yaw,
  shoulder_pitch — ankle_pitch carries a stiffer PD override (kp 120 / kd 4).
* Class C (ILM-E50 + 8:1): shoulder_roll, shoulder_yaw, elbow_pitch, wrist_roll

Joint names carry no ``_joint`` suffix (URDF convention of generate_euh1.py); root link
is ``pelvis``, feet are ``left_foot`` / ``right_foot``, torso link is ``torso``.

Install on the training box as ``source/isaaclab_assets/isaaclab_assets/robots/euh2.py``.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets import ArticulationCfg

##
# EUH-1 actuator classes (same EU-part parameters as the R1 surrogate run)
##

# Class A — TQ ILM-E85x30 + Neugart PLE 10:1 (hip roll x2, hip pitch x2, knee x2)
CLASS_A = dict(
    effort_limit=80.0,  # Nm, conservative cap (peak ~100)
    saturation_effort=100.0,  # Nm, motor peak
    velocity_limit=13.5,  # rad/s @ 48 V
    armature=2.3e-2,  # kg m^2, J_rotor * N^2 (est.)
    friction=0.1,  # placeholder until sysID
)

# Class B — Synapticon ACTILINK-JD 10 (hip yaw x2, ankle pitch x2, ankle roll x2,
# waist yaw, shoulder pitch x2). v1 change vs R1: ankle_roll promoted C -> B.
CLASS_B = dict(
    effort_limit=48.0,
    saturation_effort=60.0,
    velocity_limit=20.4,
    armature=6.5e-3,  # (est.)
    friction=0.05,
)

# Class C — ILM-E50 + 8:1 (shoulder roll/yaw, elbow, wrist roll)
CLASS_C = dict(
    effort_limit=20.0,
    saturation_effort=25.0,
    velocity_limit=26.0,
    armature=1.3e-3,  # (est.)
    friction=0.05,
)

EUH2_ACTUATORS = {
    "class_a_legs": DCMotorCfg(
        joint_names_expr=[".*_hip_roll", ".*_hip_pitch", ".*_knee_pitch"],
        stiffness=120.0,
        damping=4.0,
        **CLASS_A,
    ),
    "class_b": DCMotorCfg(
        joint_names_expr=[".*_hip_yaw", ".*_ankle_roll", "waist_yaw", ".*_shoulder_pitch"],
        stiffness=60.0,
        damping=2.5,
        **CLASS_B,
    ),
    # ankle_pitch is class-B hardware but uses the stiffer leg PD gains
    "class_b_ankle_pitch": DCMotorCfg(
        joint_names_expr=[".*_ankle_pitch"],
        stiffness=120.0,
        damping=4.0,
        **CLASS_B,
    ),
    "class_c": DCMotorCfg(
        joint_names_expr=[".*_shoulder_roll", ".*_shoulder_yaw", ".*_elbow_pitch", ".*_wrist_roll"],
        stiffness=30.0,
        damping=1.5,
        **CLASS_C,
    ),
}

##
# Robot configuration
##

EUH2_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/workspace/assets/euh1/euh1.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.63),  # stand-pose pelvis height 0.625 + small clearance
        joint_pos={
            # legs: crouched stand pose
            ".*_hip_pitch": -0.3,
            ".*_knee_pitch": 0.6,
            ".*_ankle_pitch": -0.3,
            # arms: slightly out and bent
            "left_shoulder_roll": 0.12,
            "right_shoulder_roll": -0.12,
            ".*_elbow_pitch": -0.3,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators=EUH2_ACTUATORS,
)
"""EUH-1 v1 own-body robot with EU DC-motor actuator classes (Phase 0b R2)."""
