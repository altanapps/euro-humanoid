"""EUH-1 actuator override for the Isaac Lab Unitree G1 velocity task.

Template — wire into a clone of the stock G1 env config on the training box:
  1. `./isaaclab.sh --new` (external project) or copy
     isaaclab_tasks/manager_based/locomotion/velocity/config/g1/ into your project.
  2. In the cloned robot cfg (G1_CFG / G1_MINIMAL_CFG), replace the `actuators`
     dict with EUH1_ACTUATORS below. Joint-name regexes follow the stock G1
     USD naming — verify against the asset with
     `print(robot.joint_names)` before training; adjust regexes if Unitree
     renamed joints in your Isaac Lab version.
  3. Reduce torso link mass by ~5 kg (EventCfg add_base_mass offset, or
     directly in the articulation cfg) to approximate EUH-1's 30 kg target.

Parameter provenance: design/euh1-actuator-params.md. (est.) values are
±30% until bench sysID — train R1 with these, sweep ±30% if marginal.
"""

from isaaclab.actuators import DCMotorCfg

# Class A — ILM-E85 + Neugart PLE 10:1 (hip pitch/roll, knee)
CLASS_A = dict(
    effort_limit=80.0,        # Nm, conservative cap (peak ~100)
    saturation_effort=100.0,  # Nm, motor peak
    velocity_limit=13.5,      # rad/s @ 48 V
    armature=2.3e-2,          # kg m^2, J_rotor * N^2 (est.)
    stiffness=120.0,
    damping=4.0,
    friction=0.1,             # placeholder until sysID
)

# Class B — Synapticon ACTILINK-JD 10 (hip yaw, ankle pitch, waist, shoulder pitch)
CLASS_B = dict(
    effort_limit=48.0,
    saturation_effort=60.0,
    velocity_limit=20.4,
    armature=6.5e-3,          # (est.)
    stiffness=60.0,
    damping=2.5,
    friction=0.05,
)

# Class C — ILM-E50 + 8:1 (ankle roll, shoulder roll/yaw, elbow, wrist)
CLASS_C = dict(
    effort_limit=20.0,
    saturation_effort=25.0,
    velocity_limit=26.0,
    armature=1.3e-3,          # (est.)
    stiffness=30.0,
    damping=1.5,
    friction=0.05,
)

EUH1_ACTUATORS = {
    "class_a_legs": DCMotorCfg(
        joint_names_expr=[".*_hip_pitch_joint", ".*_hip_roll_joint", ".*_knee_joint"],
        **CLASS_A,
    ),
    "class_b": DCMotorCfg(
        joint_names_expr=[".*_hip_yaw_joint", ".*_ankle_pitch_joint",
                          "waist_yaw_joint", ".*_shoulder_pitch_joint"],
        **CLASS_B,
    ),
    "class_c": DCMotorCfg(
        joint_names_expr=[".*_ankle_roll_joint", ".*_shoulder_roll_joint",
                          ".*_shoulder_yaw_joint", ".*_elbow_.*", ".*_wrist_.*"],
        **CLASS_C,
    ),
}

# Saturation logging: add a reward/metric term computing
#   mean(|applied_torque| > 0.9 * effort_limit) per joint group
# and log via extras — this is the Phase 0b headline metric.
