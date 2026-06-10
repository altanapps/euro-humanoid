"""EUH-1 morphology + actuator parameters — single source of truth (v1).

Everything `generate_euh1.py` emits (euh1.xml MJCF + euh1.urdf) is derived
from this file. Numbers trace to:
  - design/design-v0.md          (joint map, 23 DoF, ~30 kg target)
  - design/euh1-actuator-params.md (per-class effort/velocity/armature/gains)
  - design/envelopes.md          (real part masses: ILM-E85 822 g, JD-10
                                  corrected 1.05-1.26 kg, battery ~2.9 kg,
                                  Bota MiniONE Pro 60 g, vision torso-mounted)

Conventions:
  - +X forward, +Y left, +Z up. Angles in radians internally; limits below
    are written in degrees for readability and converted once.
  - pitch joints: axis +Y, NEGATIVE = limb segment swings forward
    (hip_pitch -0.3 puts the thigh forward; knee POSITIVE = flexion).
  - roll joints: axis +X. yaw joints: axis +Z.
  - Joint limits for roll/yaw joints are specified for the LEFT side and
    mirrored (lo,hi) -> (-hi,-lo) on the right.
"""

from dataclasses import dataclass, field
from math import radians


# --------------------------------------------------------------------------
# Actuator classes (design/euh1-actuator-params.md + envelopes.md corrections)
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class ActuatorClass:
    name: str
    mass: float          # kg, lumped at the joint (motor+gear+drive+housing)
    effort: float        # Nm, sim effort cap (conservative vs datasheet peak)
    velocity: float      # rad/s, no-load output speed cap
    armature: float      # kg m^2, reflected rotor inertia at the output
    kp: float            # position-servo stiffness (start gains, to be tuned)
    kd: float            # joint damping standing in for servo derivative gain


# Class A: TQ ILM-E85x30 (822 g) + custom 10:1 planetary (~600 g, the v1
# assumption from envelopes.md note 1) + housing share -> 1.6 kg/joint.
CLASS_A = ActuatorClass("A", mass=1.60, effort=80.0, velocity=13.5,
                        armature=2.3e-2, kp=120.0, kd=4.0)

# Class B: Synapticon ACTILINK-JD 10, corrected envelope mass (1.05-1.26 kg,
# envelopes.md) -> 1.15 kg midpoint.
CLASS_B = ActuatorClass("B", mass=1.15, effort=48.0, velocity=20.4,
                        armature=6.5e-3, kp=60.0, kd=2.5)

# Class C: ILM-E50 + 8:1 planetary, ~0.5 kg with drive + housing.
CLASS_C = ActuatorClass("C", mass=0.50, effort=20.0, velocity=26.0,
                        armature=1.3e-3, kp=30.0, kd=1.5)

ACTUATOR_CLASSES = {"A": CLASS_A, "B": CLASS_B, "C": CLASS_C}

# Per-joint servo-gain overrides (kp, kd), applied to both sides.
# ankle_pitch: a passive PD stand is statically stable only if the total
# ankle-pitch stiffness exceeds m*g*h_com ~ 30.6*9.81*0.64 ~ 191 Nm/rad;
# 2 x class-B kp 60 = 120 < 191, so the robot slowly tips about the ankles.
# kp 120 per ankle (240 total) gives a 1.26x margin. Torque still saturates
# at the class-B 48 Nm effort limit, so this stays physically honest.
# (euh1-actuator-params.md marks the class gains as "start gains, tune".)
JOINT_GAIN_OVERRIDES = {
    "ankle_pitch": (120.0, 4.0),
}


# --------------------------------------------------------------------------
# Joint definitions: name, class, axis, limits (deg, left side), mirrored?
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class JointDef:
    name: str            # base name without side prefix
    act_class: str       # "A" | "B" | "C"
    axis: tuple          # joint axis in local frame
    lo_deg: float        # lower limit, LEFT side
    hi_deg: float        # upper limit, LEFT side
    mirror: bool = False # right side gets (-hi, -lo)

    @property
    def range_rad(self):
        return (radians(self.lo_deg), radians(self.hi_deg))


# Per-leg joint order (design-v0 joint map):
LEG_JOINTS = [
    JointDef("hip_yaw",     "B", (0, 0, 1), -45.0,  45.0, mirror=True),
    JointDef("hip_roll",    "A", (1, 0, 0), -30.0,  45.0, mirror=True),
    JointDef("hip_pitch",   "A", (0, 1, 0), -120.0, 60.0),
    JointDef("knee_pitch",  "A", (0, 1, 0),   0.0, 150.0),
    JointDef("ankle_pitch", "B", (0, 1, 0), -50.0,  50.0),
    JointDef("ankle_roll",  "C", (1, 0, 0), -30.0,  30.0, mirror=True),
]

WAIST_JOINT = JointDef("waist_yaw", "B", (0, 0, 1), -60.0, 60.0)

# Per-arm joint order:
ARM_JOINTS = [
    JointDef("shoulder_pitch", "B", (0, 1, 0), -150.0,  60.0),
    JointDef("shoulder_roll",  "C", (1, 0, 0),  -30.0, 160.0, mirror=True),
    JointDef("shoulder_yaw",   "C", (0, 0, 1),  -90.0,  90.0, mirror=True),
    JointDef("elbow_pitch",    "C", (0, 1, 0), -145.0,   0.0),
    JointDef("wrist_roll",     "C", (0, 0, 1), -180.0, 180.0, mirror=True),
]

# Actuated DoF: 2*6 + 1 + 2*5 = 23
N_ACTUATED = 2 * len(LEG_JOINTS) + 1 + 2 * len(ARM_JOINTS)
assert N_ACTUATED == 23


# --------------------------------------------------------------------------
# Morphology (m). Total standing height ~1.25 m:
#   foot 0.05 + shank 0.30 + thigh 0.30 = hip line at 0.65
#   + torso 0.40 (pelvis/hip-line to shoulder line) = shoulders at 1.05
#   + neck 0.12 to head centre, head r 0.08 -> crown ~1.25
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Morphology:
    # legs
    thigh_len: float = 0.30          # hip pitch axis -> knee axis
    shank_len: float = 0.30          # knee axis -> ankle axis
    ankle_height: float = 0.05       # ankle axis -> sole
    hip_separation: float = 0.20     # lateral distance between hip centres
    foot_len: float = 0.20
    foot_width: float = 0.08
    foot_forward: float = 0.03       # foot centre ahead of ankle axis
    # torso
    torso_len: float = 0.40          # pelvis (hip line) -> shoulder line
    waist_above_pelvis: float = 0.10 # waist yaw joint height above hip line
    shoulder_separation: float = 0.36
    head_offset: float = 0.12        # shoulder line -> head centre
    head_radius: float = 0.08
    # arms
    upper_arm_len: float = 0.22      # shoulder -> elbow axis
    forearm_len: float = 0.22        # elbow -> wrist axis
    hand_len: float = 0.06           # wrist -> gripper stub tip
    # structural radii (capsules)
    pelvis_radius: float = 0.06
    torso_radius: float = 0.09
    thigh_radius: float = 0.050
    shank_radius: float = 0.045
    upper_arm_radius: float = 0.035
    forearm_radius: float = 0.030


MORPH = Morphology()


# --------------------------------------------------------------------------
# Non-actuator masses (kg)
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Masses:
    battery: float = 2.90            # 13S4P Aspilsan A28 brick (envelopes.md)
    compute_vision: float = 1.70     # Jetson Orin NX + carrier + IDS stereo
    #                                  pair, torso-mounted (envelopes.md n.3)
    ankle_ft: float = 0.06           # Bota MiniONE Pro, one per foot
    # structure (CNC 7075/6061 + carbon tube shares, per link)
    pelvis_struct: float = 0.60
    torso_struct: float = 0.70
    head: float = 0.25               # shell + lights, NO heavy sensor
    thigh_struct: float = 0.25
    shank_struct: float = 0.20
    foot_struct: float = 0.15
    upper_arm_struct: float = 0.10
    forearm_struct: float = 0.08
    hand_struct: float = 0.05        # gripper stub placeholder


MASSES = Masses()


def total_mass() -> float:
    m = MASSES
    actuators = 6 * CLASS_A.mass + 7 * CLASS_B.mass + 10 * CLASS_C.mass
    structure = (m.pelvis_struct + m.torso_struct + m.head
                 + 2 * (m.thigh_struct + m.shank_struct + m.foot_struct
                        + m.upper_arm_struct + m.forearm_struct
                        + m.hand_struct))
    payload = m.battery + m.compute_vision + 2 * m.ankle_ft
    return actuators + structure + payload


# --------------------------------------------------------------------------
# Stand keyframe (rad): slightly bent knees, feet flat
# (ankle_pitch = -(hip_pitch + knee) keeps the sole parallel to the ground)
# --------------------------------------------------------------------------

STAND_POSE = {
    "hip_pitch": -0.30,
    "knee_pitch": 0.60,
    "ankle_pitch": -0.30,
    "shoulder_roll": 0.12,   # left value; mirrored joints negate on the right
    "elbow_pitch": -0.30,
}


# Simulation options
SIM_TIMESTEP = 0.002
FLOOR_FRICTION = (1.0, 0.02, 0.0001)


if __name__ == "__main__":
    print(f"actuated DoF : {N_ACTUATED}")
    print(f"class A 6 x {CLASS_A.mass:.2f} = {6 * CLASS_A.mass:.2f} kg")
    print(f"class B 7 x {CLASS_B.mass:.2f} = {7 * CLASS_B.mass:.2f} kg")
    print(f"class C 10 x {CLASS_C.mass:.2f} = {10 * CLASS_C.mass:.2f} kg")
    print(f"total mass   : {total_mass():.2f} kg")
