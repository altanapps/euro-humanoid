"""Generate euh1.xml (MuJoCo MJCF) and euh1.urdf from euh1_params.py.

Deterministic and re-runnable:
    sim/.venv/bin/python sim/model/generate_euh1.py

Approach:
  1. Build an intermediate body tree (primitives with explicit masses;
     actuator masses are spheres lumped AT the joint origins).
  2. Emit MJCF. MuJoCo computes link inertials from the massed geoms.
  3. Load the MJCF with mujoco and read back each body's mass / COM /
     principal inertia -> emit the URDF inertials from those numbers, so
     MJCF and URDF are dynamically identical by construction.

Rendering: a separate VISUAL-ONLY layer (actuator pods, clamshell limbs,
chest plates, helmet...) is emitted as zero-mass, non-colliding group-2
geoms; the massed primitives are hidden in group 3 and the collision boxes
made transparent. Physics is byte-identical with or without the visual
layer — regenerate freely, never hand-edit euh1.xml.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

from euh1_params import (ACTUATOR_CLASSES, ARM_JOINTS, FLOOR_FRICTION,
                         JOINT_GAIN_OVERRIDES, LEG_JOINTS, MASSES, MORPH,
                         N_ACTUATED, SIM_TIMESTEP, STAND_POSE, WAIST_JOINT,
                         JointDef, total_mass)

HERE = Path(__file__).resolve().parent
MJCF_OUT = HERE / "euh1.xml"
URDF_OUT = HERE / "euh1.urdf"


def fnum(x: float) -> str:
    """Stable, compact float formatting."""
    s = f"{x:.6g}"
    return "0" if s in ("-0", "-0.0") else s


def fvec(v) -> str:
    return " ".join(fnum(float(x)) for x in v)


# --------------------------------------------------------------------------
# Intermediate representation
# --------------------------------------------------------------------------

@dataclass
class Geom:
    kind: str                 # sphere | capsule | box | cylinder | ellipsoid
    mass: float
    pos: tuple = (0.0, 0.0, 0.0)
    size: tuple = ()          # sphere: (r,); box: half extents; cyl: (r, halflen)
    fromto: tuple = ()        # capsule only
    rgba: str = "0.55 0.57 0.6 1"
    collision: bool = False
    name: str = ""
    # visual-only layer (zero mass, no contacts, MJCF group 2)
    visual: bool = False
    material: str = ""
    zaxis: tuple = ()         # cylinder axis orientation
    euler: tuple = ()         # rad, compiler eulerseq xyz


@dataclass
class Joint:
    name: str
    axis: tuple
    range: tuple              # rad
    act_class: str


@dataclass
class Body:
    name: str
    pos: tuple
    joint: Joint | None = None
    geoms: list = field(default_factory=list)
    children: list = field(default_factory=list)
    visuals: list = field(default_factory=list)   # cosmetic only, zero mass


COL_ACT = {"A": "0.85 0.33 0.1 1", "B": "0.95 0.65 0.15 1",
           "C": "0.3 0.55 0.85 1"}
COL_STRUCT = "0.55 0.57 0.6 1"
COL_PAYLOAD = "0.25 0.3 0.35 1"


# --------------------------------------------------------------------------
# Visual-only layer.
#
# Physics comes EXCLUSIVELY from the massed primitives above (now hidden in
# MJCF group 3) plus the collision foot boxes (made transparent). Everything
# below is emitted with class="visual": contype 0, conaffinity 0, mass 0,
# group 2 — pure cosmetics, regenerable, never hand-edited.
# --------------------------------------------------------------------------

# (name, rgba, specular, shininess, reflectance)
MATERIALS = [
    ("alu",           "0.72 0.73 0.75 1",  0.55, 0.35, 0.0),  # brushed clamshell
    ("graphite",      "0.22 0.23 0.25 1",  0.45, 0.50, 0.0),
    ("graphite_dark", "0.14 0.15 0.16 1",  0.40, 0.55, 0.0),
    ("carbon",        "0.11 0.11 0.125 1", 0.35, 0.65, 0.0),  # matte CF tube
    ("pod_orange",    "0.56 0.23 0.06 1",  0.65, 0.55, 0.0),  # anodized, class A
    ("accent_orange", "0.82 0.38 0.10 1",  0.60, 0.50, 0.0),  # class-B ring
    ("eu_blue",       "0.06 0.22 0.55 1",  0.45, 0.45, 0.0),
    ("visor",         "0.03 0.03 0.04 1",  0.90, 0.85, 0.0),
    ("shell_white",   "0.85 0.86 0.88 1",  0.60, 0.55, 0.0),
    ("rubber",        "0.10 0.10 0.11 1",  0.08, 0.12, 0.0),
    ("studio_floor",  "0.80 0.81 0.82 1",  0.40, 0.30, 0.03),
]

# Actuator pod envelopes (r, half-length), from design/envelopes.md:
# class A ~ Ø100x60 mm, class B Synapticon JD-10 Ø106x75, class C Ø55x40.
POD_DIMS = {"A": (0.050, 0.030), "B": (0.053, 0.0375), "C": (0.0275, 0.020)}
POD_BODY_MAT = {"A": "pod_orange", "B": "graphite", "C": "graphite_dark"}
POD_HUB_MAT = {"A": "graphite_dark", "B": "graphite_dark", "C": "graphite"}


def vis(kind, *, pos=(0, 0, 0), size=(), fromto=(), material="",
        zaxis=(), euler=()) -> Geom:
    return Geom(kind, 0.0, pos=pos, size=size, fromto=fromto,
                material=material, zaxis=zaxis, euler=euler, visual=True)


def pod(act_class: str, axis, *, offset: float = 0.0,
        pos=(0, 0, 0)) -> list:
    """Actuator pod: cylinder centred on the joint, oriented along its axis.

    `offset` slides the pod along its own axis (motors sit to one side of
    the joint plane; also de-conflicts co-located orthogonal joints).
    """
    ax = np.array(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    c = tuple(np.array(pos, dtype=float) + offset * ax)
    r, hl = POD_DIMS[act_class]
    geoms = [vis("cylinder", pos=c, size=(r, hl),
                 material=POD_BODY_MAT[act_class], zaxis=axis),
             # machined hub poking through both end faces
             vis("cylinder", pos=c, size=(r * 0.42, hl + 0.004),
                 material=POD_HUB_MAT[act_class], zaxis=axis)]
    if act_class == "B":   # Synapticon identity: graphite with orange ring
        geoms.append(vis("cylinder", pos=c, size=(r + 0.0025, 0.006),
                         material="accent_orange", zaxis=axis))
    return geoms


def make_joint(jd: JointDef, side: str | None) -> Joint:
    lo, hi = jd.range_rad
    if side == "right" and jd.mirror:
        lo, hi = -hi, -lo
    name = f"{side}_{jd.name}" if side else jd.name
    return Joint(name, jd.axis, (lo, hi), jd.act_class)


def joint_gains(joint: Joint) -> tuple[float, float]:
    """(kp, kd) for a joint: class start gains unless overridden."""
    base = joint.name.split("_", 1)[-1] if joint.name.startswith(
        ("left_", "right_")) else joint.name
    if base in JOINT_GAIN_OVERRIDES:
        return JOINT_GAIN_OVERRIDES[base]
    cls = ACTUATOR_CLASSES[joint.act_class]
    return cls.kp, cls.kd


def act_sphere(jd: JointDef, r: float) -> Geom:
    cls = ACTUATOR_CLASSES[jd.act_class]
    return Geom("sphere", cls.mass, size=(r,), rgba=COL_ACT[jd.act_class],
                name=f"act_{jd.act_class}")


def build_leg(side: str) -> Body:
    s = 1.0 if side == "left" else -1.0
    m, ms = MORPH, MASSES
    hy, hr, hp, kn, ap, ar = LEG_JOINTS

    foot = Body(f"{side}_foot", (0, 0, 0), make_joint(ar, side), geoms=[
        act_sphere(ar, 0.040),
        Geom("cylinder", ms.ankle_ft, pos=(0, 0, -0.022),
             size=(0.024, 0.0131), rgba=COL_PAYLOAD, name="bota_ft"),
        Geom("box", ms.foot_struct,
             pos=(m.foot_forward, 0, -(m.ankle_height - 0.015)),
             size=(m.foot_len / 2, m.foot_width / 2, 0.015),
             rgba="0.2 0.2 0.22 1", collision=True, name=f"{side}_foot_box"),
    ])
    ankle = Body(f"{side}_ankle_pitch_link", (0, 0, -m.shank_len),
                 make_joint(ap, side), geoms=[act_sphere(ap, 0.045)],
                 children=[foot])
    shank = Body(f"{side}_shank", (0, 0, -m.thigh_len),
                 make_joint(kn, side), geoms=[
        act_sphere(kn, 0.055),
        Geom("capsule", ms.shank_struct, size=(m.shank_radius,),
             fromto=(0, 0, -0.05, 0, 0, -(m.shank_len - 0.05))),
    ], children=[ankle])
    thigh = Body(f"{side}_thigh", (0, 0, 0), make_joint(hp, side), geoms=[
        act_sphere(hp, 0.060),
        Geom("capsule", ms.thigh_struct, size=(m.thigh_radius,),
             fromto=(0, 0, -0.06, 0, 0, -(m.thigh_len - 0.05))),
    ], children=[shank])
    hip_roll = Body(f"{side}_hip_roll_link", (0, 0, 0), make_joint(hr, side),
                    geoms=[act_sphere(hr, 0.050)], children=[thigh])
    hip_yaw = Body(f"{side}_hip_yaw_link", (0, s * m.hip_separation / 2, 0),
                   make_joint(hy, side), geoms=[act_sphere(hy, 0.045)],
                   children=[hip_roll])

    # ---- visual layer (zero mass) ----
    foot.visuals = pod("C", ar.axis, offset=0.028) + [
        vis("cylinder", pos=(0, 0, -0.021), size=(0.027, 0.007),
            material="graphite"),                       # Bota FT puck
        vis("box", pos=(-0.035, 0, -0.031), size=(0.035, 0.037, 0.019),
            material="rubber"),                         # heel block
        vis("box", pos=(0.065, 0, -0.043), size=(0.065, 0.037, 0.007),
            material="rubber"),                         # toe plate
        vis("box", pos=(0.045, 0, -0.029), size=(0.038, 0.030, 0.009),
            material="carbon"),                         # midfoot bridge
    ]
    ankle.visuals = pod("B", ap.axis, offset=s * 0.012)
    shank.visuals = pod("A", kn.axis, offset=s * 0.015) + [
        vis("box", pos=(0, 0, -0.045), size=(0.040, 0.044, 0.030),
            material="graphite_dark"),                  # knee clevis block
        vis("cylinder", pos=(0, 0, -0.1665), size=(0.037, 0.0885),
            material="carbon"),                         # CF shin tube
        vis("cylinder", pos=(0, 0, -0.085), size=(0.0395, 0.009),
            material="graphite"),                       # clamp rings
        vis("cylinder", pos=(0, 0, -0.245), size=(0.0395, 0.009),
            material="graphite"),
        vis("box", pos=(0, 0, -0.272), size=(0.030, 0.038, 0.022),
            material="graphite_dark"),                  # ankle clevis
    ]
    thigh.visuals = pod("A", hp.axis, offset=s * 0.02) + [
        vis("capsule", fromto=(0, 0, -0.075, 0, 0, -0.245), size=(0.053,),
            material="alu"),                            # clamshell
        vis("box", pos=(0.0515, 0, -0.16), size=(0.0045, 0.0075, 0.062),
            material="graphite_dark"),                  # accent strips
        vis("box", pos=(0, s * 0.0515, -0.16), size=(0.0075, 0.0045, 0.062),
            material="graphite_dark"),
    ]
    hip_roll.visuals = pod("A", hr.axis, offset=-0.008)
    hip_yaw.visuals = pod("B", hy.axis, offset=0.042)
    return hip_yaw


def build_arm(side: str) -> Body:
    s = 1.0 if side == "left" else -1.0
    m, ms = MORPH, MASSES
    sp, sr, sy, el, wr = ARM_JOINTS

    hand = Body(f"{side}_hand", (0, 0, -m.forearm_len),
                make_joint(wr, side), geoms=[
        act_sphere(wr, 0.035),
        Geom("box", ms.hand_struct, pos=(0, 0, -0.045),
             size=(0.015, 0.025, 0.030), rgba=COL_PAYLOAD, name="gripper"),
    ])
    forearm = Body(f"{side}_forearm", (0, 0, -m.upper_arm_len),
                   make_joint(el, side), geoms=[
        act_sphere(el, 0.040),
        Geom("capsule", ms.forearm_struct, size=(m.forearm_radius,),
             fromto=(0, 0, -0.05, 0, 0, -(m.forearm_len - 0.04))),
    ], children=[hand])
    upper_arm = Body(f"{side}_upper_arm", (0, 0, 0), make_joint(sy, side),
                     geoms=[
        act_sphere(sy, 0.035),
        Geom("capsule", ms.upper_arm_struct, size=(m.upper_arm_radius,),
             fromto=(0, 0, -0.05, 0, 0, -(m.upper_arm_len - 0.04))),
    ], children=[forearm])
    sh_roll = Body(f"{side}_shoulder_roll_link", (0, 0, 0),
                   make_joint(sr, side), geoms=[act_sphere(sr, 0.040)],
                   children=[upper_arm])
    sh_pitch = Body(f"{side}_shoulder_pitch_link",
                    (0, s * MORPH.shoulder_separation / 2,
                     m.torso_len - m.waist_above_pelvis),
                    make_joint(sp, side), geoms=[act_sphere(sp, 0.045)],
                    children=[sh_roll])

    # ---- visual layer (zero mass) ----
    hand.visuals = pod("C", wr.axis, offset=-0.008) + [
        vis("box", pos=(0, 0, -0.040), size=(0.014, 0.024, 0.018),
            material="graphite"),                       # palm
        vis("box", pos=(0, 0.014, -0.066), size=(0.009, 0.005, 0.014),
            material="rubber"),                         # fingers
        vis("box", pos=(0, -0.014, -0.066), size=(0.009, 0.005, 0.014),
            material="rubber"),
    ]
    forearm.visuals = pod("C", el.axis, offset=s * 0.012) + [
        vis("box", pos=(0, 0, -0.028), size=(0.024, 0.030, 0.022),
            material="graphite_dark"),                  # elbow clevis block
        vis("cylinder", pos=(0, 0, -0.1125), size=(0.030, 0.0625),
            material="carbon"),                         # CF forearm tube
        vis("cylinder", pos=(0, 0, -0.19), size=(0.027, 0.008),
            material="graphite"),                       # wrist collar
    ]
    upper_arm.visuals = pod("C", sy.axis, offset=-0.032) + [
        vis("capsule", fromto=(0, 0, -0.065, 0, 0, -0.175), size=(0.038,),
            material="alu"),                            # clamshell
        vis("box", pos=(0, s * 0.0365, -0.12), size=(0.010, 0.0045, 0.042),
            material="graphite_dark"),                  # accent strip
    ]
    sh_roll.visuals = pod("C", sr.axis, offset=0.005)
    sh_pitch.visuals = pod("B", sp.axis, offset=s * 0.02)
    return sh_pitch


def build_robot() -> Body:
    m, ms = MORPH, MASSES
    shoulder_z = m.torso_len - m.waist_above_pelvis  # in torso frame
    torso = Body("torso", (0, 0, m.waist_above_pelvis),
                 make_joint(WAIST_JOINT, None), geoms=[
        act_sphere(WAIST_JOINT, 0.050),
        Geom("capsule", ms.torso_struct, size=(m.torso_radius,),
             fromto=(0, 0, 0.03, 0, 0, shoulder_z - 0.03)),
        Geom("box", ms.battery, pos=(-0.06, 0, 0.13),
             size=(0.042, 0.047, 0.132), rgba=COL_PAYLOAD, name="battery"),
        Geom("box", ms.compute_vision, pos=(0.07, 0, 0.24),
             size=(0.040, 0.050, 0.040), rgba=COL_PAYLOAD,
             name="compute_vision"),
        Geom("sphere", ms.head, pos=(0, 0, shoulder_z + m.head_offset),
             size=(m.head_radius,), rgba="0.85 0.85 0.88 1", name="head"),
    ], children=[build_arm("left"), build_arm("right")])

    pelvis = Body("pelvis", (0, 0, 0), None, geoms=[
        Geom("capsule", ms.pelvis_struct, size=(m.pelvis_radius,),
             fromto=(0, -0.08, 0.05, 0, 0.08, 0.05), name="pelvis_struct"),
    ], children=[build_leg("left"), build_leg("right"), torso])

    # ---- visual layer (zero mass) ----
    torso.visuals = pod("B", WAIST_JOINT.axis) + [
        vis("box", pos=(0, 0, 0.16), size=(0.072, 0.105, 0.125),
            material="alu"),                            # torso shell
        # chest plates, slight V (apex on the centreline)
        vis("box", pos=(0.080, 0.052, 0.13), size=(0.009, 0.058, 0.075),
            euler=(0, 0, 0.16), material="alu"),
        vis("box", pos=(0.080, -0.052, 0.13), size=(0.009, 0.058, 0.075),
            euler=(0, 0, -0.16), material="alu"),
        vis("box", pos=(0.094, 0, 0.13), size=(0.0045, 0.0065, 0.0755),
            material="graphite"),                       # centre ridge
        # recessed stereo-camera sensor bar (torso-mounted, envelopes.md n.3)
        vis("box", pos=(0.079, 0, 0.235), size=(0.009, 0.064, 0.019),
            material="graphite"),                       # bezel
        vis("box", pos=(0.080, 0, 0.235), size=(0.0055, 0.054, 0.0105),
            material="visor"),                          # glass, 2.5 mm recess
        # EU-blue accent stripe
        vis("box", pos=(0.079, 0, 0.211), size=(0.009, 0.054, 0.0045),
            material="eu_blue"),
        # shoulder girdle
        vis("box", pos=(0, 0, 0.285), size=(0.055, 0.150, 0.032),
            material="graphite"),
        # back battery panel + latch + vents
        vis("box", pos=(-0.080, 0, 0.15), size=(0.012, 0.062, 0.115),
            material="graphite_dark"),
        vis("box", pos=(-0.0905, 0, 0.235), size=(0.0025, 0.028, 0.005),
            material="accent_orange"),
        vis("box", pos=(-0.0925, 0, 0.115), size=(0.0008, 0.042, 0.0035),
            material="visor"),
        vis("box", pos=(-0.0925, 0, 0.085), size=(0.0008, 0.042, 0.0035),
            material="visor"),
        # neck + helmet + visor band
        vis("cylinder", pos=(0, 0, shoulder_z + 0.015), size=(0.024, 0.022),
            material="graphite"),
        vis("ellipsoid", pos=(0.004, 0, shoulder_z + m.head_offset + 0.003),
            size=(0.084, 0.081, 0.091), material="shell_white"),
        vis("ellipsoid", pos=(0.042, 0, shoulder_z + m.head_offset + 0.012),
            size=(0.050, 0.066, 0.026), material="visor"),
    ]
    pelvis.visuals = [
        vis("box", pos=(0, 0, 0.045), size=(0.075, 0.10, 0.040),
            material="alu"),                            # machined block
        vis("box", pos=(0, 0, 0.0), size=(0.062, 0.085, 0.018),
            material="graphite"),                       # chamfer step (lower)
        vis("box", pos=(0, 0, 0.088), size=(0.060, 0.082, 0.010),
            material="graphite"),                       # chamfer step (upper)
        vis("box", pos=(0.069, 0, 0.050), size=(0.008, 0.050, 0.018),
            material="graphite_dark"),                  # front plate
    ]
    return pelvis


# --------------------------------------------------------------------------
# Keyframe
# --------------------------------------------------------------------------

def stand_root_z() -> float:
    """Root (hip-line) height in the stand pose: feet flat, knees bent."""
    q_hp = STAND_POSE["hip_pitch"]
    q_kn = STAND_POSE["knee_pitch"]
    drop = (MORPH.thigh_len * math.cos(q_hp)
            + MORPH.shank_len * math.cos(q_hp + q_kn))
    return MORPH.ankle_height + drop + 0.002


def joint_dfs(body: Body) -> list[Joint]:
    out = []
    if body.joint:
        out.append(body.joint)
    for c in body.children:
        out.extend(joint_dfs(c))
    return out


def stand_value(joint: Joint) -> float:
    base = joint.name.split("_", 1)[-1] if joint.name.split("_", 1)[0] in (
        "left", "right") else joint.name
    side = joint.name.split("_", 1)[0]
    v = STAND_POSE.get(base, 0.0)
    # mirrored joints (roll/yaw) negate the left-side value on the right
    if side == "right":
        for jd in (*LEG_JOINTS, *ARM_JOINTS):
            if jd.name == base and jd.mirror:
                v = -v
    return v


# --------------------------------------------------------------------------
# MJCF emission
# --------------------------------------------------------------------------

def geom_to_mjcf(parent: ET.Element, g: Geom, cls: str | None = None):
    e = ET.SubElement(parent, "geom")
    if g.name:
        e.set("name", g.name)
    e.set("type", g.kind)
    if g.fromto:
        e.set("fromto", fvec(g.fromto))
    else:
        if any(g.pos):
            e.set("pos", fvec(g.pos))
    if g.size:
        e.set("size", fvec(g.size))
    if g.zaxis:
        e.set("zaxis", fvec(g.zaxis))
    if g.euler:
        e.set("euler", fvec(g.euler))
    e.set("mass", fnum(g.mass))
    if g.visual:
        e.set("class", "visual")
        if g.material:
            e.set("material", g.material)
        else:
            e.set("rgba", g.rgba)
    elif g.collision:
        # collision set stays physically identical but invisible (alpha 0);
        # the rendered look comes from the visual layer.
        e.set("rgba", "0.3 0.3 0.32 0")
        e.set("class", "collision")
    else:
        e.set("rgba", g.rgba)
    return e


def unique_geom_names(body: Body):
    """Give every geom a unique, deterministic, body-prefixed name."""
    for i, g in enumerate(body.geoms):
        base = g.name or g.kind
        if not base.startswith(body.name):
            g.name = f"{body.name}_{base}_{i}"
    for c in body.children:
        unique_geom_names(c)


def body_to_mjcf(parent: ET.Element, body: Body):
    e = ET.SubElement(parent, "body", name=body.name, pos=fvec(body.pos))
    if body.joint:
        j = body.joint
        je = ET.SubElement(e, "joint", name=j.name, axis=fvec(j.axis),
                           range=fvec(j.range),
                           **{"class": f"act{j.act_class}"})
        kp, kd = joint_gains(j)
        if kd != ACTUATOR_CLASSES[j.act_class].kd:
            je.set("damping", fnum(kd))
    for g in body.geoms:
        geom_to_mjcf(e, g)
    for g in body.visuals:
        geom_to_mjcf(e, g)
    for c in body.children:
        body_to_mjcf(e, c)
    return e


def emit_mjcf(root_body: Body, hold_ctrl=None) -> str:
    mj = ET.Element("mujoco", model="euh1")
    ET.SubElement(mj, "compiler", angle="radian", autolimits="true")
    ET.SubElement(mj, "option", timestep=fnum(SIM_TIMESTEP),
                  integrator="implicitfast")

    visual = ET.SubElement(mj, "visual")
    ET.SubElement(visual, "global", offwidth="1920", offheight="1080")
    ET.SubElement(visual, "headlight", ambient="0.22 0.22 0.22",
                  diffuse="0.28 0.28 0.28", specular="0.05 0.05 0.05")

    asset = ET.SubElement(mj, "asset")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient",
                  rgb1="0.72 0.76 0.82", rgb2="0.93 0.94 0.96",
                  width="256", height="256")
    for mname, rgba, spec, shin, refl in MATERIALS:
        ET.SubElement(asset, "material", name=mname, rgba=rgba,
                      specular=fnum(spec), shininess=fnum(shin),
                      reflectance=fnum(refl))

    default = ET.SubElement(mj, "default")
    # massed primitives: physics source of truth, hidden (group 3)
    ET.SubElement(default, "geom", contype="0", conaffinity="0", group="3")
    dcol = ET.SubElement(default, "default", **{"class": "collision"})
    ET.SubElement(dcol, "geom", contype="1", conaffinity="1", group="0",
                  condim="4", friction=fvec(FLOOR_FRICTION))
    dvis = ET.SubElement(default, "default", **{"class": "visual"})
    ET.SubElement(dvis, "geom", contype="0", conaffinity="0", group="2")
    for cname, cls in sorted(ACTUATOR_CLASSES.items()):
        d = ET.SubElement(default, "default", **{"class": f"act{cname}"})
        ET.SubElement(d, "joint", damping=fnum(cls.kd),
                      armature=fnum(cls.armature))
        ET.SubElement(d, "position", kp=fnum(cls.kp),
                      forcerange=f"-{fnum(cls.effort)} {fnum(cls.effort)}")

    world = ET.SubElement(mj, "worldbody")
    # studio three-point lighting: key / fill / rim
    ET.SubElement(world, "light", name="key", pos="2.2 -2.5 3.2",
                  dir="-0.5 0.57 -0.65", directional="true",
                  diffuse="0.65 0.65 0.65", specular="0.3 0.3 0.3")
    ET.SubElement(world, "light", name="fill", pos="-2.5 2.0 2.0",
                  dir="0.6 -0.5 -0.45", directional="true",
                  diffuse="0.25 0.25 0.27", specular="0 0 0",
                  castshadow="false")
    ET.SubElement(world, "light", name="rim", pos="-2.0 -1.2 2.6",
                  dir="0.6 0.35 -0.6", directional="true",
                  diffuse="0.35 0.35 0.38", specular="0.45 0.45 0.45",
                  castshadow="false")
    ET.SubElement(world, "geom", name="floor", type="plane", size="10 10 0.1",
                  material="studio_floor", contype="1", conaffinity="1",
                  condim="4", friction=fvec(FLOOR_FRICTION), group="0")

    unique_geom_names(root_body)
    pelvis_el = body_to_mjcf(world, root_body)
    # free joint root
    pelvis_el.insert(0, ET.Element("freejoint", name="root"))
    pelvis_el.set("pos", fvec((0, 0, stand_root_z())))

    joints = joint_dfs(root_body)
    assert len(joints) == N_ACTUATED, len(joints)

    actuators = ET.SubElement(mj, "actuator")
    for j in joints:
        ae = ET.SubElement(actuators, "position", name=j.name, joint=j.name,
                           ctrlrange=fvec(j.range),
                           **{"class": f"act{j.act_class}"})
        kp, _ = joint_gains(j)
        if kp != ACTUATOR_CLASSES[j.act_class].kp:
            ae.set("kp", fnum(kp))

    qpos_joints = [stand_value(j) for j in joints]
    qpos = [0.0, 0.0, stand_root_z(), 1.0, 0.0, 0.0, 0.0] + qpos_joints
    ctrl = qpos_joints if hold_ctrl is None else list(hold_ctrl)
    kf = ET.SubElement(mj, "keyframe")
    ET.SubElement(kf, "key", name="stand", qpos=fvec(qpos), ctrl=fvec(ctrl))

    ET.indent(mj, space="  ")
    return ET.tostring(mj, encoding="unicode") + "\n"


# --------------------------------------------------------------------------
# URDF emission (inertials read back from the compiled MJCF)
# --------------------------------------------------------------------------

def quat_to_rpy(q):
    """MuJoCo quat (w,x,y,z) -> URDF fixed-axis rpy."""
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def capsule_dir_rpy(fromto):
    p1 = np.array(fromto[:3])
    p2 = np.array(fromto[3:])
    d = p2 - p1
    L = float(np.linalg.norm(d))
    c = (p1 + p2) / 2
    d = d / L
    if abs(d[2]) > 0.99:                      # along z
        rpy = (0.0, 0.0, 0.0)
    elif abs(d[1]) > 0.99:                    # along y
        rpy = (math.pi / 2, 0.0, 0.0)
    else:                                     # along x
        rpy = (0.0, math.pi / 2, 0.0)
    return c, rpy, L


def urdf_geometry(parent: ET.Element, g: Geom):
    if g.kind == "capsule":
        c, rpy, L = capsule_dir_rpy(g.fromto)
        origin = ET.SubElement(parent, "origin", xyz=fvec(c), rpy=fvec(rpy))
        geo = ET.SubElement(parent, "geometry")
        ET.SubElement(geo, "cylinder", radius=fnum(g.size[0]), length=fnum(L))
    else:
        ET.SubElement(parent, "origin", xyz=fvec(g.pos), rpy="0 0 0")
        geo = ET.SubElement(parent, "geometry")
        if g.kind == "sphere":
            ET.SubElement(geo, "sphere", radius=fnum(g.size[0]))
        elif g.kind == "box":
            ET.SubElement(geo, "box",
                          size=fvec([2 * s for s in g.size]))
        elif g.kind == "cylinder":
            ET.SubElement(geo, "cylinder", radius=fnum(g.size[0]),
                          length=fnum(2 * g.size[1]))


def emit_urdf(root_body: Body, model: mujoco.MjModel) -> str:
    robot = ET.Element("robot", name="euh1")

    def add_link(body: Body):
        link = ET.SubElement(robot, "link", name=body.name)
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body.name)
        inertial = ET.SubElement(link, "inertial")
        rpy = quat_to_rpy(model.body_iquat[bid])
        ET.SubElement(inertial, "origin", xyz=fvec(model.body_ipos[bid]),
                      rpy=fvec(rpy))
        ET.SubElement(inertial, "mass", value=fnum(model.body_mass[bid]))
        I = model.body_inertia[bid]
        ET.SubElement(inertial, "inertia", ixx=fnum(I[0]), iyy=fnum(I[1]),
                      izz=fnum(I[2]), ixy="0", ixz="0", iyz="0")
        for g in body.geoms:
            vis = ET.SubElement(link, "visual")
            urdf_geometry(vis, g)
            if g.collision:
                col = ET.SubElement(link, "collision")
                urdf_geometry(col, g)

        for child in body.children:
            j = child.joint
            cls = ACTUATOR_CLASSES[j.act_class]
            je = ET.SubElement(robot, "joint", name=j.name, type="revolute")
            ET.SubElement(je, "origin", xyz=fvec(child.pos), rpy="0 0 0")
            ET.SubElement(je, "parent", link=body.name)
            ET.SubElement(je, "child", link=child.name)
            ET.SubElement(je, "axis", xyz=fvec(j.axis))
            ET.SubElement(je, "limit", lower=fnum(j.range[0]),
                          upper=fnum(j.range[1]), effort=fnum(cls.effort),
                          velocity=fnum(cls.velocity))
            ET.SubElement(je, "dynamics", damping=fnum(cls.kd), friction="0")
            add_link(child)

    add_link(root_body)
    ET.indent(robot, space="  ")
    return ET.tostring(robot, encoding="unicode") + "\n"


# --------------------------------------------------------------------------
# Mass report
# --------------------------------------------------------------------------

def mass_report(model: mujoco.MjModel):
    print(f"{'body':<28}{'mass kg':>9}")
    print("-" * 37)
    tot = 0.0
    for b in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        m = model.body_mass[b]
        tot += m
        print(f"{name:<28}{m:>9.3f}")
    print("-" * 37)
    print(f"{'TOTAL':<28}{tot:>9.3f}")
    print(f"(params expects {total_mass():.3f} kg)")
    print(f"class A 6x{ACTUATOR_CLASSES['A'].mass} = "
          f"{6 * ACTUATOR_CLASSES['A'].mass:.2f} kg | "
          f"class B 7x{ACTUATOR_CLASSES['B'].mass} = "
          f"{7 * ACTUATOR_CLASSES['B'].mass:.2f} kg | "
          f"class C 10x{ACTUATOR_CLASSES['C'].mass} = "
          f"{10 * ACTUATOR_CLASSES['C'].mass:.2f} kg")


def compute_hold_ctrl(model: mujoco.MjModel) -> np.ndarray:
    """Fixed-point iteration for keyframe ctrl that holds the stand pose.

    Plain position servos sag under gravity (steady-state error = gravity
    torque / kp); worse, the knee sag geometrically forces a backward body
    pitch once the ankle servo re-centres, which tips the robot. Simulating
    a settling window and adding the residual pose error onto ctrl converges
    to targets whose equilibrium IS the keyframe pose, with contact
    load-sharing handled by the physics itself. Early iterations use short
    horizons (the uncompensated robot falls after ~1.2 s) and abort on
    height loss. Deterministic: no randomness anywhere.
    """
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    target = model.key_qpos[key][7:].copy()
    z0 = model.key_qpos[key][2]
    ctrl = model.key_ctrl[key].copy()
    data = mujoco.MjData(model)
    horizons = (0.3, 0.5, 1.0, 2.0, 3.0, 3.0)
    for it, settle in enumerate(horizons):
        mujoco.mj_resetDataKeyframe(model, data, key)
        data.ctrl[:] = ctrl
        for _ in range(int(round(settle / model.opt.timestep))):
            mujoco.mj_step(model, data)
            if data.qpos[2] < 0.90 * z0:    # falling: use pre-fall error
                break
        err = target - data.qpos[7:]
        ctrl = np.clip(ctrl + 0.8 * err, model.actuator_ctrlrange[:, 0],
                       model.actuator_ctrlrange[:, 1])
        print(f"  hold-ctrl iter {it} ({settle:.1f} s): max pose err "
              f"{np.max(np.abs(err)):.4f} rad, root z {data.qpos[2]:.3f}")
    return ctrl


def main():
    robot = build_robot()
    MJCF_OUT.write_text(emit_mjcf(robot))
    model = mujoco.MjModel.from_xml_path(str(MJCF_OUT))
    assert model.nu == N_ACTUATED, f"nu={model.nu}, expected {N_ACTUATED}"

    # Keyframe ctrl stays equal to the stand pose (consistent targets).
    # The fixed-point gravity-comp pass (compute_hold_ctrl) diverges on this
    # model — at deployment-level gains the stand equilibrium is unstable
    # (soft-knee sag + ankle stiffness below the m*g*h pendulum threshold),
    # so no constant-target offset can hold it. Static stands are demoed at
    # stiffer position gains instead (see render_preview.py, gain scale 5x,
    # torques stay well under the class limits); dynamic balance at
    # deployment gains is the RL policy's job.
    print(f"wrote {MJCF_OUT}")

    urdf = emit_urdf(robot, model)
    URDF_OUT.write_text(urdf)
    print(f"wrote {URDF_OUT}")
    print()
    mass_report(model)


if __name__ == "__main__":
    main()
