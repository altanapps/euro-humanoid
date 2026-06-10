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
    kind: str                 # sphere | capsule | box | cylinder
    mass: float
    pos: tuple = (0.0, 0.0, 0.0)
    size: tuple = ()          # sphere: (r,); box: half extents; cyl: (r, halflen)
    fromto: tuple = ()        # capsule only
    rgba: str = "0.55 0.57 0.6 1"
    collision: bool = False
    name: str = ""


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


COL_ACT = {"A": "0.85 0.33 0.1 1", "B": "0.95 0.65 0.15 1",
           "C": "0.3 0.55 0.85 1"}
COL_STRUCT = "0.55 0.57 0.6 1"
COL_PAYLOAD = "0.25 0.3 0.35 1"


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
    e.set("mass", fnum(g.mass))
    e.set("rgba", g.rgba)
    if g.collision:
        e.set("class", "collision")
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
    for c in body.children:
        body_to_mjcf(e, c)
    return e


def emit_mjcf(root_body: Body, hold_ctrl=None) -> str:
    mj = ET.Element("mujoco", model="euh1")
    ET.SubElement(mj, "compiler", angle="radian", autolimits="true")
    ET.SubElement(mj, "option", timestep=fnum(SIM_TIMESTEP),
                  integrator="implicitfast")

    visual = ET.SubElement(mj, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    ET.SubElement(visual, "headlight", ambient="0.3 0.3 0.3",
                  diffuse="0.6 0.6 0.6", specular="0.1 0.1 0.1")

    asset = ET.SubElement(mj, "asset")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient",
                  rgb1="0.45 0.6 0.75", rgb2="0.9 0.93 0.96",
                  width="256", height="256")
    ET.SubElement(asset, "texture", name="grid", type="2d",
                  builtin="checker", rgb1="0.22 0.24 0.26",
                  rgb2="0.28 0.30 0.32", width="512", height="512")
    ET.SubElement(asset, "material", name="grid", texture="grid",
                  texrepeat="8 8", reflectance="0.1")

    default = ET.SubElement(mj, "default")
    ET.SubElement(default, "geom", contype="0", conaffinity="0", group="1")
    dcol = ET.SubElement(default, "default", **{"class": "collision"})
    ET.SubElement(dcol, "geom", contype="1", conaffinity="1", group="0",
                  condim="4", friction=fvec(FLOOR_FRICTION))
    for cname, cls in sorted(ACTUATOR_CLASSES.items()):
        d = ET.SubElement(default, "default", **{"class": f"act{cname}"})
        ET.SubElement(d, "joint", damping=fnum(cls.kd),
                      armature=fnum(cls.armature))
        ET.SubElement(d, "position", kp=fnum(cls.kp),
                      forcerange=f"-{fnum(cls.effort)} {fnum(cls.effort)}")

    world = ET.SubElement(mj, "worldbody")
    ET.SubElement(world, "light", pos="0 -1.5 2.5", dir="0 0.5 -1",
                  directional="true")
    ET.SubElement(world, "geom", name="floor", type="plane", size="10 10 0.1",
                  material="grid", contype="1", conaffinity="1", condim="4",
                  friction=fvec(FLOOR_FRICTION), group="0")

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
