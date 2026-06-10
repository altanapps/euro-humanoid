"""Validate euh1.xml: 23 actuated DoF, total mass, 10 s PD standing hold.

Usage:
    sim/.venv/bin/python sim/model/validate_euh1.py [--seconds 10]

Pass criterion: root height stays within 15% of its initial (keyframe)
value for the whole hold. The PD "controller" is just the position
actuators (per-class kp) + joint damping (per-class kd) with ctrl pinned
to the stand keyframe targets.
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
MJCF = HERE / "euh1.xml"
EXPECTED_NU = 23
HEIGHT_TOL = 0.15


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)

    assert model.nu == EXPECTED_NU, f"nu={model.nu}, expected {EXPECTED_NU}"
    assert model.nv == EXPECTED_NU + 6, f"nv={model.nv}"
    total = float(sum(model.body_mass))
    print(f"actuated DoF : {model.nu}")
    print(f"total mass   : {total:.2f} kg")

    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    z0 = float(data.qpos[2])
    print(f"keyframe root height: {z0:.3f} m")

    n_steps = int(round(args.seconds / model.opt.timestep))
    min_z, max_z = z0, z0
    max_drift_xy = 0.0
    for i in range(n_steps):
        mujoco.mj_step(model, data)
        z = float(data.qpos[2])
        min_z, max_z = min(min_z, z), max(max_z, z)
        max_drift_xy = max(max_drift_xy,
                           float(np.hypot(data.qpos[0], data.qpos[1])))
        if abs(z - z0) > HEIGHT_TOL * z0:
            t = (i + 1) * model.opt.timestep
            raise SystemExit(
                f"FAIL: root height {z:.3f} m at t={t:.2f} s "
                f"(limit ±{HEIGHT_TOL * z0:.3f} around {z0:.3f})")

    zf = float(data.qpos[2])
    print(f"hold {args.seconds:.0f} s: z final {zf:.3f} m "
          f"(range {min_z:.3f}..{max_z:.3f}, "
          f"dev {100 * max(abs(min_z - z0), abs(max_z - z0)) / z0:.1f}%), "
          f"xy drift {max_drift_xy * 100:.1f} cm")
    print("PASS: robot remained standing")


if __name__ == "__main__":
    main()
