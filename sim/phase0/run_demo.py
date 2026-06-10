"""Phase 0 demo: Berkeley Humanoid Lite walking closed-loop in MuJoCo.

Replicates the upstream sim2sim pipeline (Berkeley-Humanoid-Lite/scripts/sim2sim/
play_mujoco.py + Berkeley-Humanoid-Lite-Lowlevel rl_controller.py) without the
torch / gamepad / UDP dependencies, and renders an MP4 offscreen.

Policy: checkpoints/policy_humanoid.onnx from HybridRobotics/Berkeley-Humanoid-Lite.
Obs (75) = [cmd_vel(3), base_ang_vel(3), projected_gravity(3),
            joint_pos - default(22), joint_vel(22), prev_actions(22)]
Action (22): targets = action * 0.25 + default_joint_positions, tracked by a
joint-space PD loop (kp 10 arms / 20 legs, kd 2, effort limits 4 / 6 Nm) at
2 kHz physics; the policy runs at 25 Hz (policy_dt = 0.04 s).

Usage:
    sim/.venv/bin/python sim/phase0/run_demo.py [--out walk.mp4] [--seconds 15]
"""

import argparse
from pathlib import Path

import imageio
import mujoco
import numpy as np
import onnxruntime as ort

HERE = Path(__file__).resolve().parent
EXTERNAL = HERE.parent / "external"
MJCF_PATH = (
    EXTERNAL
    / "Berkeley-Humanoid-Lite-Assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf/bhl_scene.xml"
)
POLICY_PATH = EXTERNAL / "Berkeley-Humanoid-Lite/checkpoints/policy_humanoid.onnx"

# === From Berkeley-Humanoid-Lite/configs/policy_humanoid.yaml ===
PHYSICS_DT = 0.0005
POLICY_DT = 0.04  # 25 Hz
NUM_JOINTS = 22
NUM_OBS = 75
ACTION_SCALE = 0.25
JOINT_KP = np.array([10.0] * 10 + [20.0] * 12, dtype=np.float32)
JOINT_KD = np.array([2.0] * 22, dtype=np.float32)
EFFORT_LIMITS = np.array([4.0] * 10 + [6.0] * 12, dtype=np.float32)
DEFAULT_JOINT_POS = np.array(
    [0.0] * 12 + [-0.2, 0.4, -0.3, 0.0] + [0.0, 0.0] + [-0.2, 0.4, -0.3, 0.0],
    dtype=np.float32,
)
# Sensor layout in bhl_scene.xml: [jointpos(22), jointvel(22), jointtorque(22),
#                                  imu_quat(4), imu_gyro(3), ...]
SD_QUAT = 3 * NUM_JOINTS
SD_GYRO = SD_QUAT + 4
GRAVITY_VEC = np.array([0.0, 0.0, -1.0], dtype=np.float32)


def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by the inverse of quaternion q = [w, x, y, z]."""
    q_w = q[0]
    q_vec = q[1:4]
    a = v * (2.0 * q_w**2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c


class OnnxPolicy:
    def __init__(self, checkpoint_path: Path):
        self.session = ort.InferenceSession(str(checkpoint_path))
        self.key = self.session.get_inputs()[0].name

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        return np.asarray(self.session.run(None, {self.key: obs})[0])


def command_at(t: float) -> np.ndarray:
    """Velocity command [vx, vy, vyaw] over the episode timeline."""
    if t < 2.0:
        return np.zeros(3, dtype=np.float32)  # stand / stabilize
    if t < 11.0:
        return np.array([0.4, 0.0, 0.0], dtype=np.float32)  # walk forward
    return np.array([0.2, 0.0, 0.5], dtype=np.float32)  # gentle left turn


def settle_height(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Find base z so the lowest geom sits just above the floor."""
    data.qpos[7:] = DEFAULT_JOINT_POS
    data.qpos[0:3] = [0.0, 0.0, 1.0]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    lowest = min(
        data.geom_xpos[g][2] - model.geom_rbound[g]
        for g in range(model.ngeom)
        if model.geom_bodyid[g] != 0  # skip floor
    )
    return 1.0 - lowest + 0.002


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=str(HERE / "walk.mp4"))
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=50)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    model.opt.timestep = PHYSICS_DT
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    data = mujoco.MjData(model)
    print(f"Model: nq={model.nq} nv={model.nv} nu={model.nu} "
          f"mass={sum(model.body_mass):.2f} kg")

    # Initial state: default pose, feet just above the floor.
    base_z = settle_height(model, data)
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = [0.0, 0.0, base_z]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[7:] = DEFAULT_JOINT_POS
    mujoco.mj_forward(model, data)
    print(f"Initial base height: {base_z:.3f} m")

    policy = OnnxPolicy(POLICY_PATH)
    prev_actions = np.zeros(NUM_JOINTS, dtype=np.float32)
    target_positions = DEFAULT_JOINT_POS.copy()

    substeps = int(round(POLICY_DT / PHYSICS_DT))
    render_every = int(round(1.0 / (args.fps * PHYSICS_DT)))  # physics steps/frame
    n_policy_steps = int(round(args.seconds / POLICY_DT))

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = 1  # base
    camera.distance = 2.2
    camera.elevation = -15.0
    camera.azimuth = 135.0

    writer = imageio.get_writer(args.out, fps=args.fps, macro_block_size=1,
                                codec="libx264", quality=7)
    frames = 0
    try:
        for step in range(n_policy_steps):
            t = step * POLICY_DT

            # --- Observation (mirrors RlController.update) ---
            sd = data.sensordata
            base_quat = sd[SD_QUAT:SD_QUAT + 4].astype(np.float32)
            base_ang_vel = sd[SD_GYRO:SD_GYRO + 3].astype(np.float32)
            joint_pos = sd[0:NUM_JOINTS].astype(np.float32) - DEFAULT_JOINT_POS
            joint_vel = sd[NUM_JOINTS:2 * NUM_JOINTS].astype(np.float32)
            projected_gravity = quat_rotate_inverse(base_quat, GRAVITY_VEC)
            obs = np.concatenate([
                command_at(t),
                base_ang_vel,
                projected_gravity,
                joint_pos,
                joint_vel,
                prev_actions,
            ]).astype(np.float32).reshape(1, NUM_OBS)

            # --- Policy ---
            actions = policy(obs)[0]
            prev_actions[:] = actions
            target_positions = actions * ACTION_SCALE + DEFAULT_JOINT_POS

            # --- Physics: PD torque control at 2 kHz ---
            for sub in range(substeps):
                q = data.sensordata[0:NUM_JOINTS]
                qd = data.sensordata[NUM_JOINTS:2 * NUM_JOINTS]
                torques = JOINT_KP * (target_positions - q) + JOINT_KD * (-qd)
                data.ctrl[:] = np.clip(torques, -EFFORT_LIMITS, EFFORT_LIMITS)
                mujoco.mj_step(model, data)

                if (step * substeps + sub + 1) % render_every == 0:
                    renderer.update_scene(data, camera=camera)
                    writer.append_data(renderer.render())
                    frames += 1

            # Torso height (base body frame is at ground level by design;
            # geom 1 is the torso collision box, ~0.71 m up when standing).
            torso_z = data.geom_xpos[1][2]
            if step % 50 == 0:
                print(f"t={t:5.1f}s  torso z={torso_z:.3f}  "
                      f"x={data.qpos[0]:.2f}  y={data.qpos[1]:.2f}")
            if torso_z < 0.40:
                print(f"FELL at t={t:.2f}s (torso z={torso_z:.3f})")
                break
    finally:
        writer.close()
        renderer.close()

    print(f"Wrote {frames} frames to {args.out}")
    print(f"Final base position: x={data.qpos[0]:.2f} y={data.qpos[1]:.2f} "
          f"z={data.qpos[2]:.3f}")


if __name__ == "__main__":
    main()
