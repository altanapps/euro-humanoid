# Phase 0 — Berkeley Humanoid Lite walking in MuJoCo

`walk.mp4` (15 s, 720p @ 50 fps): the Berkeley Humanoid Lite (22 DoF, 16.3 kg)
walking closed-loop in MuJoCo, driven by the pretrained `policy_humanoid.onnx`
checkpoint that ships in the upstream repo. Timeline: 0–2 s stand/stabilize,
2–11 s walk forward at 0.4 m/s, 11–15 s gentle left turn. Covers ~4.4 m.

## How it works

`run_demo.py` is a standalone re-implementation of upstream's sim2sim loop
(`Berkeley-Humanoid-Lite/scripts/sim2sim/play_mujoco.py` +
`Berkeley-Humanoid-Lite-Lowlevel/.../policy/rl_controller.py`), with the
torch / gamepad / UDP plumbing removed:

- **Model:** `bhl_scene.xml` MJCF from Berkeley-Humanoid-Lite-Assets
  (nq=29, nv=28, nu=22 torque actuators).
- **Policy:** ONNX, 25 Hz (`policy_dt=0.04`). Obs (75) =
  `[cmd_vel(3), base_ang_vel(3), projected_gravity(3), joint_pos − default(22),
  joint_vel(22), prev_actions(22)]`. Action: `target = action × 0.25 + default`.
- **Low level:** joint-space PD at 2 kHz physics (`physics_dt=0.0005`),
  kp 10 (arms) / 20 (legs), kd 2, torque limits 4 / 6 Nm — all from
  `Berkeley-Humanoid-Lite/configs/policy_humanoid.yaml`.
- **Rendering:** `mujoco.Renderer` offscreen + imageio/ffmpeg.

## Rerun

```bash
cd <repo-root>   # side-projects/active/euro-humanoid

# One-time setup (already done):
#   git clone --depth 1 https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets sim/external/Berkeley-Humanoid-Lite-Assets
#   git clone --depth 1 https://github.com/HybridRobotics/Berkeley-Humanoid-Lite        sim/external/Berkeley-Humanoid-Lite
#   git clone --depth 1 https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel sim/external/Berkeley-Humanoid-Lite-Lowlevel  # reference only
#   python3.11 -m venv sim/.venv
#   sim/.venv/bin/pip install mujoco numpy imageio imageio-ffmpeg onnxruntime
#   # MJCF references assets/merged/*.stl; meshes live in meshes/ — symlink:
#   ln -sfn ../../meshes sim/external/Berkeley-Humanoid-Lite-Assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf/assets/merged

sim/.venv/bin/python sim/phase0/run_demo.py            # writes sim/phase0/walk.mp4
sim/.venv/bin/python sim/phase0/run_demo.py --seconds 30 --out /tmp/long.mp4
```

Note: `sim/external/` is gitignored. The symlink step is required after a fresh
clone of the assets repo (the MJCF's `meshdir="assets"` + `merged/` prefix does
not match the repo layout).

## Knobs

- `command_at()` in `run_demo.py` — velocity command schedule `[vx, vy, vyaw]`.
- `--width/--height/--fps/--seconds/--out` CLI flags.
- Other checkpoints in `sim/external/Berkeley-Humanoid-Lite/checkpoints/`
  (biped-only variants use the `bhl_biped_scene.xml` model and different
  configs — see `configs/*.yaml`).
