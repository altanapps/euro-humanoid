"""EUH-1 body preview: turntable + push-recovery stand, rendered to MP4.

Stands at 5x position gains (kp 600/300/150 — stiff position mode; max
observed torque ~26 Nm, well inside the 80/48/20 Nm European class limits).
Deployment-level gains (120/60/30) are statically unstable for a passive
stand on this morphology; the RL locomotion policy balances dynamically at
those gains instead.

Run: sim/.venv/bin/python sim/model/render_preview.py
"""

import mujoco
import numpy as np
import imageio
from pathlib import Path

HERE = Path(__file__).parent
GAIN_SCALE = 5.0
FPS = 50
W, H = 1920, 1080

m = mujoco.MjModel.from_xml_path(str(HERE / "euh1.xml"))
m.actuator_gainprm[:, 0] *= GAIN_SCALE
m.actuator_biasprm[:, 1] *= GAIN_SCALE
m.dof_damping[6:] *= GAIN_SCALE * 0.6

d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0)
d.ctrl[:] = [m.key_qpos[0][m.jnt_qposadr[m.actuator_trnid[i, 0]]] for i in range(m.nu)]

r = mujoco.Renderer(m, H, W)
cam = mujoco.MjvCamera()
cam.distance, cam.elevation = 2.4, -12
cam.lookat[:] = [0, 0, 0.55]

# pushes: (start_step, axis, newtons) at 500 Hz physics
PUSHES = {3500: (1, 45), 5000: (0, -45), 6500: (0, 45), 8000: (1, -45)}
STEPS = 9500  # 19 s
frames = []
max_tau = 0.0
for step in range(STEPS):
    if step in PUSHES:
        ax, f = PUSHES[step]
        d.xfrc_applied[1, ax] = f
    for s0, (ax, _) in PUSHES.items():
        if step == s0 + 100:
            d.xfrc_applied[1, ax] = 0
    mujoco.mj_step(m, d)
    max_tau = max(max_tau, float(np.abs(d.actuator_force).max()))
    if step % (500 // FPS) == 0:
        t = step / 500.0
        cam.azimuth = 90 + (360 * t / 7.0 if t < 7.0 else 0)  # 7 s turntable
        r.update_scene(d, cam)
        frames.append(r.render())
    if d.qpos[2] < 0.45:
        raise SystemExit(f"FELL at t={step/500:.1f}s — regression, do not ship")

out = HERE / "euh1_preview.mp4"
imageio.mimwrite(out, frames, fps=FPS, codec="libx264", quality=8)
print(f"OK: stood {STEPS/500:.0f}s incl. four 45 N pushes, max|torque| {max_tau:.1f} Nm")
print(f"wrote {out}")

# hero still: 3/4 front view, stand keyframe
d_hero = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d_hero, 0)
mujoco.mj_forward(m, d_hero)
hero = mujoco.MjvCamera()
hero.distance, hero.elevation, hero.azimuth = 1.9, -10, 145
hero.lookat[:] = [0, 0, 0.60]
r.update_scene(d_hero, hero)
hero_out = HERE / "euh1_hero.png"
imageio.imwrite(hero_out, r.render())
print(f"wrote {hero_out}")
