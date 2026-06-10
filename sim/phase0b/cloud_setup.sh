#!/usr/bin/env bash
# Phase 0b provisioning — Ubuntu 22.04 + NVIDIA GPU (RTX 4090 class, >=16 GB VRAM)
# Tested target: Vast.ai / RunPod instance with nvidia-docker preinstalled.
set -euo pipefail

# --- 0. sanity ---
nvidia-smi || { echo "No NVIDIA GPU visible"; exit 1; }
docker --version || { echo "Docker required"; exit 1; }

# --- 1. Isaac Lab container (2.3.x stable line) ---
# NGC image is public; pin the tag once verified against current registry.
IMAGE="nvcr.io/nvidia/isaac-lab:2.3.0"
docker pull "$IMAGE"

# --- 2. workspace ---
mkdir -p ~/euh1 && cd ~/euh1
git clone --depth 1 https://github.com/altanapps/euro-humanoid.git

# --- 3. R0 smoke test: stock G1 flat velocity task ---
docker run --rm --gpus all --network host \
  -v ~/euh1:/workspace/euh1 \
  -e OMNI_KIT_ACCEPT_EULA=YES \
  "$IMAGE" bash -lc '
    cd /workspace/isaaclab &&
    ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
      --task Isaac-Velocity-Flat-G1-v0 --headless \
      --max_iterations 3000 \
      --logger tensorboard
  '

# --- 4. R1: EUH-1 actuator params ---
# Wire euro-humanoid/sim/phase0b/euh1_g1_patch.py into a cloned G1 env config
# (see patch docstring), register as Isaac-Velocity-Flat-EUH1-v0, then:
#   ./isaaclab.sh -p .../train.py --task Isaac-Velocity-Flat-EUH1-v0 --headless --max_iterations 5000
# Add the torque-saturation metric before launching R1 — it is the headline number.

# --- 5. results out ---
# logs/rsl_rl/<run>/ -> copy checkpoints + tensorboard back into
# euro-humanoid/sim/phase0b/results/ and push.
echo "R0 done. Wire the EUH-1 patch for R1 (see sim/phase0b/README.md)."
