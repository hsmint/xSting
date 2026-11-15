# xSting Hexapod High-Precision Omnidirectional Walk & Turn (`xsting_walk`)

Isaac Lab Direct RL (`skrl` PPO + `PhysX` + Body `IMU`) locomotion package for the **xSting** hexapod on fabric carpet.

## Included Files

- `xsting_walk/`
  - `xsting_walk_env_cfg.py`: Scene, robot (`xsting_training.usda`), carpet physics, IMU sensor, and gait/RL config (`74D` obs, `18D` action).
  - `xsting_walk_env.py`: Constant-velocity linear stance + $C^1$ Hermite swing tripod CPG, closed-loop PI error observations, and multi-scale precision rewards.
  - `agents/skrl_ppo_cfg.yaml`: `skrl` PPO hyperparameter configuration (`[256, 256, 128]` ELU MLP).
  - `scripts/play.py`: Real-time GUI playback and directional/rotational mode evaluation.
  - `scripts/train.py`: Single-GPU training entrypoint.
  - `scripts/train_cluster.py`: Multi-node distributed GPU training launcher with automatic SSH reverse tunnel and per-node free-VRAM env scaling.
  - `checkpoints/best_agent.pt`: Final high-precision trained policy checkpoint (`25,600` iterations across `1,408` envs).
- `xsting_description/`: Robot USD (`usd/xsting_training.usda`, `usd/xsting_meshes.usdc`), URDF, and joint/sensor configs.
- `logs/skrl/xsting_carpet_walk/2026-11-15_08-19-46_ppo_torch/`: Final training run logs, config dump (`params/`), TensorBoard events, and `checkpoints/best_agent.pt`.

## Quick Start

Extract `xsting_walk_bundle.zip` and run from the extracted root directory:

### 1. Play Trained Policy (GUI)

```bash
# Cycle all 8 walking directions + left/right turns
/opt/IsaacLab/.venv/bin/python xsting_walk/scripts/play.py \
  --num_envs 10 \
  --checkpoint xsting_walk/checkpoints/best_agent.pt

# Test a specific mode:
#   stop, forward, backward, left, right,
#   diag_fl, diag_fr, diag_rl, diag_rr,
#   turn_left, turn_right, curve_left, curve_right
/opt/IsaacLab/.venv/bin/python xsting_walk/scripts/play.py \
  --num_envs 6 \
  --dir curve_left \
  --checkpoint xsting_walk/checkpoints/best_agent.pt
```

### 2. Single-GPU Training

```bash
/opt/IsaacLab/.venv/bin/python xsting_walk/scripts/train.py \
  --headless \
  --num_envs 512
```

### 3. Multi-Node Cluster Training

```bash
/opt/IsaacLab/.venv/bin/python xsting_walk/scripts/train_cluster.py \
  --max_iterations 25600
```
