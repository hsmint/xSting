#!/usr/bin/env python3
"""Export MK2 skrl navigation checkpoint and static arena ESDF map for xsting_nav (ROS 2 C++)."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

import numpy as np
import torch
import torch.nn.functional as F

DEFAULT_SIM_ROOT = Path("/home/pa03/Documents/FINAL/xSting_hexapod")
if DEFAULT_SIM_ROOT.is_dir() and str(DEFAULT_SIM_ROOT) not in sys.path:
    sys.path.insert(0, str(DEFAULT_SIM_ROOT))

from nav2.leg_gait import (  # noqa: E402
    HIP0,
    HIP_Z,
    KNEE0,
    LEG_HIGH,
    LEG_LOW,
    MOUNT_X,
    MOUNT_Y,
    SIDE,
    STAND_POSE,
    YAW0,
    LegGait,
    dock_vel,
    nvblox_shield,
    quat_to_grav,
)
from nav2.nav_env import build_obs  # noqa: E402
from nav2.nav_env_cfg import (  # noqa: E402
    ARENA_CACHE,
    ARM_HOME_ELBOW,
    ARM_HOME_SHOULDER,
    BOXES,
    GOALS,
    GRID_X,
    GRID_Y,
    JOINTS,
    LIDAR_LEG_MASK_RADIUS,
    LIDAR_MAX_RANGE,
    LIDAR_MIN_RANGE,
    LIDAR_OFFSET_XYZ,
    NVBLOX_CAMS,
    START,
    ULTRASONIC_OFFSET_XYZ,
)


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def compute_static_esdf(cache_path: Path, nx: int = 55, ny: int = 79, res: float = 0.08) -> np.ndarray:
    cache = np.load(cache_path)
    esdf = cache["maze_esdf_xy"]
    org = cache["grid_origin"]
    gres = float(cache["grid_res"])
    rects = np.array(
        [(cx - sx * 0.5, cy - sy * 0.5, cx + sx * 0.5, cy + sy * 0.5) for cx, cy, sx, sy, _ in BOXES]
    )
    xs = (np.arange(nx) + 0.5) * res
    ys = (np.arange(ny) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    ix = np.clip(((gx - org[0]) / gres).astype(int), 0, esdf.shape[0] - 1)
    iy = np.clip(((gy - org[1]) / gres).astype(int), 0, esdf.shape[1] - 1)
    wall_d = esdf[ix, iy]
    dx = np.maximum(rects[:, 0] - gx[..., None], gx[..., None] - rects[:, 2])
    dy = np.maximum(rects[:, 1] - gy[..., None], gy[..., None] - rects[:, 3])
    box_d = (
        np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
        + np.minimum(np.maximum(dx, dy), 0.0)
    ).min(axis=-1)
    return np.minimum(wall_d, box_d).astype(np.float32)


def forward_policy(ckpt: dict, obs_batch: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    pol = ckpt["policy"]
    prep = ckpt["observation_preprocessor"]
    w0 = pol["net_container.0.weight"].float()
    b0 = pol["net_container.0.bias"].float()
    w2 = pol["net_container.2.weight"].float()
    b2 = pol["net_container.2.bias"].float()
    w4 = pol["net_container.4.weight"].float()
    b4 = pol["net_container.4.bias"].float()
    mean = prep["running_mean"].float()
    var = prep["running_variance"].float()

    norm = torch.clamp((obs_batch - mean) / (torch.sqrt(var) + 1e-8), -5.0, 5.0)
    h0 = F.elu(norm @ w0.T + b0)
    h1 = F.elu(h0 @ w2.T + b2)
    raw_act = torch.clamp(h1 @ w4.T + b4, -1.0, 1.0)
    safe_act = nvblox_shield(obs_batch, raw_act)
    return raw_act, safe_act


def main() -> None:
    pkg_default = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ckpt",
        type=Path,
        default=pkg_default / "models/best_human_nav.pt",
        help="Path to trained skrl checkpoint (.pt).",
    )
    parser.add_argument(
        "--arena_cache",
        type=Path,
        default=ARENA_CACHE,
        help="Path to map/arena/nav_arena_cache.npz.",
    )
    parser.add_argument(
        "--pkg_dir",
        type=Path,
        default=pkg_default,
        help="Target ROS 2 C++ package root (mk2_ws/src/xsting_nav).",
    )
    args = parser.parse_args()

    cfg_dir = args.pkg_dir / "config"
    mdl_dir = args.pkg_dir / "models"
    cfg_dir.mkdir(parents=True, exist_ok=True)
    mdl_dir.mkdir(parents=True, exist_ok=True)

    dst_ckpt = mdl_dir / "best_human_nav.pt"
    if args.ckpt.resolve() != dst_ckpt.resolve():
        shutil.copy2(args.ckpt, dst_ckpt)

    ws_src = args.pkg_dir.parent
    for cand in (
        ws_src / "xsting_description/config/joint_order.yaml",
        DEFAULT_SIM_ROOT / "xsting_description/config/joint_order.yaml",
    ):
        if cand.is_file():
            shutil.copy2(cand, cfg_dir / "joint_order_mk2.yaml")
            shutil.copy2(cand, cfg_dir / "joint_order_xsting.yaml")
            break

    ckpt = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    pol = ckpt["policy"]
    prep = ckpt["observation_preprocessor"]

    torch.manual_seed(20261114)
    esdf_dummy = torch.full((80, 80), 0.75, dtype=torch.float32)
    esdf_dummy[42:48, 38:43] = 0.02
    box_pts = torch.tensor(
        [[0.46, -0.08, 0.03], [0.46, 0.00, 0.03], [0.46, 0.08, 0.03], [0.47, 0.04, 0.03]],
        dtype=torch.float32,
    )
    puck_pts = torch.tensor(
        [[0.46, -0.04, 0.10], [0.46, 0.00, 0.10], [0.46, 0.04, 0.10]],
        dtype=torch.float32,
    )
    obs_case1 = build_obs(
        odom_pose=(0.25, 0.10, 0.15),
        odom_vel=(0.18, -0.03, 0.12),
        goal=GOALS[1],
        esdf_map=esdf_dummy,
        slice_info=(-1.5, -1.5, 0.05, 1000.0),
        box_pts=box_pts,
        puck_pts=puck_pts,
        init_pose=START,
        imu_state=(0.04, -0.02, 0.10, *quat_to_grav((0.998, 0.03, -0.04, 0.02))),
        nav_waypoint=(0.95, 0.95, 0.0),
    )
    rand_obs = torch.randn(5, 41, dtype=torch.float32) * 0.6
    rand_obs[2, 17:33] = 0.6
    rand_obs[2, 17 + 2 * 4 + 1] = 0.02
    rand_obs[2, 33] = 0.0
    rand_obs[3, 33] = 1.0
    rand_obs[3, 34] = 0.45
    rand_obs[3, 35] = 0.03
    rand_obs[3, 36] = 0.99
    rand_obs[3, 37] = 0.14

    all_obs = torch.cat([obs_case1, rand_obs], dim=0)
    raw_acts, safe_acts = forward_policy(ckpt, all_obs)

    gait = LegGait(freq=2.2, step_h=0.030)
    gait_steps = [gait.step((0.24, -0.06, 0.35), dt=0.01) for _ in range(5)]
    dock_sample = dock_vel((1.0, 0.44, -0.03, 0.98, 0.199), standoff=0.315)

    policy_export = {
        "format": "xsting_mk2_skrl_ppo_v1",
        "robot": "xsting_description",
        "source_checkpoint": args.ckpt.name,
        "source_sha256": sha256_of(args.ckpt),
        "obs_dim": 41,
        "act_dim": 3,
        "activation": "elu",
        "clip_observations": 5.0,
        "scaler_epsilon": 1e-8,
        "clip_actions": 1.0,
        "speed_limits_mps_radps": [0.35, 0.22, 1.80],
        "nvblox_cams_xyz_yaw": [[float(v) for v in c] for c in NVBLOX_CAMS],
        "lidar_2d_extrinsics": {
            "xyz_base_m": [float(v) for v in LIDAR_OFFSET_XYZ],
            "roll_rad": float(math.pi),
            "min_range_m": float(LIDAR_MIN_RANGE),
            "max_range_m": float(LIDAR_MAX_RANGE),
            "leg_mask_radius_m": float(LIDAR_LEG_MASK_RADIUS),
        },
        "ultrasonic_xyz_base_m": [float(v) for v in ULTRASONIC_OFFSET_XYZ],
        "joints_order": list(JOINTS),
        "arm_home_mk2_rad": [0.0, float(ARM_HOME_SHOULDER), float(ARM_HOME_ELBOW), 0.0, 0.0, 0.0],
        "stand_pose_rad": [float(v) for v in STAND_POSE],
        "leg_low_rad": [float(v) for v in LEG_LOW],
        "leg_high_rad": [float(v) for v in LEG_HIGH],
        "kinematics": {
            "mount_x": list(MOUNT_X),
            "mount_y": list(MOUNT_Y),
            "side": list(SIDE),
            "yaw0_rad": [float(v) for v in YAW0],
            "hip_z_m": float(HIP_Z),
            "hip0_rad": float(HIP0),
            "knee0_rad": float(KNEE0),
        },
        "scaler": {
            "running_mean": [float(x) for x in prep["running_mean"].tolist()],
            "running_variance": [float(x) for x in prep["running_variance"].tolist()],
        },
        "layers": [
            {
                "in_features": 41,
                "out_features": 128,
                "weight": [[float(v) for v in row] for row in pol["net_container.0.weight"].tolist()],
                "bias": [float(v) for v in pol["net_container.0.bias"].tolist()],
            },
            {
                "in_features": 128,
                "out_features": 128,
                "weight": [[float(v) for v in row] for row in pol["net_container.2.weight"].tolist()],
                "bias": [float(v) for v in pol["net_container.2.bias"].tolist()],
            },
            {
                "in_features": 128,
                "out_features": 3,
                "weight": [[float(v) for v in row] for row in pol["net_container.4.weight"].tolist()],
                "bias": [float(v) for v in pol["net_container.4.bias"].tolist()],
            },
        ],
        "golden_verification": {
            "obs_case1_inputs": {
                "odom_pose": [0.25, 0.10, 0.15],
                "odom_vel": [0.18, -0.03, 0.12],
                "goal": list(GOALS[1]),
                "slice_info": [-1.5, -1.5, 0.05, 1000.0],
                "init_pose": list(START),
                "quat_wxyz": [0.998, 0.03, -0.04, 0.02],
                "gyro_xyz": [0.04, -0.02, 0.10],
                "nav_waypoint": [0.95, 0.95, 0.0],
                "box_pts": box_pts.tolist(),
                "puck_pts": puck_pts.tolist(),
            },
            "obs_batch": [[float(v) for v in row] for row in all_obs.tolist()],
            "raw_actions": [[float(v) for v in row] for row in raw_acts.tolist()],
            "shield_actions": [[float(v) for v in row] for row in safe_acts.tolist()],
            "gait_step5_joints": [float(v) for v in gait_steps[-1]],
            "dock_vel_sample": [
                float(dock_sample[0]),
                float(dock_sample[1]),
                float(dock_sample[2]),
                1.0 if dock_sample[3] else 0.0,
            ],
        },
    }

    policy_json_path = cfg_dir / "mk2_nav_policy.json"
    policy_json_path.write_text(json.dumps(policy_export, indent=2), encoding="utf-8")

    # Align HOME (goals[0] & start_pose) with xsting_mission/config/arena.yaml: [0.734, 0.950, pi]
    home_pose = [0.734, 0.950, float(math.pi)]
    exported_goals = [home_pose] + [[float(x), float(y), float(yaw)] for x, y, yaw in GOALS[1:]]

    static_ce = compute_static_esdf(args.arena_cache, nx=55, ny=79, res=0.08)
    arena_export = {
        "format": "xsting_arena_static_esdf_v1",
        "arena_size_m": [4.404, 6.304],
        "nx": 55,
        "ny": 79,
        "res_m": 0.08,
        "start_pose": home_pose,
        "grid_x_body_m": [float(v) for v in GRID_X],
        "grid_y_body_m": [float(v) for v in GRID_Y],
        "goals": exported_goals,
        "boxes": [[float(v) for v in b] for b in BOXES],
        "cell_esdf_row_major": [round(float(v), 5) for v in static_ce.reshape(-1).tolist()],
    }
    arena_json_path = cfg_dir / "arena_static_esdf.json"
    arena_json_path.write_text(json.dumps(arena_export, indent=2), encoding="utf-8")

    print(f"[OK] Exported policy weights -> {policy_json_path}")
    print(f"[OK] Exported static ESDF    -> {arena_json_path}")
    print(f"[OK] Bundled .pt backup      -> {dst_ckpt}")


if __name__ == "__main__":
    main()
