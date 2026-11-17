#!/usr/bin/env python3
"""skrl PPO 체크포인트(best_agent.pt)를 xsting_walk C++ 이진 가중치(XSTMLP01)로 변환한다."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import struct

import numpy as np
import torch


DEFAULT_LEG_POS = [
    math.radians(30.0) if leg in (1, 4) else (0.0 if leg in (2, 5) else math.radians(-30.0))
    if j == "yaw"
    else (math.radians(15.0) if j == "hip" else math.radians(-90.0))
    for leg in range(1, 7)
    for j in ("yaw", "hip", "knee")
]


def export_checkpoint(ckpt_path: Path, out_path: Path, clip_obs: float = 5.0) -> None:
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    policy = ckpt["policy"]
    preproc = ckpt["observation_preprocessor"]

    running_mean = preproc["running_mean"].detach().cpu().to(torch.float32).numpy()
    running_var = preproc["running_variance"].detach().cpu().to(torch.float32).numpy()
    # skrl RunningStandardScaler: (x - mean.float()) / (sqrt(var.float()) + 1e-8)
    std = np.sqrt(running_var).astype(np.float32) + np.float32(1e-8)

    in_dim = int(running_mean.shape[0])
    if in_dim != 74:
        raise ValueError(f"예상 관측 차원(74)과 다릅니다: {in_dim}")

    layer_keys = [
        ("net_container.0.weight", "net_container.0.bias"),
        ("net_container.2.weight", "net_container.2.bias"),
        ("net_container.4.weight", "net_container.4.bias"),
        ("policy_layer.weight", "policy_layer.bias"),
    ]
    layers: list[tuple[np.ndarray, np.ndarray]] = []
    prev = in_dim
    for wk, bk in layer_keys:
        w = policy[wk].detach().cpu().to(torch.float32).contiguous().numpy()
        b = policy[bk].detach().cpu().to(torch.float32).contiguous().numpy()
        if w.ndim != 2 or b.ndim != 1 or w.shape[0] != b.shape[0] or w.shape[1] != prev:
            raise ValueError(f"레이어 차원 불일치 ({wk}): {w.shape}, {b.shape}")
        layers.append((w, b))
        prev = int(w.shape[0])

    if prev != 18:
        raise ValueError(f"예상 행동 차원(18)과 다릅니다: {prev}")

    extras = [np.asarray(DEFAULT_LEG_POS, dtype=np.float32)]

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("wb") as f:
        f.write(b"XSTMLP01")
        f.write(struct.pack("<IIIf", in_dim, len(layers), len(extras), float(clip_obs)))
        f.write(running_mean.astype("<f4").tobytes())
        f.write(std.astype("<f4").tobytes())
        for w, b in layers:
            rows, cols = w.shape
            f.write(struct.pack("<II", rows, cols))
            f.write(w.astype("<f4").tobytes(order="C"))
            f.write(b.astype("<f4").tobytes(order="C"))
        for vec in extras:
            f.write(struct.pack("<I", int(vec.shape[0])))
            f.write(vec.astype("<f4").tobytes(order="C"))

    # PyTorch 기준 순전파 검증 벡터 생성
    rng = np.random.default_rng(42)
    test_obs = torch.tensor(rng.uniform(-0.4, 0.4, size=(74,)), dtype=torch.float32)
    test_obs[8] = -0.998  # projected_gravity_b.z
    with torch.no_grad():
        x = torch.clamp(
            (test_obs - preproc["running_mean"].float())
            / (torch.sqrt(preproc["running_variance"].float()) + 1e-8),
            min=-clip_obs,
            max=clip_obs,
        )
        for idx, (wk, bk) in enumerate(layer_keys):
            x = policy[wk].float() @ x + policy[bk].float()
            if idx + 1 < len(layer_keys):
                x = torch.nn.functional.elu(x)
            else:
                x = torch.clamp(x, -1.0, 1.0)

    print(f"Exported: {out_path} ({out_path.stat().st_size} bytes)")
    print("Golden obs[:6]:", [round(float(v), 7) for v in test_obs[:6]])
    print("Golden act[:6]:", [round(float(v), 7) for v in x[:6]])


def main() -> None:
    pkg_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=pkg_dir / "models" / "best_agent.pt",
        help="입력 skrl .pt 체크포인트 경로",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=pkg_dir / "models" / "walk_policy_74d.bin",
        help="출력 C++ 이진 가중치(.bin) 경로",
    )
    args = parser.parse_args()
    export_checkpoint(args.checkpoint, args.output)


if __name__ == "__main__":
    main()
