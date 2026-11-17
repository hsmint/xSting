"""Play or evaluate a trained xSting fabric-carpet walking policy with skrl."""

from pathlib import Path
import sys

import warp as wp

wp.config.enable_backward = False
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from isaaclab_rl.entrypoints import run_play_cli  # noqa: E402
import xsting_walk  # noqa: E402,F401

NAMED_DIRS = {
    "stop": (0.0, 0.0, 0.0),
    "forward": (0.35, 0.0, 0.0),
    "backward": (-0.35, 0.0, 0.0),
    "left": (0.0, 0.30, 0.0),
    "right": (0.0, -0.30, 0.0),
    "diag_fl": (0.25, 0.23, 0.0),
    "diag_fr": (0.25, -0.23, 0.0),
    "diag_rl": (-0.25, 0.23, 0.0),
    "diag_rr": (-0.25, -0.23, 0.0),
    "turn_left": (0.0, 0.0, 1.25),
    "turn_right": (0.0, 0.0, -1.25),
    "curve_left": (0.28, 0.0, 0.95),
    "curve_right": (0.28, 0.0, -0.95),
}


def main() -> int:
    raw = sys.argv[1:]
    user_args: list[str] = []
    i = 0
    while i < len(raw):
        if raw[i] == "--dir" and i + 1 < len(raw) and raw[i + 1] in NAMED_DIRS:
            vx, vy, wz = NAMED_DIRS[raw[i + 1]]
            user_args.append(f"env.fixed_command=[{vx},{vy},{wz}]")
            i += 2
        else:
            user_args.append(raw[i])
            i += 1

    has_ckpt = any(a == "--checkpoint" or a.startswith("--checkpoint=") for a in user_args)
    default_ckpt = Path(__file__).resolve().parents[1] / "checkpoints" / "best_agent.pt"
    has_viz = any(a in ("--viz", "--visualizer") or a.startswith(("--viz=", "--visualizer=")) for a in user_args)
    args = ["--task", "Isaac-XSting-CarpetWalk-Direct-v0", "--rl_library", "skrl"]
    if not has_ckpt and default_ckpt.is_file():
        args.extend(["--checkpoint", str(default_ckpt)])
    if not has_viz and "--video" not in user_args:
        args.extend(["--viz", "kit", "--real-time"])
    return run_play_cli([*args, *user_args])


if __name__ == "__main__":
    raise SystemExit(main())
