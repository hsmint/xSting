"""Train the xSting fabric-carpet walking policy with skrl PPO."""

from pathlib import Path
import sys

import warp as wp

wp.config.enable_backward = False
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from isaaclab_rl.entrypoints import run_train_cli  # noqa: E402
import xsting_walk  # noqa: E402,F401


def main() -> int:
    args = ["--task", "Isaac-XSting-CarpetWalk-Direct-v0", "--rl_library", "skrl"]
    return run_train_cli([*args, *sys.argv[1:]])


if __name__ == "__main__":
    raise SystemExit(main())
