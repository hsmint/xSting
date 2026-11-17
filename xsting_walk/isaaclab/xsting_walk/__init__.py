"""Omnidirectional fabric-carpet walking task for xSting (Cartpole-style layout)."""

import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-XSting-CarpetWalk-Direct-v0",
    entry_point=f"{__name__}.xsting_walk_env:XStingWalkEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.xsting_walk_env_cfg:XStingWalkEnvCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
        "default_agent": "skrl",
    },
)
