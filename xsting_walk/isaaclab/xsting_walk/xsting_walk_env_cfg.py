"""Configuration for the direct-workflow xSting fabric-carpet walking environment."""

from __future__ import annotations

import math
from pathlib import Path

import gymnasium as gym
import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ImuCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.visualizers import VisualizerCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg
import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[1]
_CANDIDATE_USD = ROOT_DIR / "xsting_description" / "usd" / "xsting_training.usda"
XSTING_USD = (
    _CANDIDATE_USD
    if _CANDIDATE_USD.is_file()
    else ROOT_DIR.parents[1] / "xsting_description" / "usd" / "xsting_training.usda"
)
LEG_JOINTS = [f"leg{i}_{j}" for i in range(1, 7) for j in ("yaw", "hip", "knee")]
FOOT_BODIES = [f"leg{i}_knee" for i in range(1, 7)]


@configclass
class XStingWalkSceneCfg(InteractiveSceneCfg):
    """Fabric carpet ground, xsting_description robot, and body IMU sensor."""

    # Fabric carpet ground plane with high pile friction and zero bounce.
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.GroundPlaneCfg(
            color=(0.28, 0.30, 0.38),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.05,
                dynamic_friction=0.85,
                restitution=0.0,
                friction_combine_mode="multiply",
            ),
        ),
    )

    # xSting hexapod from xsting_description (finger_r is passive mimic).
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(XSTING_USD),
            articulation_props=PhysxArticulationCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.042),
            joint_pos={
                "leg[14]_yaw": math.radians(30.0),
                "leg[25]_yaw": 0.0,
                "leg[36]_yaw": math.radians(-30.0),
                "leg[1-6]_hip": math.radians(15.0),
                "leg[1-6]_knee": math.radians(-90.0),
                "arm_yaw": 0.0,
                "arm_shoulder": -1.2962634,
                "arm_elbow": 1.2962634,
                "arm_wrist": 0.0,
                "arm_roll": 0.0,
                "finger_l": 0.0,
                "finger_r": 0.0,
            },
        ),
        soft_joint_pos_limit_factor=0.95,
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=["leg[1-6]_(yaw|hip|knee)"],
                stiffness=45.0,
                damping=1.2,
                armature=0.01,
                joint_effort_limit=3.8,
                joint_velocity_limit=4.5,
            ),
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["arm_.*"],
                stiffness=80.0,
                damping=4.0,
                armature=0.01,
                joint_effort_limit=3.8,
                joint_velocity_limit=4.5,
            ),
            "finger": ImplicitActuatorCfg(
                joint_names_expr=["finger_l"],
                stiffness=2000.0,
                damping=20.0,
                armature=0.001,
                joint_effort_limit=10.0,
                joint_velocity_limit=0.05,
            ),
        },
    )

    # PhysX IMU sensor attached to xsting_description's /xsting/base/imu frame.
    imu: ImuCfg = ImuCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base/imu",
        update_period=0.02,
    )

    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=1800.0, color=(0.85, 0.85, 0.85)),
    )


@configclass
class EventsCfg:
    """Randomize foot-to-carpet friction across parallel environments."""

    carpet_friction = EventTermCfg(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_BODIES),
            "static_friction_range": (0.80, 1.30),
            "dynamic_friction_range": (0.60, 1.10),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 32,
            "make_consistent": True,
        },
    )


@configclass
class XStingWalkEnvCfg(DirectRLEnvCfg):
    """Cartpole-style DirectRLEnvCfg for high-speed 8-dir walking + left/right rotation."""

    # 200 Hz PhysX, 50 Hz policy & IMU (step_dt = 0.02 s).
    seed = 42
    decimation = 4
    episode_length_s = 15.0
    action_scale = 0.28
    action_ema = 0.55
    action_space = gym.spaces.Box(-1.0, 1.0, (18,), dtype=np.float32)
    observation_space = 74
    state_space = 0

    sim: SimulationCfg = SimulationCfg(
        dt=0.005,
        render_interval=decimation,
        physics=PhysxCfg(
            gpu_max_rigid_contact_count=2**19,
            gpu_max_rigid_patch_count=2**17,
            gpu_found_lost_pairs_capacity=2**18,
            gpu_found_lost_aggregate_pairs_capacity=2**20,
            gpu_total_aggregate_pairs_capacity=2**18,
            gpu_collision_stack_size=2**24,
            gpu_heap_capacity=2**24,
        ),
    )
    scene: XStingWalkSceneCfg = XStingWalkSceneCfg(
        num_envs=512, env_spacing=2.2, replicate_physics=True, clone_in_fabric=False
    )
    events: EventsCfg = EventsCfg()

    # High-speed omnidirectional + yaw-rotation tripod CPG + RL residual parameters.
    gait_freq = 3.5
    stride_gain_xy = (1.08, 1.24)
    stride_gain_yaw = 1.04
    swing_clearance = 0.032
    stance_press = 0.003
    cmd_resample_s = 5.0
    max_speed = (0.38, 0.32, 1.40)
    fixed_command: tuple[float, float, float] | None = None
    min_base_height = 0.012
    min_upright = 0.65

    def __post_init__(self) -> None:
        super().__post_init__()
        self.sim.default_visualizer_cfg = VisualizerCfg(eye=(3.0, 3.0, 1.8), lookat=(0.0, 0.0, 0.05))
