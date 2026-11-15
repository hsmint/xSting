"""Direct-workflow omnidirectional fabric-carpet walking environment for xSting."""

from __future__ import annotations

from collections.abc import Sequence
import math

from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_apply, quat_apply_inverse, sample_uniform
import torch

from .xsting_walk_env_cfg import FOOT_BODIES, LEG_JOINTS, XStingWalkEnvCfg

# 15 locomotion modes:
#   0: stop, 1..8: 8 planar directions, 9..10: pure left/right rotation, 11..14: curved turns.
D = 0.70710678
MODES = (
    (0.0, 0.0, 0.0),
    (1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, -1.0, 0.0),
    (D, D, 0.0),
    (D, -D, 0.0),
    (-D, D, 0.0),
    (-D, -D, 0.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, -1.0),
    (0.75, 0.0, 0.75),
    (0.75, 0.0, -0.75),
    (-0.75, 0.0, 0.75),
    (-0.75, 0.0, -0.75),
)
MODE_PROBS = (0.06, *(0.07,) * 8, 0.09, 0.09, *(0.05,) * 4)


class XStingWalkEnv(DirectRLEnv):
    """Hexapod high-speed omnidirectional + yaw-rotation locomotion environment on fabric carpet."""

    cfg: XStingWalkEnvCfg

    def __init__(self, cfg: XStingWalkEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self.robot = self.scene["robot"]
        self.imu = self.scene["imu"]
        leg_ids, _ = self.robot.find_joints(LEG_JOINTS, preserve_order=True)
        foot_ids, _ = self.robot.find_bodies(FOOT_BODIES, preserve_order=True)
        self.leg_ids = torch.tensor(leg_ids, device=self.device)
        self.foot_ids = torch.tensor(foot_ids, device=self.device)

        # Distal foot-tip offsets in leg{1..6}_knee frames (from xsting_description STL).
        tips = [[0.0, 0.13275, -0.02]] * 3 + [[0.0, -0.13275, -0.02]] * 3
        self.foot_tips = torch.tensor(tips, device=self.device).expand(self.num_envs, -1, -1)
        self.tripod_offsets = torch.tensor([0.0, 0.5, 0.0, 0.5, 0.0, 0.5], device=self.device)

        self.default_pos = self.robot.data.default_joint_pos.torch.clone()
        self.targets = self.default_pos.clone()
        self.raw_actions = torch.zeros(self.num_envs, 18, device=self.device)
        self.prev_raw_actions = torch.zeros_like(self.raw_actions)
        self.actions = torch.zeros_like(self.raw_actions)

        self.modes = torch.tensor(MODES, dtype=torch.float32, device=self.device)
        self.mode_probs = torch.tensor(MODE_PROBS, dtype=torch.float32, device=self.device)
        self.max_speed = torch.tensor(cfg.max_speed, dtype=torch.float32, device=self.device)
        self.grav_bias = torch.tensor((0.0, 0.0, 9.81), device=self.device)
        self.cmd = torch.zeros(self.num_envs, 3, device=self.device)
        self.cmd_scale = torch.tensor((3.0, 3.0, 0.8), device=self.device)
        self.cmd_step = torch.zeros_like(self.episode_length_buf)
        self.cmd_cycle = torch.zeros_like(self.episode_length_buf)
        self.phase = torch.zeros(self.num_envs, device=self.device)
        self.heading_ref = torch.zeros(self.num_envs, device=self.device)
        self.vel_int = torch.zeros(self.num_envs, 2, device=self.device)

        self.jac_inv, self.foot_home_xy = self._compute_foot_jacobian_inv()

    def _compute_foot_jacobian_inv(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Compute exact (6, 3, 3) inverse foot Jacobian and (6, 2) home XY in base frame."""
        data = self.robot.data
        env0 = torch.zeros(1, dtype=torch.long, device=self.device)
        pose0 = data.default_root_pose.torch[env0].clone()
        pose0[:, :3] += self.scene.env_origins[env0]
        q0 = self.default_pos[env0].clone()
        self.robot.write_root_pose_to_sim_index(root_pose=pose0, env_ids=env0)
        self.robot.write_joint_position_to_sim_index(position=q0, env_ids=env0)
        self.sim.forward()

        def foot_pos_b() -> torch.Tensor:
            off = quat_apply(data.body_link_quat_w.torch[0, self.foot_ids], self.foot_tips[0])
            rel_w = data.body_link_pos_w.torch[0, self.foot_ids] + off - data.root_link_pos_w.torch[0]
            return quat_apply_inverse(data.root_link_quat_w.torch[0:1].expand(6, -1), rel_w)

        f0 = foot_pos_b()
        jac = torch.zeros(6, 3, 3, device=self.device)
        eps = 0.02
        for leg in range(6):
            for j in range(3):
                q = q0.clone()
                q[0, self.leg_ids[3 * leg + j]] += eps
                self.robot.write_joint_position_to_sim_index(position=q, env_ids=env0)
                self.sim.forward()
                jac[leg, :, j] = (foot_pos_b()[leg] - f0[leg]) / eps

        self.robot.write_joint_position_to_sim_index(position=q0, env_ids=env0)
        self.sim.forward()
        return torch.linalg.inv(jac), f0[:, :2].clone()

    def _moving_mask(self) -> torch.Tensor:
        return (self.cmd[:, :2].norm(dim=-1) > 0.01) | (self.cmd[:, 2].abs() > 0.02)

    def _root_yaw(self) -> torch.Tensor:
        q = self.robot.data.root_link_quat_w.torch
        return torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))

    def _heading_error(self) -> torch.Tensor:
        diff = self._root_yaw() - self.heading_ref
        return torch.atan2(torch.sin(diff), torch.cos(diff))

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        moving_mask = self._moving_mask()
        moving = moving_mask.float().unsqueeze(-1)

        self.prev_raw_actions.copy_(self.raw_actions)
        self.raw_actions.copy_(actions.clamp(-1.0, 1.0) * moving)
        alpha = self.cfg.action_ema
        self.actions.mul_(1.0 - alpha).add_(alpha * self.raw_actions).mul_(moving)

        self.phase = torch.where(
            moving_mask, (self.phase + self.cfg.gait_freq * self.step_dt) % 1.0, torch.zeros_like(self.phase)
        )

        # Constant-velocity stance + C1 velocity-matched Hermite swing + zero-impact vertical lift.
        leg_phase = (self.phase.unsqueeze(-1) + self.tripod_offsets) % 1.0
        u = 2.0 * leg_phase
        s_swing = -1.0 - 2.0 * u + 12.0 * u * u - 8.0 * u * u * u
        s_prof = torch.where(leg_phase < 0.5, s_swing, 3.0 - 4.0 * leg_phase)
        z_swing = torch.square(torch.sin(math.pi * torch.clamp(u, 0.0, 1.0)))
        z_stance = torch.square(torch.sin(math.pi * torch.clamp(u - 1.0, 0.0, 1.0)))

        gx, gy = self.cfg.stride_gain_xy
        gyaw = self.cfg.stride_gain_yaw
        f_vx = gx * self.cmd[:, 0:1] - gyaw * self.cmd[:, 2:3] * self.foot_home_xy[:, 1]
        f_vy = gy * self.cmd[:, 1:2] + gyaw * self.cmd[:, 2:3] * self.foot_home_xy[:, 0]
        half_stride = torch.stack((f_vx, f_vy), dim=-1) / (4.0 * self.cfg.gait_freq)
        df_xy = s_prof.unsqueeze(-1) * half_stride
        df_z = (self.cfg.swing_clearance * z_swing - self.cfg.stance_press * z_stance).unsqueeze(-1)
        df = torch.cat((df_xy, df_z), dim=-1)
        cpg_dq = torch.einsum("lij,nlj->nli", self.jac_inv, df).reshape(self.num_envs, 18)

        limits = self.robot.data.soft_joint_pos_limits.torch
        self.targets.copy_(self.default_pos)
        self.targets[:, self.leg_ids] += moving * (cpg_dq + self.cfg.action_scale * self.actions)
        self.targets.clamp_(limits[..., 0], limits[..., 1])

    def _apply_action(self) -> None:
        self.robot.set_joint_position_target_index(target=self.targets)

    def _get_observations(self) -> dict[str, torch.Tensor]:
        resample_steps = max(1, round(self.cfg.cmd_resample_s / self.step_dt))
        due = (self.episode_length_buf - self.cmd_step >= resample_steps).nonzero().flatten()
        if due.numel() > 0:
            self._sample_commands(due)

        data = self.robot.data
        moving = self._moving_mask().float()
        ang = 2.0 * math.pi * self.phase
        clock = torch.stack((torch.sin(ang) * moving, torch.cos(ang) * moving), dim=-1)
        h_err = self._heading_error().unsqueeze(-1)
        vel_err_xy = data.root_link_lin_vel_b.torch[:, :2] - self.cmd[:, :2]
        yaw_err = self.imu.data.ang_vel_b.torch[:, 2:3] - self.cmd[:, 2:3]

        obs = torch.cat(
            (
                self.imu.data.ang_vel_b.torch * 0.25,
                (self.imu.data.lin_acc_b.torch - self.grav_bias) * 0.10,
                data.projected_gravity_b.torch,
                self.cmd * self.cmd_scale,
                clock,
                vel_err_xy * 4.0,
                self.vel_int * 20.0,
                yaw_err * 1.5,
                h_err * 4.0,
                (data.joint_pos.torch - self.default_pos)[:, self.leg_ids],
                data.joint_vel.torch[:, self.leg_ids] * 0.05,
                self.actions,
            ),
            dim=-1,
        )
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        data = self.robot.data
        yaw = self._root_yaw()
        self.heading_ref = self.heading_ref + self.cmd[:, 2] * self.step_dt
        h_err = torch.atan2(torch.sin(yaw - self.heading_ref), torch.cos(yaw - self.heading_ref)).clamp(-0.35, 0.35)
        self.heading_ref = yaw - h_err
        self.vel_int = (
            self.vel_int + (data.root_link_lin_vel_b.torch[:, :2] - self.cmd[:, :2]) * self.step_dt
        ).clamp(-0.06, 0.06)

        off = quat_apply(data.body_link_quat_w.torch[:, self.foot_ids], self.foot_tips)
        foot_pos = data.body_link_pos_w.torch[:, self.foot_ids] + off
        foot_vel = data.body_link_lin_vel_w.torch[:, self.foot_ids] + torch.cross(
            data.body_link_ang_vel_w.torch[:, self.foot_ids], off, dim=-1
        )
        foot_z = foot_pos[..., 2] - self.scene.env_origins[:, 2:3]
        leg_phase = (self.phase.unsqueeze(-1) + self.tripod_offsets) % 1.0

        rew, lin_err, yaw_err, scuff = compute_rewards(
            data.root_link_lin_vel_b.torch,
            self.imu.data.ang_vel_b.torch,
            self.imu.data.lin_acc_b.torch,
            data.projected_gravity_b.torch,
            h_err,
            self.vel_int,
            self.cmd,
            leg_phase,
            foot_z,
            foot_vel[..., :2],
            data.applied_torque.torch[:, self.leg_ids],
            self.raw_actions,
            self.prev_raw_actions,
            self.actions,
            self.reset_terminated,
            self.cfg.swing_clearance,
            self.step_dt,
        )
        self.extras.setdefault("log", {}).update(
            {
                "Metrics/vel_xy_err": lin_err.sqrt().mean(),
                "Metrics/yaw_err": yaw_err.sqrt().mean(),
                "Metrics/speed_xy": data.root_link_lin_vel_b.torch[:, :2].norm(dim=-1).mean(),
                "Metrics/carpet_scuff": scuff.mean(),
            }
        )
        return rew

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        data = self.robot.data
        height = data.root_link_pos_w.torch[:, 2] - self.scene.env_origins[:, 2]
        upright = -data.projected_gravity_b.torch[:, 2]
        fallen = (height < self.cfg.min_base_height) | (upright < self.cfg.min_upright)
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return fallen, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = torch.arange(self.num_envs, device=self.device)
            full_reset = True
        else:
            env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
            full_reset = False

        self.extras.setdefault("log", {})["Metrics/survival_rate"] = self.reset_time_outs[env_ids].float().mean()
        super()._reset_idx(env_ids)
        if full_reset and self.cfg.fixed_command is None and self.num_envs > 32:
            self.episode_length_buf[:] = torch.randint_like(self.episode_length_buf, high=int(self.max_episode_length))

        data = self.robot.data
        root_pose = data.default_root_pose.torch[env_ids].clone()
        root_pose[:, :3] += self.scene.env_origins[env_ids]
        joint_pos = self.default_pos[env_ids].clone()
        joint_vel = data.default_joint_vel.torch[env_ids].clone()

        self.robot.write_root_pose_to_sim_index(root_pose=root_pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(
            root_velocity=data.default_root_vel.torch[env_ids], env_ids=env_ids
        )
        self.robot.write_joint_position_to_sim_index(position=joint_pos, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=joint_vel, env_ids=env_ids)

        self.targets[env_ids] = joint_pos
        self.raw_actions[env_ids] = 0.0
        self.prev_raw_actions[env_ids] = 0.0
        self.actions[env_ids] = 0.0
        self.phase[env_ids] = 0.0
        self.cmd_cycle[env_ids] = 0
        self.heading_ref[env_ids] = 0.0
        self.vel_int[env_ids] = 0.0
        self._sample_commands(env_ids)

    def _sample_commands(self, ids: torch.Tensor) -> None:
        if self.cfg.fixed_command is not None:
            self.cmd[ids] = torch.tensor(self.cfg.fixed_command, dtype=torch.float32, device=self.device)
        elif self.num_envs <= 32:
            # Playback mode: cycle 8 high-speed planar directions + left/right turns (modes 1..10).
            mode = ((ids + self.cmd_cycle[ids]) % 10) + 1
            self.cmd[ids] = self.modes[mode] * (0.92 * self.max_speed)
            self.cmd_cycle[ids] += 1
        else:
            n = ids.numel()
            mode = torch.multinomial(self.mode_probs, n, replacement=True)
            spd = sample_uniform(0.55, 1.0, (n, 1), self.device)
            self.cmd[ids] = self.modes[mode] * spd * self.max_speed
        self.heading_ref[ids] = self._root_yaw()[ids]
        self.vel_int[ids] = 0.0
        self.cmd_step[ids] = self.episode_length_buf[ids]


@torch.jit.script
def compute_rewards(
    lin_vel_b: torch.Tensor,
    imu_ang_vel_b: torch.Tensor,
    imu_lin_acc_b: torch.Tensor,
    proj_grav_b: torch.Tensor,
    heading_err: torch.Tensor,
    vel_int: torch.Tensor,
    cmd: torch.Tensor,
    leg_phase: torch.Tensor,
    foot_z: torch.Tensor,
    foot_vel_xy: torch.Tensor,
    torques: torch.Tensor,
    raw_actions: torch.Tensor,
    prev_raw_actions: torch.Tensor,
    ema_actions: torch.Tensor,
    terminated: torch.Tensor,
    swing_clearance: float,
    step_dt: float,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    cmd_spd = torch.norm(cmd[:, :2], dim=-1)
    lin_moving = (cmd_spd > 0.01).float()
    turning = torch.abs(cmd[:, 2]) >= 0.02
    moving = ((cmd_spd > 0.01) | turning).float()
    cmd_dir = cmd[:, :2] / torch.clamp(cmd_spd.unsqueeze(-1), min=1e-4)

    # Multi-scale planar velocity + integral position-lag tracking (zero steady-state XY error).
    lin_err = torch.sum(torch.square(lin_vel_b[:, :2] - cmd[:, :2]), dim=-1)
    lin_l1 = torch.sqrt(lin_err + 1e-6) - 1e-3
    pos_lag_sq = torch.sum(torch.square(vel_int), dim=-1)
    pos_lag_l1 = torch.sqrt(pos_lag_sq + 1e-6) - 1e-3
    vel_tol_sq = 0.0025 + 0.035 * torch.square(cmd_spd)
    along = torch.sum(lin_vel_b[:, :2] * cmd_dir, dim=-1)
    orth_sq = torch.clamp(torch.sum(torch.square(lin_vel_b[:, :2]), dim=-1) - torch.square(along), min=0.0)
    rew_vel = (
        3.0 * torch.exp(-lin_err / vel_tol_sq)
        + 2.5 * torch.exp(-lin_err / 0.0020)
        + 2.5 * torch.exp(-lin_err / 0.0003)
        + 1.5 * torch.exp(-pos_lag_sq / 0.0004)
    )

    # Multi-scale yaw-rate + unified integral heading lock across ALL modes (zero steady-state WZ error).
    yaw_rate_err = torch.square(imu_ang_vel_b[:, 2] - cmd[:, 2])
    yaw_l1 = torch.sqrt(yaw_rate_err + 1e-6) - 1e-3
    yaw_tol_sq = 0.025 + 0.035 * torch.square(cmd[:, 2])
    head_sq = torch.square(heading_err)
    head_l1 = torch.abs(heading_err)
    rew_yaw = (
        2.5 * torch.exp(-yaw_rate_err / yaw_tol_sq)
        + 2.0 * torch.exp(-yaw_rate_err / 0.012)
        + 1.5 * torch.exp(-yaw_rate_err / 0.0015)
        + 2.0 * torch.exp(-head_sq / 0.025)
    )
    tilt = torch.sum(torch.square(proj_grav_b[:, :2]), dim=-1)
    imu_rp_rate = torch.sum(torch.square(imu_ang_vel_b[:, :2]), dim=-1)
    imu_z_acc = torch.square(imu_lin_acc_b[:, 2] - 9.81)

    # Per-leg feather-touchdown swing clearance & fabric carpet anti-scuffing.
    u = 2.0 * leg_phase
    swing_wave = torch.square(torch.sin(math.pi * torch.clamp(u, 0.0, 1.0)))
    stance_mask = torch.where(
        moving.unsqueeze(-1) > 0.5, (leg_phase >= 0.52).float() * (leg_phase <= 0.98).float(), torch.ones_like(leg_phase)
    )

    target_z = 0.0015 + swing_clearance * swing_wave * moving.unsqueeze(-1)
    rew_swing = 1.5 * moving * torch.mean(torch.exp(-torch.square(foot_z - target_z) / 0.00018), dim=-1)

    foot_spd_sq = torch.sum(torch.square(foot_vel_xy), dim=-1)
    stance_slip = torch.sum(stance_mask * foot_spd_sq, dim=-1)
    low_drag = torch.sum((1.0 - stance_mask) * torch.clamp(1.0 - foot_z / 0.010, 0.0, 1.0) * foot_spd_sq, dim=-1)
    scuff = stance_slip + low_drag

    smooth = torch.sum(torch.square(raw_actions - prev_raw_actions), dim=-1)
    res_mag = torch.sum(torch.square(ema_actions), dim=-1)
    effort = torch.sum(torch.square(torques), dim=-1)

    total = (
        rew_vel
        + rew_yaw
        + rew_swing
        - 8.0 * lin_err
        - 2.5 * lin_l1
        - 20.0 * pos_lag_sq
        - 2.0 * pos_lag_l1
        - 5.0 * lin_moving * orth_sq
        - 2.5 * yaw_rate_err
        - 1.2 * yaw_l1
        - 2.5 * head_sq
        - 0.8 * head_l1
        - 3.0 * tilt
        - 1.5 * torch.square(lin_vel_b[:, 2])
        - 0.08 * imu_rp_rate
        - 0.002 * imu_z_acc
        - 1.0 * scuff
        - 0.20 * smooth
        - 0.008 * res_mag
        - 0.0002 * effort
        - 2.0 * terminated.float()
    )
    return total * step_dt, lin_err, yaw_rate_err, scuff
