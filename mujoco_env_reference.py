"""
Humanoid Locomotion Validation Environment (Pure MuJoCo)
For validating IsaacSim-trained policies on CPU MuJoCo.

Policy order (interleaved, what policy observes/outputs):
  [left_hip1, right_hip1, left_hip2, right_hip2, left_thigh, right_thigh,
   left_knee, right_knee, left_ankle, right_ankle]

MuJoCo actuator order (left block then right block):
  [left_hip1, left_hip2, left_thigh, left_knee, left_ankle,
   right_hip1, right_hip2, right_thigh, right_knee, right_ankle]
"""

import math
import numpy as np
import mujoco


class HumanoidLocomotionEnv:
    """Pure MuJoCo locomotion validation environment."""

    def __init__(
        self,
        xml_path: str,
        frame_stack: int = 3,
        stack_frame_major: bool = False,
        use_phase_obs: bool = True,
        gait_period_s: float = 1.0,
        # IsaacLab-aligned phase behavior.
        freeze_phase_when_standing: bool = True,
        stand_phase_lin_threshold: float = 1e-3,
        stand_phase_yaw_threshold: float = 1e-3,
        stand_phase_value: float = 0.0,  # radians
        # Legacy aliases (optional for backward compatibility).
        phase_gate_when_standing: bool | None = None,
        phase_freeze_when_standing: bool | None = None,
        phase_cmd_lin_eps: float | None = None,
        phase_cmd_yaw_eps: float | None = None,
        phase_standing_clock: tuple[float, float] | None = None,
        include_height: bool = False,
        disturbance_force_max: float = 5.0,
        disturbance_torque_max: float = 2.0,
        disturbance_prob: float = 0.00,
        action_scale: float = 1.0,
        action_scale_by_joint={
            # "left_hip1_joint": 0.50,  "right_hip1_joint": 0.50,
            # "left_hip2_joint": 0.30,  "right_hip2_joint": 0.30,
            "left_thigh_joint": 0.30, "right_thigh_joint": 0.30,
            # "left_knee_joint": 0.45,  "right_knee_joint": 0.45,
            # "left_ankle_joint": 0.35, "right_ankle_joint": 0.35,
        },
        action_noise_std: float = 0.0,
        action_smoothing_alpha: float = 1.0,      # 1.0 = no EMA smoothing
        action_delta_max: float | None = None,    # max change per control step in normalized action units
        act_max_latency: int = 5,
        act_latency_steps: int = 0,
        obs_max_latency: int = 0,
        obs_latency_steps: int = 3,
        imu_bias_gravity: tuple[float, float, float] = (0.0, 0.0, 0.0),
        imu_bias_gyro: tuple[float, float, float] = (0.0, 0.0, 0.0),
        imu_mount_axis: tuple[float, float, float] = (1.0, 0.0, 0.0),
        imu_mount_ang: float = 0.0,
        obs_noise_gravity_std: float = 0.0,
        obs_noise_gyro_std: float = 0.0,
        obs_noise_joint_pos_std: float = 0.0,
        obs_noise_joint_vel_std: float = 0.0,
        dt: float = 1 / 50,
        sim_dt: float = 1 / 2000,
    ):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # Timing matches IsaacLab setup: 250 Hz sim, 50 Hz control, decimation=5.
        self.model.opt.timestep = sim_dt
        self.dt = dt
        self.n_substeps = int(round(self.dt / self.model.opt.timestep))
        if not np.isclose(self.n_substeps * self.model.opt.timestep, self.dt):
            raise ValueError(
                f"control dt ({self.dt}) must be an integer multiple of sim dt ({self.model.opt.timestep})"
            )

        self._frame_stack = max(1, int(frame_stack))
        self._stack_frame_major = bool(stack_frame_major)
        self._use_phase_obs = bool(use_phase_obs)
        self._gait_period_s = float(gait_period_s)

        # Backward-compat mapping from old args -> new behavior.
        if phase_freeze_when_standing is not None:
            freeze_phase_when_standing = phase_freeze_when_standing
        if phase_cmd_lin_eps is not None:
            stand_phase_lin_threshold = phase_cmd_lin_eps
        if phase_cmd_yaw_eps is not None:
            stand_phase_yaw_threshold = phase_cmd_yaw_eps
        if phase_standing_clock is not None:
            psc = np.asarray(phase_standing_clock, dtype=float).reshape(-1)
            if psc.shape != (2,):
                raise ValueError("phase_standing_clock must be length-2, e.g. (0.0, 1.0)")
            # old clock=(sin, cos) -> phase angle
            stand_phase_value = math.atan2(float(psc[0]), float(psc[1]))
        if (phase_gate_when_standing is not None) and phase_gate_when_standing:
            # closest equivalent under new semantics
            freeze_phase_when_standing = True

        self._freeze_phase_when_standing = bool(freeze_phase_when_standing)
        self._stand_phase_lin_threshold = float(max(0.0, stand_phase_lin_threshold))
        self._stand_phase_yaw_threshold = float(max(0.0, stand_phase_yaw_threshold))
        self._stand_phase_value = float(stand_phase_value)
        self._include_height = bool(include_height)

        self._action_scale = float(action_scale)
        self._disturbance_force_max = float(disturbance_force_max)
        self._disturbance_torque_max = float(disturbance_torque_max)
        self._disturbance_prob = float(disturbance_prob)

        self._action_noise_std = float(action_noise_std)
        self._action_smoothing_alpha = float(np.clip(action_smoothing_alpha, 0.0, 1.0))
        self._action_delta_max = None if action_delta_max is None else float(max(0.0, action_delta_max))
        self._act_max_latency = max(0, int(act_max_latency))
        self._act_latency_steps = int(np.clip(act_latency_steps, 0, self._act_max_latency))

        self._obs_max_latency = max(0, int(obs_max_latency))
        self._obs_latency_steps = int(np.clip(obs_latency_steps, 0, self._obs_max_latency))

        self._imu_bias_gravity = np.asarray(imu_bias_gravity, dtype=float).copy()
        self._imu_bias_gyro = np.asarray(imu_bias_gyro, dtype=float).copy()
        self._imu_mount_axis = np.asarray(imu_mount_axis, dtype=float).copy()
        axis_norm = np.linalg.norm(self._imu_mount_axis)
        if axis_norm < 1e-8:
            self._imu_mount_axis[:] = np.array([1.0, 0.0, 0.0])
        else:
            self._imu_mount_axis /= axis_norm
        self._imu_mount_ang = float(imu_mount_ang)

        self._obs_noise_gravity_std = float(obs_noise_gravity_std)
        self._obs_noise_gyro_std = float(obs_noise_gyro_std)
        self._obs_noise_joint_pos_std = float(obs_noise_joint_pos_std)
        self._obs_noise_joint_vel_std = float(obs_noise_joint_vel_std)

        self._torso_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        self._torso_top_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu")

        self._nu = self.model.nu
        self._nq = self.model.nq
        self._nv = self.model.nv

        self._q_joint_start = 7
        self._qd_joint_start = 6

        self._policy_joint_order = [
            "left_hip1_joint",
            "right_hip1_joint",
            "left_hip2_joint",
            "right_hip2_joint",
            "left_thigh_joint",
            "right_thigh_joint",
            "left_knee_joint",
            "right_knee_joint",
            "left_ankle_joint",
            "right_ankle_joint",
        ]

        if len(self._policy_joint_order) != self._nu:
            raise ValueError(
                f"policy_joint_order length ({len(self._policy_joint_order)}) must equal num actuators ({self._nu})"
            )

        self._model_joint_order = []
        for actuator_id in range(self._nu):
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            self._model_joint_order.append(joint_name)

        self._policy_to_mj = np.array(
            [self._model_joint_order.index(jn) for jn in self._policy_joint_order],
            dtype=int,
        )
        self._mj_to_policy = np.argsort(self._policy_to_mj)

        # Mapping sanity checks.
        x = np.arange(self._nu)
        if not np.all(x == x[self._mj_to_policy][self._policy_to_mj]):
            raise ValueError("invalid reorder mapping: policy -> mj -> policy mismatch")
        if not np.all(x == x[self._policy_to_mj][self._mj_to_policy]):
            raise ValueError("invalid reorder mapping: mj -> policy -> mj mismatch")

        # Actuator-order joint ids/addresses (MuJoCo ctrl order).
        self._act_joint_ids = np.array(
            [int(self.model.actuator_trnid[i, 0]) for i in range(self._nu)],
            dtype=int,
        )
        self._act_qpos_adr = np.array([self.model.jnt_qposadr[j] for j in self._act_joint_ids], dtype=int)
        self._act_qvel_adr = np.array([self.model.jnt_dofadr[j] for j in self._act_joint_ids], dtype=int)

        # Policy-order joint ids/addresses (interleaved policy order).
        self._policy_joint_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn) for jn in self._policy_joint_order],
            dtype=int,
        )
        self._policy_qpos_adr = np.array([self.model.jnt_qposadr[j] for j in self._policy_joint_ids], dtype=int)
        self._policy_qvel_adr = np.array([self.model.jnt_dofadr[j] for j in self._policy_joint_ids], dtype=int)

        self._joint_action_scales = np.ones(self._nu, dtype=float)
        if action_scale_by_joint:
            for actuator_id in range(self._nu):
                actuator_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
                joint_id = int(self.model.actuator_trnid[actuator_id, 0])
                joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                if joint_name in action_scale_by_joint:
                    self._joint_action_scales[actuator_id] = float(action_scale_by_joint[joint_name])
                elif actuator_name in action_scale_by_joint:
                    self._joint_action_scales[actuator_id] = float(action_scale_by_joint[actuator_name])

        standing_key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "locomotion_standing_pose")
        self._standing_qpos = self.model.key_qpos[standing_key_id].copy()
        # Standing joint baseline in actuator order.
        self._standing_joint_pos_mj = self._standing_qpos[self._act_qpos_adr].copy()

        # Hard limits in *policy (interleaved)* order, read from model by joint name.
        # This keeps obs scaling and action clipping aligned with the policy indexing.
        self._joint_range_lower_policy = np.array(
            [
                float(
                    self.model.jnt_range[
                        mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name), 0
                    ]
                )
                for joint_name in self._policy_joint_order
            ],
            dtype=float,
        )
        self._joint_range_upper_policy = np.array(
            [
                float(
                    self.model.jnt_range[
                        mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name), 1
                    ]
                )
                for joint_name in self._policy_joint_order
            ],
            dtype=float,
        )

        self._soft_joint_limit_factor = 0.95
        # Shrink each joint range around its center (not around zero).
        range_center_policy = 0.5 * (self._joint_range_lower_policy + self._joint_range_upper_policy)
        range_halfspan_policy = 0.5 * (self._joint_range_upper_policy - self._joint_range_lower_policy)
        soft_halfspan_policy = self._soft_joint_limit_factor * range_halfspan_policy
        self._joint_soft_lower_policy = range_center_policy - soft_halfspan_policy
        self._joint_soft_upper_policy = range_center_policy + soft_halfspan_policy

        self._joint_range_lower_mj = self._joint_range_lower_policy[self._mj_to_policy]
        self._joint_range_upper_mj = self._joint_range_upper_policy[self._mj_to_policy]
        self._joint_soft_lower_mj = self._joint_soft_lower_policy[self._mj_to_policy]
        self._joint_soft_upper_mj = self._joint_soft_upper_policy[self._mj_to_policy]

        self._ang_vel_scale = 0.25
        self._dof_vel_scale = 0.1

        # Commands are in body frame [vx, vy, yaw_rate].
        self._commands = np.array([0.0, 0.0, 0.0], dtype=float)

        self._single_frame_size = 3 + 3 + 3 + 3 + self._nu + self._nu + self._nu
        if self._use_phase_obs:
            self._single_frame_size += 2
        self._obs_size = self._single_frame_size * self._frame_stack

        # Action buffers.
        self._actions = np.zeros(self._nu, dtype=float)
        self._last_act = np.zeros(self._nu, dtype=float)
        self._act_hist_buf = (
            np.zeros((self._nu, self._act_max_latency + 1), dtype=float)
            if self._act_max_latency > 0
            else None
        )

        # Observation latency + stack buffers.
        self._obs_hist_buf = (
            np.zeros((self._single_frame_size, self._obs_max_latency + 1), dtype=float)
            if self._obs_max_latency > 0
            else None
        )
        self._obs_stack_buf = (
            (
                np.zeros((self._frame_stack, self._single_frame_size), dtype=float)
                if self._stack_frame_major
                else np.zeros((self._single_frame_size, self._frame_stack), dtype=float)
            )
            if self._frame_stack > 1
            else None
        )

        self._step_count = 0

        self._torso_mass = 3.175
        self._approx_inertia = 0.02
        self._command_yaw_offset = -math.pi / 2.0  # -90 deg: align body frame with command frame
        self._cmd_yaw_cos = math.cos(self._command_yaw_offset)
        self._cmd_yaw_sin = math.sin(self._command_yaw_offset)
        self._use_cmd_yaw_offset = True

        # Cached state tensors used for observation construction.
        self.torso_quat_w = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.torso_lin_vel_w = np.zeros(3, dtype=float)
        self.torso_ang_vel_w = np.zeros(3, dtype=float)
        self.torso_lin_vel_b = np.zeros(3, dtype=float)
        self.torso_ang_vel_b = np.zeros(3, dtype=float)
        self.torso_lin_vel_cmd = np.zeros(3, dtype=float)
        self.torso_ang_vel_cmd = np.zeros(3, dtype=float)
        self.up_b = np.array([0.0, 0.0, 1.0], dtype=float)
        self.up_cmd = np.array([0.0, 0.0, 1.0], dtype=float)
        self.act_pos = np.zeros(self._nu, dtype=float)
        self.act_vel = np.zeros(self._nu, dtype=float)
        self.act_vel_scaled = np.zeros(self._nu, dtype=float)
        self.act_pos_scaled = np.zeros(self._nu, dtype=float)

        print("=" * 60)
        print("Humanoid Locomotion Validation (MuJoCo CPU)")
        print("=" * 60)
        print(f"Number of actuators: {self._nu}")
        print(f"Number of position DOFs: {self._nq}")
        print(f"Number of velocity DOFs: {self._nv}")
        print(f"Torso body ID: {self._torso_body_id}")
        print(f"Torso_top site ID: {self._torso_top_site_id}")
        print(f"Action order: {self._policy_joint_order}")
        print(f"Action scale: +/-{self._action_scale} rad")
        print(f"Control frequency: {1 / self.dt:.1f} Hz")
        print(f"Physics timestep: {self.model.opt.timestep}s")
        print(f"Substeps per control: {self.n_substeps}")
        print(f"Frame stack: {self._frame_stack} frames")
        print(f"Stack frame-major flatten: {self._stack_frame_major}")
        print(f"Use phase obs: {self._use_phase_obs}")
        print(f"Freeze phase when standing: {self._freeze_phase_when_standing}")
        print(f"Stand phase lin threshold: {self._stand_phase_lin_threshold}")
        print(f"Stand phase yaw threshold: {self._stand_phase_yaw_threshold}")
        print(f"Stand phase value (rad): {self._stand_phase_value:.4f}")
        print(f"Single frame obs size: {self._single_frame_size}")
        print(f"Total obs size: {self._obs_size}")
        print(f"Action noise std: {self._action_noise_std}")
        print(f"Action latency steps: {self._act_latency_steps} (max {self._act_max_latency})")
        print(f"Obs latency steps: {self._obs_latency_steps} (max {self._obs_max_latency})")
        print(f"Disturbance probability: {self._disturbance_prob:.1%}")
        if self._include_height:
            print("Note: include_height=True is ignored for IsaacLab-compatible policy observations.")
        if action_scale_by_joint:
            print("\nPer-joint action scale multipliers:")
            for actuator_id in range(self._nu):
                if self._joint_action_scales[actuator_id] != 1.0:
                    joint_id = int(self.model.actuator_trnid[actuator_id, 0])
                    joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                    print(f"  {joint_name}: {self._joint_action_scales[actuator_id]:.2f}")
        print("=" * 60)

        self._step_count = 0

        self._raw_actions = np.zeros(self._nu, dtype=float)

        self._imu_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu")
        if self._imu_site_id < 0:
            raise ValueError("IMU site 'imu' not found in MJCF.")

        self._Jp_imu = np.zeros((3, self.model.nv), dtype=np.float64)
        self._Jr_imu = np.zeros((3, self.model.nv), dtype=np.float64)


    def reset(self) -> np.ndarray:
        """Reset environment to standing pose and prefill observation histories."""
        self.data.qpos[:] = self._standing_qpos
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = self._standing_joint_pos_mj

        mujoco.mj_forward(self.model, self.data)

        self._actions[:] = 0.0
        self._last_act[:] = 0.0
        if self._act_hist_buf is not None:
            self._act_hist_buf[:, :] = 0.0

        self._raw_actions[:] = 0.0

        self._step_count = 0

        self._update_state()
        obs0 = self._compute_single_observation()

        if self._obs_hist_buf is not None:
            self._obs_hist_buf[:, :] = obs0[:, None]
        delayed_obs = obs0

        if self._obs_stack_buf is not None:
            if self._stack_frame_major:
                self._obs_stack_buf[:, :] = delayed_obs[None, :]
            else:
                self._obs_stack_buf[:, :] = delayed_obs[:, None]
            obs = self._obs_stack_buf.reshape(-1)
        else:
            obs = delayed_obs

        if self._torso_top_site_id >= 0:
            init_height = float(self.data.site_xpos[self._torso_top_site_id, 2])
        else:
            init_height = float(self.data.xpos[self._torso_body_id, 2])
        print(f"\nReset complete. Initial torso height: {init_height:.3f}m")
        return obs.copy()

    def step(self, action: np.ndarray) -> np.ndarray:
        """Step the environment by one policy step."""
        self._pre_physics_step(action)
        self._apply_action()

        self._apply_disturbance()
        for _ in range(self.n_substeps):
            mujoco.mj_step(self.model, self.data)

        self._step_count += 1
        return self._get_obs()

    def set_command(self, vx: float, vy: float, yaw_rate: float):
        self._commands[:] = np.array([vx, vy, yaw_rate], dtype=float)

    def _is_standing_command(self) -> bool:
        lin_norm = float(np.linalg.norm(self._commands[:2]))
        yaw_abs = float(abs(self._commands[2]))
        return (lin_norm < self._stand_phase_lin_threshold) and (yaw_abs < self._stand_phase_yaw_threshold)

    def _phase_for_observation(self) -> float:
        if self._gait_period_s <= 0.0:
            raise ValueError("gait_period_s must be > 0 when use_phase_obs is enabled")

        # IsaacLab-aligned: phase from episode time, not a separately advanced local clock.
        t = float(self._step_count) * self.dt
        phase = 2.0 * math.pi * (t / self._gait_period_s)

        if self._freeze_phase_when_standing and self._is_standing_command():
            phase = self._stand_phase_value

        return phase

    def _pre_physics_step(self, action: np.ndarray):
        """Isaac-style action preprocessing + optional smoothing:
        clip -> noise -> latency -> rate-limit -> EMA -> prev/current update.
        """
        action_policy = np.asarray(action, dtype=float).reshape(-1)
        if action_policy.shape != (self._nu,):
            raise ValueError(f"expected action shape {(self._nu,)}, got {action_policy.shape}")

        # 1) clip
        a = np.clip(action_policy, -1.0, 1.0)

        # 2) noise
        if self._action_noise_std > 0.0:
            a = np.clip(a + np.random.randn(*a.shape) * self._action_noise_std, -1.0, 1.0)

        # 3) latency (same as before)
        if self._act_hist_buf is not None:
            self._act_hist_buf = np.roll(self._act_hist_buf, shift=-1, axis=1)
            self._act_hist_buf[:, -1] = a
            idx = int(np.clip(self._act_max_latency - self._act_latency_steps, 0, self._act_max_latency))
            a = self._act_hist_buf[:, idx].copy()

        self._raw_actions = a.copy()  # optional debug

        # Previous applied action (for smoothing and for obs feature)
        prev_applied = self._actions.copy()
        a_cmd = a

        # 4) slew-rate limit (optional)
        # action_delta_max is in normalized action units per control step.
        if self._action_delta_max is not None:
            delta = np.clip(a_cmd - prev_applied, -self._action_delta_max, self._action_delta_max)
            a_cmd = prev_applied + delta

        # 5) EMA low-pass (optional)
        # alpha=1.0 => no filtering, alpha->0 => heavier filtering
        alpha = self._action_smoothing_alpha
        if alpha < 1.0:
            a_cmd = (1.0 - alpha) * prev_applied + alpha * a_cmd

        a_cmd = np.clip(a_cmd, -1.0, 1.0)

        # 6) update previous/current applied actions
        self._last_act = prev_applied
        self._actions = a_cmd



    def _apply_action(self):
        """Apply position-offset actions around the standing keyframe with soft-limit clipping."""
        action_mj = self._actions[self._mj_to_policy]
        standing_joint_pos = self._standing_joint_pos_mj

        pos_offsets = action_mj * self._action_scale * self._joint_action_scales
        target_positions = standing_joint_pos + pos_offsets

        self.data.ctrl[:] = np.clip(target_positions, self._joint_soft_lower_mj, self._joint_soft_upper_mj)

    def _apply_disturbance(self):
        """Apply random disturbance forces and torques."""
        if np.random.uniform() > self._disturbance_prob:
            return

        force_magnitude = np.random.uniform(0, self._disturbance_force_max)
        force_direction = np.random.randn(3)
        force_direction /= (np.linalg.norm(force_direction) + 1e-8)
        disturbance_force = force_direction * force_magnitude

        disturbance_velocity = disturbance_force * self.dt / self._torso_mass
        self.data.qvel[:3] += disturbance_velocity

        torque_magnitude = np.random.uniform(0, self._disturbance_torque_max)
        torque_direction = np.random.randn(3)
        torque_direction /= (np.linalg.norm(torque_direction) + 1e-8)
        disturbance_torque = torque_direction * torque_magnitude

        disturbance_ang_velocity = disturbance_torque * self.dt / self._approx_inertia
        self.data.qvel[3:6] += disturbance_ang_velocity

    def _update_state(self):
        self.torso_quat_w = self.data.xquat[self._torso_body_id].copy()  # (w,x,y,z)

        imu = self._get_imu_state()
        self.torso_lin_vel_w = imu["v_wi"].copy()
        self.torso_ang_vel_w = imu["w_w"].copy()
        self.torso_lin_vel_b = imu["v_i"].copy()
        self.torso_ang_vel_b = imu["w_i"].copy()
        self.up_b = imu["up_i"].copy()  # "up" in local frame

        # >>> THIS WAS MISSING <<<
        self.torso_lin_vel_cmd = self.torso_lin_vel_b.copy()
        self.torso_ang_vel_cmd = self.torso_ang_vel_b.copy()
        self.up_cmd = self.up_b.copy()

        # Optional: align sensor frame with command frame (if needed)
        if self._use_cmd_yaw_offset:
            self.torso_lin_vel_cmd = self._rotate_xy(self.torso_lin_vel_cmd, self._cmd_yaw_cos, self._cmd_yaw_sin)
            self.torso_ang_vel_cmd = self._rotate_xy(self.torso_ang_vel_cmd, self._cmd_yaw_cos, self._cmd_yaw_sin)
            self.up_cmd = self._rotate_xy(self.up_cmd, self._cmd_yaw_cos, self._cmd_yaw_sin)

        # Joint states (existing code)
        joint_pos_mj = self.data.qpos[self._act_qpos_adr].copy()
        joint_vel_mj = self.data.qvel[self._act_qvel_adr].copy()
        self.act_pos = joint_pos_mj[self._policy_to_mj]
        self.act_vel = joint_vel_mj[self._policy_to_mj]

        lo = self._joint_soft_lower_policy
        hi = self._joint_soft_upper_policy
        self.act_pos_scaled = 2.0 * (self.act_pos - lo) / (hi - lo + 1e-6) - 1.0
        self.act_vel_scaled = self.act_vel * self._dof_vel_scale


    def _get_imu_state(self) -> dict[str, np.ndarray]:
        """IMU state at site 'imu', independent of model/root origin choice."""
        sid = self._imu_site_id

        # World pose of IMU site
        p_wi = self.data.site_xpos[sid].copy()
        R_wi = self.data.site_xmat[sid].reshape(3, 3).copy()

        # Site Jacobians: map qvel -> site linear/angular velocity (world frame)
        mujoco.mj_jacSite(self.model, self.data, self._Jp_imu, self._Jr_imu, sid)
        qvel = self.data.qvel
        v_wi = self._Jp_imu @ qvel   # linear vel at IMU point (world)
        w_w  = self._Jr_imu @ qvel   # angular vel of IMU frame (world)

        # Convert to IMU local frame
        v_i = R_wi.T @ v_wi
        w_i = R_wi.T @ w_w

        # Project gravity into IMU frame
        g_w = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        g_i = R_wi.T @ g_w

        # Site quaternion (wxyz)
        q_wi = np.empty(4, dtype=np.float64)
        mujoco.mju_mat2Quat(q_wi, self.data.site_xmat[sid])

        up_w = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        up_i = R_wi.T @ up_w

        return {
            "p_wi": p_wi,
            "q_wi": q_wi,
            "v_i": v_i,
            "w_i": w_i,
            "g_i": g_i,
            "v_wi": v_wi,
            "w_w": w_w,
            "up_i": up_i,
        }


    def _compute_single_observation(self) -> np.ndarray:
        """Build one observation frame using IsaacLab ordering and transforms."""
        up_cmd = self.up_cmd.copy()
        ang_vel_cmd = self.torso_ang_vel_cmd.copy()
        commands = self._commands.copy()
        act_pos_scaled = self.act_pos_scaled.copy()
        act_vel_scaled = self.act_vel_scaled.copy()

        # if self._use_cmd_yaw_offset:
            # commands = self._rotate_xy(commands, self._cmd_yaw_cos, self._cmd_yaw_sin)

        # IMU biases.
        up_cmd = up_cmd + self._imu_bias_gravity
        ang_vel_cmd = ang_vel_cmd + self._imu_bias_gyro

        # IMU mount misalignment small-angle approx: v' ~= v + theta x v.
        theta = self._imu_mount_axis * self._imu_mount_ang
        up_cmd = up_cmd + np.cross(theta, up_cmd)
        ang_vel_cmd = ang_vel_cmd + np.cross(theta, ang_vel_cmd)

        # Observation noise.
        if self._obs_noise_gravity_std > 0.0:
            up_cmd = up_cmd + np.random.randn(*up_cmd.shape) * self._obs_noise_gravity_std
        if self._obs_noise_gyro_std > 0.0:
            ang_vel_cmd = ang_vel_cmd + np.random.randn(*ang_vel_cmd.shape) * self._obs_noise_gyro_std
        # if self._obs_noise_joint_pos_std > 0.0:
        #     act_pos_scaled = act_pos_scaled + np.random.randn(*act_pos_scaled.shape) * self._obs_noise_joint_pos_std
        # if self._obs_noise_joint_vel_std > 0.0:
        #     act_vel_scaled = act_vel_scaled + np.random.randn(*act_vel_scaled.shape) * (
        #         self._obs_noise_joint_vel_std * self._dof_vel_scale
        #     )

        obs = np.concatenate(
            [
                self.torso_lin_vel_cmd,
                ang_vel_cmd * self._ang_vel_scale,
                up_cmd,
                commands,
                act_pos_scaled,
                act_vel_scaled,
                self._last_act,
            ]
        )

        if self._use_phase_obs:
            phase = self._phase_for_observation()
            clock = np.array([math.sin(phase), math.cos(phase)], dtype=float)
            obs = np.concatenate([obs, clock])

        return obs

    def _get_obs(self) -> np.ndarray:
        """Isaac-style observation path: state -> single obs -> latency -> stack."""
        self._update_state()
        obs = self._compute_single_observation()

        if self._obs_hist_buf is not None:
            self._obs_hist_buf = np.roll(self._obs_hist_buf, shift=-1, axis=1)
            self._obs_hist_buf[:, -1] = obs

            idx = int(np.clip(self._obs_max_latency - self._obs_latency_steps, 0, self._obs_max_latency))
            obs = self._obs_hist_buf[:, idx].copy()

        if self._obs_stack_buf is not None:
            if self._stack_frame_major:
                self._obs_stack_buf = np.roll(self._obs_stack_buf, shift=-1, axis=0)
                self._obs_stack_buf[-1, :] = obs
            else:
                self._obs_stack_buf = np.roll(self._obs_stack_buf, shift=-1, axis=1)
                self._obs_stack_buf[:, -1] = obs
            obs = self._obs_stack_buf.reshape(-1)

        return obs.copy()

    def _rotate_vector(self, vec: np.ndarray, quat: np.ndarray, inverse: bool = False) -> np.ndarray:
        """Rotate a vector by a quaternion in (w, x, y, z) convention."""
        q = quat
        if inverse:
            q = np.array([quat[0], -quat[1], -quat[2], -quat[3]], dtype=float)

        rot_mat = np.zeros(9, dtype=float)
        mujoco.mju_quat2Mat(rot_mat, q)
        rot_mat = rot_mat.reshape(3, 3)
        return rot_mat @ vec

    def _rotate_xy(self, v: np.ndarray, cos_yaw: float, sin_yaw: float) -> np.ndarray:
        """Rotate xy components of vector by yaw angle."""
        x = v[..., 0] * cos_yaw - v[..., 1] * sin_yaw
        y = v[..., 0] * sin_yaw + v[..., 1] * cos_yaw
        z = v[..., 2] if v.shape[-1] > 2 else np.zeros_like(x)
        return np.stack([x, y, z], axis=-1)

    def render(self, mode: str = "rgb_array", width: int = 640, height: int = 480, camera_name: str | None = None):
        """Render the environment."""
        if mode != "rgb_array":
            raise NotImplementedError(f"Render mode '{mode}' not supported")

        renderer = mujoco.Renderer(self.model, height=height, width=width)
        if camera_name:
            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            renderer.update_scene(self.data, camera=camera_id)
        else:
            renderer.update_scene(self.data)
        return renderer.render()
