"""Tracking-policy math, independent of ROS. All joint angles are radians."""
import math

import numpy as np
import onnx
import onnxruntime as ort


class TrackingPolicy:
    """The 56-input, ten-joint tracking export with embedded reference motion."""

    def __init__(self, model_path):
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self.session = ort.InferenceSession(
            str(model_path), sess_options=options, providers=['CPUExecutionProvider'])
        inputs = {item.name: (item.shape, item.type) for item in self.session.get_inputs()}
        if inputs != {'obs': ([1, 56], 'tensor(float)'),
                      'time_step': ([1, 1], 'tensor(float)')}:
            raise ValueError(f'Unsupported tracking model inputs: {inputs}')
        meta = self.session.get_modelmeta().custom_metadata_map
        if meta.get('observation_names') != 'command,base_ang_vel,joint_pos,joint_vel,actions,gravity':
            raise ValueError('Unsupported observation order')
        self.names = meta['joint_names'].split(',')
        if len(self.names) != 10 or len(set(self.names)) != 10:
            raise ValueError('Expected ten unique policy joints')
        self.offset = self._vector(meta['default_joint_pos'])
        self.scale = self._vector(meta['action_scale'])
        self.kp = self._vector(meta['joint_stiffness'])
        self.kd = self._vector(meta['joint_damping'])
        tensors = {item.name: onnx.numpy_helper.to_array(item)
                   for item in onnx.load(str(model_path)).graph.initializer}
        self.reference_q = tensors['joint_pos.1']
        self.reference_dq = tensors['joint_vel.1']
        if (self.reference_q.ndim != 2 or self.reference_q.shape[1] != 10
                or self.reference_q.shape != self.reference_dq.shape
                or len(self.reference_q) == 0
                or not np.isfinite(self.reference_q).all()
                or not np.isfinite(self.reference_dq).all()):
            raise ValueError('Invalid embedded reference motion')
        self.frames = len(self.reference_q)

    @staticmethod
    def _vector(text):
        values = np.fromstring(text, sep=',', dtype=np.float32)
        if values.shape == (1,):
            values = np.repeat(values, 10)
        if values.shape != (10,) or not np.isfinite(values).all():
            raise ValueError('Invalid policy metadata vector')
        return values

    def observation(self, frame, omega, gravity, q, dq, previous):
        """Reference q/dq, body gyro, q-offset, dq, raw prior action, down vector."""
        for value, size in ((omega, 3), (gravity, 3), (q, 10), (dq, 10), (previous, 10)):
            if np.asarray(value).shape != (size,):
                raise ValueError('Invalid observation component shape')
        if not 0 <= frame < self.frames:
            raise ValueError('Reference frame is out of range')
        obs = np.concatenate((self.reference_q[frame], self.reference_dq[frame],
                              omega, q - self.offset, dq, previous, gravity))
        obs = obs.astype(np.float32)[None, :]
        if obs.shape != (1, 56) or not np.isfinite(obs).all():
            raise ValueError('Invalid observation')
        return obs

    def run(self, obs, frame):
        actions = self.session.run(['actions'], {
            'obs': obs, 'time_step': np.asarray([[frame]], dtype=np.float32)})[0][0]
        if actions.shape != (10,) or not np.isfinite(actions).all():
            raise ValueError('Invalid policy output')
        if hasattr(self, "action_clip"):
            actions = np.clip(actions, -self.action_clip, self.action_clip)
        targets = self.offset + self.scale * actions
        if hasattr(self, "target_limits"):
            targets = np.clip(targets, *self.target_limits)
        if not np.isfinite(targets).all():
            raise ValueError('Invalid position targets')
        # This export has no [-1, 1] clamp. Preserve its trained action semantics.
        return actions, targets


class GravityEstimator:
    """Gyro propagation plus specific-force correction for six-axis bench data."""

    def __init__(self):
        self.gravity = None
        self.last_time = None

    def update(self, acceleration, omega, now):
        norm = np.linalg.norm(acceleration)
        if not np.isfinite(norm) or not 0.1 < norm < 100:
            raise ValueError('Invalid acceleration')
        measured = -acceleration / norm
        dt = 0.0 if self.last_time is None else now - self.last_time
        if self.gravity is None or not 0 < dt < 0.2:
            self.gravity = measured.copy()
        else:
            self.gravity -= np.cross(omega, self.gravity) * dt
            if 0.8 * 9.80665 < norm < 1.2 * 9.80665:
                alpha = 1 - math.exp(-dt / 0.5)
                self.gravity = (1 - alpha) * self.gravity + alpha * measured
            self.gravity /= np.linalg.norm(self.gravity)
        self.last_time = now
        return self.gravity.copy()


def joint_feedback(names, positions, velocities, policy_names):
    """Require a complete finite frame; never reuse missing or old joint values."""
    if (len(set(names)) != len(names) or len(positions) != len(names)
            or len(velocities) != len(names)):
        raise ValueError('JointState needs unique names and full position/velocity arrays')
    index = {name: i for i, name in enumerate(names)}
    if not all(name in index for name in policy_names):
        raise ValueError('JointState is missing policy joints')
    q = np.array([positions[index[name]] for name in policy_names], dtype=np.float32)
    dq = np.array([velocities[index[name]] for name in policy_names], dtype=np.float32)
    if not np.isfinite(q).all() or not np.isfinite(dq).all():
        raise ValueError('JointState contains non-finite values')
    return q, dq


def validate_tracking_registry(policy, params, model_sha256):
    """Reject drift between the embedded registry and the loaded model."""
    contract = params['model_contract']
    if (contract['format'] != 'tracking_onnx' or contract['control_hz'] != 50.0
            or contract['model_sha256'] != model_sha256
            or contract['joint_order'] != policy.names
            or list(params['motors']) != policy.names):
        raise ValueError('Motor registry does not match the tracking ONNX export; resync it')
    for key, values in (('default_joint_pos', policy.offset), ('action_scale', policy.scale),
                        ('joint_stiffness', policy.kp), ('joint_damping', policy.kd)):
        configured = np.asarray(contract[key], dtype=np.float32)
        if configured.shape != values.shape or not np.array_equal(configured, values):
            raise ValueError(f'Motor registry {key} differs from ONNX metadata')
    if 'runtime_policy' in params:
        return
    for index, name in enumerate(policy.names):
        motor = params['motors'][name]
        if (np.float32(motor['kp']) != policy.kp[index]
                or np.float32(motor['kd']) != policy.kd[index]):
            raise ValueError(f'{name}: motor gains differ from tracking ONNX metadata')


def fresh(receipt_time, stamp_time, now_monotonic, now_ros, timeout):
    """Gate both host receipt age and ROS message timestamp; reject future stamps."""
    return (receipt_time is not None and stamp_time is not None
            and 0 <= now_monotonic - receipt_time <= timeout
            and -0.01 <= now_ros - stamp_time <= timeout)


def apply_runtime_policy(policy, params):
    """Apply explicit deployment settings after validating original ONNX metadata."""
    runtime = params.get('runtime_policy')
    if runtime is None:
        return
    if runtime['control_hz'] != 50.0:
        raise ValueError('Runtime policy must remain at 50 Hz')
    names = policy.names
    policy.offset = np.asarray([runtime['default_joint_pos_real_rad_by_joint'][n] for n in names], dtype=np.float32)
    policy.scale = np.asarray([runtime['action_scale_by_joint'].get(n, runtime['action_scale']) for n in names], dtype=np.float32)
    policy.kp = np.asarray([params['motors'][n]['kp'] for n in names], dtype=np.float32)
    policy.kd = np.asarray([params['motors'][n]['kd'] for n in names], dtype=np.float32)
    policy.action_clip = float(runtime['policy_action_clip'])
    if not all(np.isfinite(v).all() for v in (policy.offset, policy.scale, policy.kp, policy.kd)) or not math.isfinite(policy.action_clip) or policy.action_clip <= 0:
        raise ValueError('Invalid runtime policy settings')
    if runtime['use_soft_joint_limits']:
        factor = float(runtime['soft_joint_limit_factor'])
        lower = np.asarray([params['motors'][n]['min_position'] for n in names])
        upper = np.asarray([params['motors'][n]['max_position'] for n in names])
        center, half = (lower + upper) / 2, (upper - lower) * factor / 2
        policy.target_limits = (center - half, center + half)
