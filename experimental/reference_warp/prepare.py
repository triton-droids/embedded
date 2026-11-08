"""Create isolated reference variants; never edits source model or motor registry."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
import onnx
from onnx import numpy_helper
import yaml

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'humanoid_control/motor_control_hybrid/models/20260914_170827'
REGISTRY = ROOT / 'humanoid_control/motor_control_hybrid/config/motors.yaml'


def warp(q, dq, names, scale):
    """Scale sagittal excursions about frame zero, ramping over the first second.

    XML axes identify hip1/knee/ankle as X hinges; thigh is a Z hinge, so is
    deliberately excluded. q'=q0+g(t)*(q-q0), dq'=g*dq+g_dot*(q-q0).
    These are joint-space excursions, not a calibrated Cartesian step length.
    """
    if not np.isfinite(scale) or not 0.9 <= scale <= 1.1:
        raise ValueError('Experimental scale must be finite and in [0.9, 1.1]')
    q, dq = np.asarray(q), np.asarray(dq)
    if q.shape != dq.shape or q.ndim != 2 or q.shape[1] != len(names):
        raise ValueError('Invalid reference arrays')
    if not np.isfinite(q).all() or not np.isfinite(dq).all():
        raise ValueError('Nonfinite reference')
    if scale == 1.0:
        return q.copy(), dq.copy()
    t = np.minimum(np.arange(len(q), dtype=np.float64)/50.0, 1.0)
    blend = 10*t**3-15*t**4+6*t**5
    blend_dot = 30*t*t-60*t**3+30*t**4
    gain = 1+(scale-1)*blend
    gain_dot = (scale-1)*blend_dot
    selected = [i for i, name in enumerate(names)
                if name.endswith(('_hip1_joint', '_knee_joint', '_ankle_joint'))]
    if len(selected) != 6:
        raise ValueError('Expected the six known sagittal hinges')
    out_q, out_dq = q.copy(), dq.copy()
    excursion = q[:, selected]-q[0, selected]
    out_q[:, selected] = q[0, selected]+gain[:, None]*excursion
    out_dq[:, selected] = gain[:, None]*dq[:, selected]+gain_dot[:, None]*excursion
    return out_q, out_dq


def prepare(scale, output):
    from motor_control_hybrid.tracking_onnx import TrackingPolicy, validate_tracking_registry
    output = Path(output).resolve()
    # Only write generated artifacts beneath this experiment.
    allowed = Path(__file__).resolve().parent / 'generated'
    if not output.is_relative_to(allowed) or output == allowed:
        raise ValueError(f'Output must be a subdirectory of {allowed}')
    original = SOURCE/'20260914_170827.onnx'
    policy = TrackingPolicy(original)
    q, dq = warp(policy.reference_q, policy.reference_dq, policy.names, scale)
    if np.any(q < policy.target_limits[0]) or np.any(q > policy.target_limits[1]):
        raise ValueError('Modified reference exceeds original joint limits; refusing to clip')
    output.mkdir(parents=True, exist_ok=True)
    model_path = output/original.name
    if scale == 1.0:
        shutil.copyfile(original, model_path)
    else:
        model = onnx.load(original)
        for tensor in model.graph.initializer:
            if tensor.name in ('joint_pos.1', 'joint_vel.1'):
                tensor.CopyFrom(numpy_helper.from_array(
                    q if tensor.name == 'joint_pos.1' else dq, tensor.name))
        onnx.checker.check_model(model)
        onnx.save(model, model_path)
    for name in ('chrobot_16kg_candidate.xml', 'chrobot_16kg_actuated.xml'):
        shutil.copyfile(SOURCE/name, output/name)
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    registry = yaml.safe_load(REGISTRY.read_text())
    params = registry['motor_control_node']['ros__parameters']
    if params.get('hardware_verified') is not False:
        raise ValueError('Experiment requires hardware_verified=false')
    params['model_contract']['model_sha256'] = digest(model_path)
    registry_path = output/'motors.yaml'
    registry_path.write_text(yaml.safe_dump(registry, sort_keys=False))
    modified = TrackingPolicy(model_path)
    validate_tracking_registry(modified, params, digest(model_path))
    # Assert neural weights and every unrelated initializer are bit-identical.
    before = {x.name: x.SerializeToString() for x in onnx.load(original).graph.initializer}
    after = {x.name: x.SerializeToString() for x in onnx.load(model_path).graph.initializer}
    assert before.keys() == after.keys()
    assert all(before[n] == after[n] for n in before if n not in ('joint_pos.1', 'joint_vel.1'))
    manifest = dict(scale=scale, source_sha256=digest(original), model_sha256=digest(model_path),
                    source_registry_sha256=digest(REGISTRY), hardware_verified=False,
                    max_reference_delta_rad=float(np.max(np.abs(q-policy.reference_q))),
                    affected_joints=[n for n in policy.names if n.endswith(
                        ('_hip1_joint', '_knee_joint', '_ankle_joint'))],
                    ramp_s=1.0, control_hz=50, neural_weights_unchanged=True)
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return model_path, registry_path, manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scale', type=float, required=True)
    args = parser.parse_args()
    directory = Path(__file__).resolve().parent/'generated'/f'scale_{args.scale:.3f}'
    model, registry, manifest = prepare(args.scale, directory)
    print(json.dumps(dict(model_path=str(model), motor_config_file=str(registry), **manifest), indent=2))
