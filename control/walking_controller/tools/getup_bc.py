"""Train the get-up student by behaviour cloning on getup_demo.py episodes (BC, and DAgger rounds when the
directories include episodes recorded with --student).

  tools/getup_bc.py --data demos dagger1 --out bc.npz [--onnx bc.onnx] [--epochs 60] [--device cuda]

Writes the numpy policy getup_demo.py --student loads (obs normalisation + MLP weights) and, with --onnx,
an ONNX model (input "obs" [1, N], output "actions" [1, 18]) whose metadata carries the observation layout,
joint order, default joint positions and action scale. Needs torch (and onnx for --onnx); no MuJoCo.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def load_episode(path, history, time_input):
    z = np.load(path)
    frames, prev, label = z['frame'], z['prev'], z['label']
    if len(frames) == 0 or not np.isfinite(label).all():
        return None
    padded = np.concatenate([np.repeat(frames[:1], history - 1, 0), frames])
    stacked = np.stack([padded[i:i + len(frames)] for i in range(history)], 1).reshape(len(frames), -1)
    meta = dict(q_nominal=z['q_nominal'], scale=float(z['scale']), history=history,
                joint_names=[str(n) for n in z['joint_names']], command_dim=0)
    parts = [stacked, prev]
    if time_input:
        parts.append(np.clip(z['t'][:, None] / 10.0, 0.0, 2.0))
    if 'command' in z.files:
        command = z['command']
        if command.ndim != 2 or len(command) != len(frames):
            raise SystemExit(f'{path}: command must have shape (ticks, command_dim)')
        meta['command_dim'] = command.shape[1]
        parts.append(command)
    return np.concatenate(parts, 1), label, meta


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--data', type=Path, nargs='+', action='extend', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--onnx', type=Path)
    parser.add_argument('--hidden', type=int, nargs='+', default=[512, 256, 128])
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--batch', type=int, default=1024)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--history', type=int, default=3, help='frames per observation (50 Hz)')
    parser.add_argument('--time-input', action='store_true', help='add the time since the get-up started')
    parser.add_argument('--obs-noise', type=float, default=0.0, help='Gaussian noise on normalised inputs')
    parser.add_argument('--dropout', type=float, default=0.0)
    parser.add_argument('--upright-repeat', type=int, default=1,
                        help='repeat training samples whose torso is above --upright-cos this many times')
    parser.add_argument('--upright-cos', type=float, default=0.7)
    parser.add_argument('--weight-decay', type=float, default=1e-4)
    args = parser.parse_args()
    import torch

    episodes, meta = [], None
    for directory in args.data:
        for path in sorted(directory.glob('*.npz')):
            loaded = load_episode(path, args.history, args.time_input)
            if loaded:
                if meta is not None and loaded[2]['command_dim'] != meta['command_dim']:
                    raise SystemExit(f'{path}: mixed command_dim values in training episodes '
                                     f'({meta["command_dim"]} and {loaded[2]["command_dim"]})')
                episodes.append(loaded[:2])
                meta = loaded[2]
    if not episodes:
        raise SystemExit('no episodes found')
    task = 'walk' if meta['command_dim'] > 0 else 'getup'
    rng = np.random.default_rng(args.seed)
    order = rng.permutation(len(episodes))
    n_val = max(1, len(episodes) // 10) if len(episodes) > 5 else 0
    split = {'val': order[:n_val], 'train': order[n_val:]}
    data = {k: (np.concatenate([episodes[i][0] for i in v]) if len(v) else np.zeros((0, 1)),
                np.concatenate([episodes[i][1] for i in v]) if len(v) else np.zeros((0, 1)))
            for k, v in split.items()}
    x_train, y_train = data['train']
    if args.upright_repeat > 1:
        # Newest frame's gravity z is at offset (history-1)*frame + 5; upright = -gravity_z.
        frame_size = (x_train.shape[1] - y_train.shape[1] - int(args.time_input)
                      - meta['command_dim']) // args.history
        up = -x_train[:, (args.history - 1) * frame_size + 5]
        extra = np.repeat(np.nonzero(up > args.upright_cos)[0], args.upright_repeat - 1)
        print(f'upright samples {np.mean(up > args.upright_cos):.1%}, repeated x{args.upright_repeat}', flush=True)
        x_train, y_train = np.concatenate([x_train, x_train[extra]]), np.concatenate([y_train, y_train[extra]])
    mean, std = x_train.mean(0), x_train.std(0) + 1e-3
    print(f'{len(episodes)} episodes, {len(x_train)} train / {len(data["val"][0])} val samples, '
          f'obs {x_train.shape[1]}, act {y_train.shape[1]}', flush=True)

    torch.manual_seed(args.seed)
    dev = torch.device(args.device)
    layers, width = [], x_train.shape[1]
    for h in args.hidden:
        layers += [torch.nn.Linear(width, h), torch.nn.ELU()]
        if args.dropout > 0:
            layers.append(torch.nn.Dropout(args.dropout))
        width = h
    layers.append(torch.nn.Linear(width, y_train.shape[1]))
    net = torch.nn.Sequential(*layers).to(dev)
    to = lambda a: torch.as_tensor(a, dtype=torch.float32, device=dev)  # noqa: E731
    xt, yt = to((x_train - mean) / std), to(y_train)
    xv, yv = to((data['val'][0] - mean) / std), to(data['val'][1])
    opt = torch.optim.AdamW(net.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, args.epochs)
    history = []
    for epoch in range(args.epochs):
        net.train()
        perm = torch.randperm(len(xt), device=dev)
        total = 0.0
        for i in range(0, len(xt), args.batch):
            idx = perm[i:i + args.batch]
            xb = xt[idx]
            if args.obs_noise > 0:
                xb = xb + args.obs_noise * torch.randn_like(xb)
            loss = torch.nn.functional.mse_loss(net(xb), yt[idx])
            opt.zero_grad()
            loss.backward()
            opt.step()
            total += loss.item() * len(idx)
        sched.step()
        net.eval()
        with torch.no_grad():
            val = torch.nn.functional.mse_loss(net(xv), yv).item() if len(xv) else float('nan')
        history.append((total / len(xt), val))
        if epoch % 10 == 0 or epoch == args.epochs - 1:
            print(f'epoch {epoch:3d} train {total / len(xt):.4f} val {val:.4f}', flush=True)

    linear = [m for m in net if isinstance(m, torch.nn.Linear)]
    weights = {f'w{i}': m.weight.detach().cpu().numpy() for i, m in enumerate(linear)}
    weights.update({f'b{i}': m.bias.detach().cpu().numpy() for i, m in enumerate(linear)})
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.out, obs_mean=mean.astype(np.float32), obs_std=std.astype(np.float32), layers=len(linear),
             q_nominal=meta['q_nominal'], scale=meta['scale'], history=meta['history'], time_input=args.time_input,
             command_dim=meta['command_dim'], task=task,
             joint_names=np.array(meta['joint_names']), **weights)
    args.out.with_suffix('.json').write_text(json.dumps(dict(
        episodes=len(episodes), samples=len(x_train), data=[str(d) for d in args.data], hidden=args.hidden,
        epochs=args.epochs, final_train=history[-1][0], final_val=history[-1][1]), indent=2) + '\n')

    if args.onnx:
        import onnx

        class Normalised(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.net = net.cpu()
                self.register_buffer('mean', torch.as_tensor(mean, dtype=torch.float32))
                self.register_buffer('std', torch.as_tensor(std, dtype=torch.float32))

            def forward(self, obs):
                return self.net((obs - self.mean) / self.std)

        model = Normalised().eval()
        torch.onnx.export(model, torch.zeros(1, len(mean)), str(args.onnx), input_names=['obs'],
                          output_names=['actions'], opset_version=17, dynamo=False)
        proto = onnx.load(str(args.onnx))
        csv = lambda v: ','.join(f'{x:.6g}' for x in v)  # noqa: E731
        for key, value in dict(
                observation_names=f'history{meta["history"]}(base_ang_vel,gravity,joint_pos,joint_vel),actions'
                + (',elapsed_s/10' if args.time_input else '')
                + (',command' if meta['command_dim'] else ''),
                command_dim=str(meta['command_dim']), task=task,
                joint_names=','.join(meta['joint_names']), default_joint_pos=csv(meta['q_nominal']),
                action_scale=csv([meta['scale']] * len(meta['q_nominal'])), control_dt='0.02',
                kind='getup_student').items():
            entry = proto.metadata_props.add()
            entry.key, entry.value = key, value
        onnx.save(proto, str(args.onnx))
    print(f'wrote {args.out}' + (f' and {args.onnx}' if args.onnx else ''), flush=True)


if __name__ == '__main__':
    main()
