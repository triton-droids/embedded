"""TensorRT (device='gpu') must give the same football as ONNX Runtime CPU.

Skipped where no engine was built (scripts/build_ball_chase_engines.sh).
"""
import json
from pathlib import Path

import numpy as np
import pytest

from humanoid_ball_chase.football_detector import FootballDetector, engine_path
from humanoid_ball_chase.scene import assets_directory

MODELS = sorted(p for p in assets_directory().glob('*.onnx'))


def _frame():
    """A football-like disc on a pitch-green background with flat 1.8 m depth."""
    import cv2
    rgb = np.full((240, 424, 3), (30, 95, 28), np.uint8)
    cv2.circle(rgb, (200, 110), 13, (240, 240, 240), -1)
    for dx, dy in ((0, 0), (-7, -6), (7, -6), (-6, 7), (6, 7)):
        cv2.circle(rgb, (200 + dx, 110 + dy), 3, (10, 10, 10), -1)
    depth = np.full((240, 424), 1.8, np.float32)
    k = [216.0, 0, 211.5, 0, 216.0, 119.5, 0, 0, 1]
    return rgb, depth, k


@pytest.mark.parametrize('onnx', MODELS, ids=[p.stem for p in MODELS])
def test_gpu_matches_cpu(onnx):
    if not engine_path(onnx).exists():
        pytest.skip(f'no engine for {onnx.name}')
    cpu, gpu = FootballDetector(str(onnx), device='cpu'), FootballDetector(str(onnx), device='gpu')
    rgb = np.random.default_rng(0).integers(0, 255, (1, 3, 640, 640)).astype(np.float32) / 255
    a, b = cpu._infer(rgb)[0], gpu._infer(rgb)[0]
    assert a.shape == b.shape
    assert np.abs(a[4:] - b[4:]).max() < 0.02               # class scores, FP16 vs FP32
    confident = a[4:].max(axis=0) > 0.25                     # boxes of near-zero score may drift
    assert np.abs(a[:4, confident] - b[:4, confident]).max(initial=0) < 1.0   # pixels of 640 input
    frame = _frame()
    ra, rb = cpu.detect(*frame), gpu.detect(*frame)
    assert (ra is None) == (rb is None)
    if ra is not None:
        assert np.allclose(ra[:2], rb[:2], atol=1.0)
        assert abs(cpu.last_confidence - gpu.last_confidence) < 0.02


def test_gpu_rejects_engine_from_other_onnx(tmp_path):
    onnx = next((p for p in MODELS if engine_path(p).exists()), None)
    if onnx is None:
        pytest.skip('no engine built')
    copy = tmp_path / onnx.name
    engine = engine_path(copy)
    engine.write_bytes(engine_path(onnx).read_bytes())
    Path(str(engine) + '.json').write_text(json.dumps({'onnx_sha256': '0' * 64}))
    copy.write_bytes(onnx.read_bytes())
    with pytest.raises(ValueError, match='different ONNX'):
        FootballDetector(str(copy), device='gpu')


def test_gpu_without_engine_names_the_build_script(tmp_path):
    copy = tmp_path / MODELS[0].name
    copy.write_bytes(MODELS[0].read_bytes())
    with pytest.raises(FileNotFoundError, match='build_ball_chase_engines.sh'):
        FootballDetector(str(copy), device='gpu')
