"""Closed-form four-bar against motor_control_hybrid's AnkleMapper (values from motors.yaml ankle_mapping)."""
import math

from humanoid_sim.ankle_linkage import AnkleLinkage

CFG = {'link_lengths': [6.5625, 1.875, 6.5, 1.79], 'theta2_offset_deg': 90.0,
       't4_to_motor_offset_deg': -90.0, 'ankle_to_theta2_sign': 1.0}
# ankle deg -> motor deg from AnkleMapper.ankle_rad_to_motor_logical_rad, continued from 0
REFERENCE = {-65.0: -55.929, -45.0: -39.827, 0.0: 1.927, 45.0: 45.115, 75.0: 72.899}


def test_matches_ankle_mapper():
    for ankle, motor in REFERENCE.items():
        linkage = AnkleLinkage(CFG)
        linkage.reset(math.radians(ankle))
        m, _ = linkage.motor(math.radians(ankle))
        assert abs(math.degrees(m) - motor) < 0.01, (ankle, math.degrees(m))


def test_ratio_is_derivative():
    linkage = AnkleLinkage(CFG)
    for ankle in (-1.0, -0.3, 0.0, 0.4, 1.2):
        linkage.reset(ankle)
        m, ratio = linkage.motor(ankle)
        h = 1e-6
        numeric = (linkage.motor(ankle + h, m)[0] - linkage.motor(ankle - h, m)[0]) / (2 * h)
        assert abs(ratio - numeric) < 1e-6


def test_dead_points_lose_drive():
    linkage = AnkleLinkage(CFG)
    linkage.reset(0.0)
    assert 0.9 < linkage.motor(0.0)[1] < 1.0
    ratios = []
    for i in range(0, 1700):  # 0 .. 97 deg
        ratios.append(linkage.motor(math.radians(i * 0.057))[1])
    assert min(ratios) < 0.0 < max(ratios)  # passes the dead point near +92 deg
