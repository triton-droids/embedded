"""Penalties shared by the tasks: behaviour the real motors and structure cannot (or should not) do."""


def realism_penalty(meas, airborne=1.0, vz_limit=0.8, power_w=2e-4, speed_ratio_limit=0.8):
    """Flight (no floor contact), fast vertical motion, mechanical power, motors near their no-load speed."""
    p = airborne * meas['airborne']
    p += 2.0 * max(0.0, abs(meas['vz']) - vz_limit)
    p += power_w * meas['power']
    p += 5.0 * max(0.0, meas['speed_ratio'] - speed_ratio_limit)
    return p


def flat_factor(meas, scale_deg=120.0):
    """1 with both soles flat, falling linearly with the summed tilt (0 at scale_deg)."""
    return max(0.0, 1.0 - (meas['tilts'][0] + meas['tilts'][1]) / scale_deg)
