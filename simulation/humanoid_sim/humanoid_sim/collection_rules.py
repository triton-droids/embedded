"""Pure collection/label definitions shared by the simulator and offline jobs."""
import math
import random

TIME_EPS_S = 1e-9


def future_label(t, fall_time, observed_until, horizon):
    """Unknown future is None; boundary comparisons tolerate numerical roundoff."""
    if fall_time is not None:
        delta = fall_time-t
        if delta <= 0:
            return None
        if delta <= horizon+TIME_EPS_S:
            return 1
    return 0 if observed_until-t+TIME_EPS_S >= horizon else None


def recovery_scenario(seed):
    """48 scenario families; repeats change force/time without crossing folds.

    Mode x floor friction x horizontal direction sector defines a family.
    Outcome is never used to assign train/validation/test.
    """
    rng = random.Random(int(seed))
    case = seed % 48
    mode = 'walking_reference' if case < 24 else 'standing_reference'
    friction_index = case % 3
    sector = (case % 24)//3
    cycle = seed//48
    limits = [(5.,15.),(15.,35.),(35.,65.)][cycle % 3]
    force = 0. if cycle % 6 == 0 else rng.uniform(*limits)
    angle = sector*math.pi/4+rng.uniform(-math.pi/16,math.pi/16)
    return dict(family=f'{mode}:friction{friction_index}:sector{sector}',mode=mode,
                friction=[.65,.85,1.15][friction_index],
                force_world_n=[force*math.cos(angle),force*math.sin(angle),0.],
                delay_s=rng.uniform(.4,2.4),duration_s=rng.uniform(.06,.18))
