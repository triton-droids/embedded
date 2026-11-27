"""Registers the humanoid walking task with mjlab."""

from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from humanoid_mjlab.env_cfg import make_env_cfg
from humanoid_mjlab.rl_cfg import make_rl_cfg

register_mjlab_task(
    task_id="Mjlab-Velocity-Flat-Humanoid",
    env_cfg=make_env_cfg(),
    play_env_cfg=make_env_cfg(play=True),
    rl_cfg=make_rl_cfg(),
    runner_cls=VelocityOnPolicyRunner,
)
