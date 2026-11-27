"""CPU checks for the walking reward and low-torso termination.

Run from embedded: mjlab/.venv/bin/python -m unittest discover -s humanoid_mjlab/tests
"""

import unittest
from types import SimpleNamespace

import torch
from humanoid_mjlab.env_cfg import make_env_cfg, single_support_air_time
from mjlab.tasks.velocity.config.g1.env_cfgs import unitree_g1_flat_env_cfg


class WalkingRewardTests(unittest.TestCase):
    def test_single_support_air_time(self):
        # Standing, left/right swing, hopping, too-short/long swings, and no command.
        contacts = torch.tensor(
            [[2, 3], [0, 2], [3, 0], [0, 0], [0, 1], [1, 0], [0, 1]]
        )
        air_times = torch.tensor(
            [
                [0.0, 0.0],
                [0.2, 0.0],
                [0.0, 0.2],
                [0.2, 0.2],
                [0.04, 0.0],
                [0.0, 0.6],
                [0.2, 0.0],
            ]
        )
        commands = torch.tensor([[0.5, 0.0, 0.0]] * 6 + [[0.0, 0.0, 0.0]])
        env = SimpleNamespace(
            scene={
                "feet_ground_contact": SimpleNamespace(
                    data=SimpleNamespace(found=contacts, current_air_time=air_times)
                )
            },
            command_manager=SimpleNamespace(get_command=lambda name: commands),
            extras={"log": {}},
        )
        result = (
            single_support_air_time(
                env, "feet_ground_contact", command_name="twist", command_threshold=0.1
            )
            * 0.5
        )
        torch.testing.assert_close(
            result, torch.tensor([0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0])
        )
        self.assertTrue(torch.isfinite(result).all())

    def test_g1_reward_baseline(self):
        for play in (False, True):
            cfg = make_env_cfg(play=play)
            baseline = unitree_g1_flat_env_cfg(play=play)
            for name, term in cfg.rewards.items():
                expected = baseline.rewards[name]
                if name == "air_time":
                    self.assertIs(term.func, single_support_air_time)
                    self.assertEqual(term.weight, 0.5)
                    self.assertEqual(term.params["command_threshold"], 0.1)
                    self.assertEqual(term.params["threshold_min"], 0.05)
                    self.assertEqual(term.params["threshold_max"], 0.5)
                    continue
                self.assertIs(term.func, expected.func, name)
                self.assertEqual(term.weight, expected.weight, name)
                for param, value in term.params.items():
                    if param not in {
                        "asset_cfg",
                        "std_standing",
                        "std_walking",
                        "std_running",
                    }:
                        self.assertEqual(value, expected.params[param], (name, param))
            self.assertEqual(
                cfg.rewards["pose"].params["std_standing"],
                baseline.rewards["pose"].params["std_standing"],
            )

    def test_low_torso_termination(self):
        for play in (False, True):
            cfg = make_env_cfg(play=play)
            term = cfg.terminations["torso_too_low"]
            limit = term.params["minimum_height"]
            reset_height = cfg.scene.entities["robot"].init_state.pos[2]
            heights = torch.tensor([reset_height, 0.6588, limit, limit - 0.001, 0.3])
            pos = torch.zeros((len(heights), 3))
            pos[:, 2] = heights
            env = SimpleNamespace(
                scene={
                    "robot": SimpleNamespace(data=SimpleNamespace(root_link_pos_w=pos))
                }
            )
            self.assertFalse(term.time_out)
            self.assertEqual(
                term.func(env, **term.params).tolist(),
                [False, False, False, True, True],
            )
            self.assertIn("fell_over", cfg.terminations)


if __name__ == "__main__":
    unittest.main()
