import math
import unittest

from humanoid_mjlab.path_follower import PathFollower
from humanoid_mjlab.plan_path import PlanningMap, plan_path


class NavigationTests(unittest.TestCase):
    def test_rewiring_improves_route_with_more_samples(self):
        world = PlanningMap(
            (0.0, 0.0, 6.0, 5.0), ((2.5, 0.0, 3.5, 3.0),),
            (1.0, 1.0), (5.0, 1.0), 0.3,
        )
        short = plan_path(world, iterations=150, tracking_buffer=0)
        long = plan_path(world, iterations=900, tracking_buffer=0)
        self.assertIsNotNone(short)
        self.assertIsNotNone(long)
        self.assertEqual((long[0], long[-1]), (world.start, world.goal))
        self.assertTrue(all(world.edge_clear(a, b) for a, b in zip(long, long[1:])))
        self.assertLess(
            sum(math.dist(a, b) for a, b in zip(long, long[1:])),
            sum(math.dist(a, b) for a, b in zip(short, short[1:])),
        )

    def test_plan_reserves_tracking_buffer(self):
        bounds = (0.0, 0.0, 6.0, 5.0)
        obstacles = ((2.5, 0.0, 3.5, 3.0),)
        start, goal = (1.0, 1.0), (5.0, 1.0)
        world = PlanningMap(bounds, obstacles, start, goal, 0.50)
        path = plan_path(world)
        planning_world = PlanningMap(bounds, obstacles, start, goal, 0.75)
        self.assertIsNotNone(path)
        self.assertTrue(all(planning_world.edge_clear(a, b) for a, b in zip(path, path[1:])))
        self.assertEqual(world.clearance, 0.50)

    def test_plan_rejects_start_inside_tracking_buffer(self):
        world = PlanningMap((0.0, 0.0, 4.0, 4.0), (), (0.6, 1.0), (3.0, 1.0), 0.50)
        self.assertEqual(plan_path(world, tracking_buffer=0), [world.start, world.goal])
        with self.assertRaisesRegex(ValueError, "tracking buffer.*start"):
            plan_path(world)

    def test_unreachable_goal(self):
        world = PlanningMap(
            (0.0, 0.0, 6.0, 5.0), ((2.5, 0.0, 3.5, 5.0),),
            (1.0, 1.0), (5.0, 1.0), 0.3,
        )
        self.assertIsNone(plan_path(world, iterations=200))

    def test_turning_and_goal(self):
        world = PlanningMap((-1.0, -1.0, 2.0, 2.0), (), (0.0, 0.0), (0.0, 1.0), 0.3)
        follower = self._follower(world)
        vx, vy, yaw = follower.update(world.start, 0.0)
        self.assertAlmostEqual(vx, 0.0)
        self.assertEqual((vy, yaw), (0.0, 0.6))
        self.assertEqual(follower.update((0.0, 0.5), math.pi / 2), (0.35, 0.0, 0.0))
        self.assertEqual(follower.update(world.goal, math.pi / 2), (0.0, 0.0, 0.0))
        self.assertEqual(follower.status, "reached_goal")

    def test_crossed_obstacle_between_observations(self):
        world = PlanningMap(
            (-1.0, -1.0, 4.0, 4.0), ((1.0, 0.0, 2.0, 1.0),),
            (0.0, 0.0), (3.0, 0.0), 0.3,
        )
        follower = self._follower(world)
        self.assertEqual(follower.update(world.goal, 0.0), (0.0, 0.0, 0.0))
        self.assertEqual(follower.status, "clearance_violation")

    @staticmethod
    def _follower(world):
        return PathFollower(
            world, plan_path(world, iterations=900), speed=0.35, yaw_rate=0.6,
            tolerance=0.07, max_tracking_error=0.18,
        )
