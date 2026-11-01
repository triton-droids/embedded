import math

import numpy as np
import pytest

from humanoid_ball_chase.astar import plan


def test_open_grid_has_optimal_diagonal_cost():
    path = plan(np.zeros((10, 10)), 0.1, (-0.5, -0.5),
                (-0.35, -0.35), (0.35, 0.35), radius=0)
    assert len(path) == 8
    assert sum(math.dist(a, b) for a, b in zip(path, path[1:])) == pytest.approx(0.7*math.sqrt(2))


def test_wall_detour_and_closed_wall():
    grid = np.zeros((12, 12))
    grid[:9, 6] = 100
    path = plan(grid, 1, (0, 0), (2.5, 2.5), (9.5, 2.5), radius=0)
    assert path and max(y for x, y in path) >= 9.5
    assert all(grid[int(y), int(x)] == 0 for x, y in path)
    grid[9:, 6] = 100
    assert not plan(grid, 1, (0, 0), (2.5, 2.5), (9.5, 2.5), radius=0)


def test_no_diagonal_corner_cutting():
    grid = np.array([[0, 100], [100, 0]])
    assert not plan(grid, 1, (0, 0), (0.5, 0.5), (1.5, 1.5), radius=0)


def test_inflation_closes_narrow_passage_and_respects_boundaries():
    grid = np.zeros((10, 10))
    grid[:, 3] = grid[:, 5] = 100
    assert plan(grid, 1, (0, 0), (4.5, 2.5), (4.5, 7.5), radius=0.4)
    assert not plan(grid, 1, (0, 0), (4.5, 2.5), (4.5, 7.5), radius=0.6)
    assert not plan(np.zeros((10, 10)), 1, (0, 0), (0.5, 0.5), (5.5, 5.5), radius=0.6)


@pytest.mark.parametrize('start,goal', [((-1, 1), (2, 2)), ((1, 1), (10, 2))])
def test_outside_map(start, goal):
    assert not plan(np.zeros((10, 10)), 1, (0, 0), start, goal, radius=0)


def test_unknown_and_occupied_endpoints_and_search_budget():
    grid = np.zeros((10, 10))
    grid[2, 2] = -1
    assert not plan(grid, 1, (0, 0), (1, 1), (2, 2), radius=0)
    grid[2, 2] = 100
    assert not plan(grid, 1, (0, 0), (2, 2), (5, 5), radius=0)
    assert not plan(grid, 1, (0, 0), (0, 0), (9, 9), radius=0, max_expansions=1)
    assert len(plan(grid, 1, (0, 0), (1.1, 1.1), (1.2, 1.2), radius=0)) == 1


@pytest.mark.parametrize('resolution,radius', [(0, 0), (1, -1), (float('nan'), 0)])
def test_invalid_geometry(resolution, radius):
    with pytest.raises(ValueError):
        plan(np.zeros((2, 2)), resolution, (0, 0), (0, 0), (1, 1), radius)
