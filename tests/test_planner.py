"""
Unit tests – robotaksi_planning A* Global Planner
"""
import math
import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'robotaksi_planning'))

from robotaksi_planning.global_planner import AStarPlanner


class TestAStarPlanner:
    def setup_method(self):
        self.planner = AStarPlanner(grid_size=(100, 100))

    def test_plan_returns_list(self):
        path = self.planner.plan((0.0, 0.0), (10.0, 10.0))
        assert isinstance(path, list)
        assert len(path) > 0

    def test_start_in_path(self):
        path = self.planner.plan((0.0, 0.0), (5.0, 0.0))
        # First point should be near start
        assert math.hypot(path[0][0], path[0][1]) < 1.0

    def test_goal_in_path(self):
        path = self.planner.plan((0.0, 0.0), (5.0, 0.0))
        end = path[-1]
        assert math.hypot(end[0] - 5.0, end[1] - 0.0) < 1.0

    def test_monotone_progress(self):
        path = self.planner.plan((0.0, 0.0), (10.0, 0.0))
        # X distance to goal should decrease over the path
        goal = (10.0, 0.0)
        dists = [math.hypot(p[0] - goal[0], p[1] - goal[1]) for p in path]
        # Not required to be strictly monotone, but should end smaller
        assert dists[-1] <= dists[0] + 0.1
