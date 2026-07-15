import math
import random

from .env import ContinuousMap


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:
    def __init__(
        self,
        start,
        goal,
        step_len=0.5,
        goal_sample_rate=0.05,
        iter_max=10000,
        env_map=None,
    ):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.env = env_map if env_map is not None else ContinuousMap()
        self.vertex = [self.start]

    def plan(self):
        for _ in range(self.iter_max):
            rnd = self._random_node()
            nearest = self._nearest(rnd)
            new = self._steer(nearest, rnd)
            if new is None:
                continue
            if self.env.is_collision(nearest.x, nearest.y, new.x, new.y):
                continue
            new.parent = nearest
            self.vertex.append(new)
            dist = math.hypot(new.x - self.goal.x, new.y - self.goal.y)
            if dist <= self.step_len and not self.env.is_collision(
                new.x, new.y, self.goal.x, self.goal.y
            ):
                self.goal.parent = new
                return self._extract_path(self.goal)
        return None

    def _random_node(self):
        if random.random() < self.goal_sample_rate:
            return Node(self.goal.x, self.goal.y)
        x_min, x_max = self.env.x_range
        y_min, y_max = self.env.y_range
        return Node(
            random.uniform(x_min + 0.5, x_max - 0.5),
            random.uniform(y_min + 0.5, y_max - 0.5),
        )

    def _nearest(self, node):
        return min(
            self.vertex,
            key=lambda n: math.hypot(n.x - node.x, n.y - node.y),
        )

    def _steer(self, from_node, to_node):
        dist = math.hypot(to_node.x - from_node.x, to_node.y - from_node.y)
        if dist == 0:
            return None
        step = min(self.step_len, dist)
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        return Node(
            from_node.x + step * math.cos(theta),
            from_node.y + step * math.sin(theta),
        )

    def _extract_path(self, end_node):
        path = []
        node = end_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path
