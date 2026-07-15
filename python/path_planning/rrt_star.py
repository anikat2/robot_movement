import math
import random

from .env import ContinuousMap
from .rrt import Node


class RRTStar:
    def __init__(
        self,
        start,
        goal,
        step_len=0.5,
        goal_sample_rate=0.1,
        search_radius=5.0,
        iter_max=5000,
        env_map=None,
    ):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.env = env_map if env_map is not None else ContinuousMap()
        self.vertex = [self.start]
        self.path = []

    def plan(self):
        for _ in range(self.iter_max):
            rnd = self._random_node()
            nearest = self._nearest(rnd)
            new = self._steer(nearest, rnd)
            if new is None:
                continue
            if self.env.is_collision(nearest.x, nearest.y, new.x, new.y):
                continue

            near_indices = self._near_neighbors(new)
            self._choose_parent(new, near_indices)
            self.vertex.append(new)
            self._rewire(new, near_indices)

        goal_idx = self._best_goal_parent()
        if goal_idx is None:
            return None
        self.path = self._extract_path(self.vertex[goal_idx])
        return self.path

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

    def _near_neighbors(self, node):
        n = len(self.vertex) + 1
        r = min(self.search_radius * math.sqrt(math.log(n) / n), self.step_len * 5)
        indices = []
        for i, v in enumerate(self.vertex):
            if math.hypot(v.x - node.x, v.y - node.y) <= r:
                if not self.env.is_collision(v.x, v.y, node.x, node.y):
                    indices.append(i)
        return indices

    def _cost(self, node):
        cost = 0.0
        current = node
        while current.parent is not None:
            cost += math.hypot(
                current.x - current.parent.x, current.y - current.parent.y
            )
            current = current.parent
        return cost

    def _choose_parent(self, new, near_indices):
        if not near_indices:
            nearest = self._nearest(new)
            new.parent = nearest
            return
        best = None
        best_cost = math.inf
        for i in near_indices:
            candidate = self.vertex[i]
            cost = self._cost(candidate) + math.hypot(
                candidate.x - new.x, candidate.y - new.y
            )
            if cost < best_cost:
                best_cost = cost
                best = candidate
        new.parent = best

    def _rewire(self, new, near_indices):
        for i in near_indices:
            neighbor = self.vertex[i]
            new_cost = self._cost(new) + math.hypot(
                neighbor.x - new.x, neighbor.y - new.y
            )
            if new_cost < self._cost(neighbor):
                if not self.env.is_collision(new.x, new.y, neighbor.x, neighbor.y):
                    neighbor.parent = new

    def _best_goal_parent(self):
        candidates = []
        for i, v in enumerate(self.vertex):
            dist = math.hypot(v.x - self.goal.x, v.y - self.goal.y)
            if dist <= self.step_len:
                if not self.env.is_collision(v.x, v.y, self.goal.x, self.goal.y):
                    candidates.append((self._cost(v) + dist, i))
        if not candidates:
            return None
        candidates.sort(key=lambda c: c[0])
        return candidates[0][1]

    def _extract_path(self, end_node):
        path = [(self.goal.x, self.goal.y)]
        node = end_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path
