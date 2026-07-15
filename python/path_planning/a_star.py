import heapq
import math

from .env import GridMap


class AStar:
    def __init__(self, start, goal, heuristic="euclidean", grid_map=None):
        self.start = start
        self.goal = goal
        self.heuristic_type = heuristic
        self.env = grid_map if grid_map is not None else GridMap()
        self.open_set = []
        self.closed = []
        self.parent = {}
        self.g = {}

    def search(self):
        self.parent[self.start] = self.start
        self.g[self.start] = 0.0
        self.g[self.goal] = math.inf
        heapq.heappush(self.open_set, (self._f(self.start), self.start))

        while self.open_set:
            _, current = heapq.heappop(self.open_set)
            self.closed.append(current)

            if current == self.goal:
                break

            for neighbor in self._neighbors(current):
                tentative = self.g[current] + self._cost(current, neighbor)
                if neighbor not in self.g:
                    self.g[neighbor] = math.inf
                if tentative < self.g[neighbor]:
                    self.g[neighbor] = tentative
                    self.parent[neighbor] = current
                    heapq.heappush(self.open_set, (self._f(neighbor), neighbor))

        if self.goal not in self.parent:
            return [], self.closed
        return self._extract_path(), self.closed

    def _neighbors(self, node):
        result = []
        for dx, dy in self.env.motions():
            nxt = (node[0] + dx, node[1] + dy)
            if self.env.in_bounds(nxt) and not self.env.is_obstacle(nxt):
                if not self._is_diagonal_blocked(node, nxt):
                    result.append(nxt)
        return result

    def _is_diagonal_blocked(self, a, b):
        if a[0] == b[0] or a[1] == b[1]:
            return False
        if b[0] - a[0] == a[1] - b[1]:
            s1 = (min(a[0], b[0]), min(a[1], b[1]))
            s2 = (max(a[0], b[0]), max(a[1], b[1]))
        else:
            s1 = (min(a[0], b[0]), max(a[1], b[1]))
            s2 = (max(a[0], b[0]), min(a[1], b[1]))
        return self.env.is_obstacle(s1) or self.env.is_obstacle(s2)

    def _cost(self, a, b):
        if self.env.is_obstacle(a) or self.env.is_obstacle(b):
            return math.inf
        if self._is_diagonal_blocked(a, b):
            return math.inf
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def _f(self, node):
        return self.g[node] + self._heuristic(node)

    def _heuristic(self, node):
        if self.heuristic_type == "manhattan":
            return abs(self.goal[0] - node[0]) + abs(self.goal[1] - node[1])
        return math.hypot(self.goal[0] - node[0], self.goal[1] - node[1])

    def _extract_path(self):
        path = [self.goal]
        node = self.goal
        while node != self.start:
            node = self.parent[node]
            path.append(node)
        path.reverse()
        return path
