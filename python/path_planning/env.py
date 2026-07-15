import math


class GridMap:
    def __init__(self, width=50, height=30):
        self.width = width
        self.height = height
        self.obs = set()
        self._build_default_obstacles()

    def _build_default_obstacles(self):
        for x in range(self.width):
            self.obs.add((x, 0))
            self.obs.add((x, self.height - 1))
        for y in range(self.height):
            self.obs.add((0, y))
            self.obs.add((self.width - 1, y))

        for x in range(10, 21):
            self.obs.add((x, 15))
        for y in range(5, 16):
            self.obs.add((20, y))
        for x in range(30, 41):
            self.obs.add((x, 10))
        for y in range(10, 21):
            self.obs.add((35, y))
        for x in range(25, 31):
            for y in range(20, 26):
                self.obs.add((x, y))

    def in_bounds(self, node):
        x, y = node
        return 0 <= x < self.width and 0 <= y < self.height

    def is_obstacle(self, node):
        return node in self.obs

    def motions(self):
        return [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]


class ContinuousMap:
    def __init__(self, x_range=(0, 50), y_range=(0, 30)):
        self.x_range = x_range
        self.y_range = y_range
        self.obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30],
        ]
        self.obs_rectangle = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2],
        ]
        self.obs_circle = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3],
        ]

    def is_inside_obs(self, x, y, delta=0.5):
        for ox, oy, w, h in self.obs_boundary:
            if ox - delta <= x <= ox + w + delta and oy - delta <= y <= oy + h + delta:
                return True
        for ox, oy, w, h in self.obs_rectangle:
            if ox - delta <= x <= ox + w + delta and oy - delta <= y <= oy + h + delta:
                return True
        for ox, oy, r in self.obs_circle:
            if math.hypot(x - ox, y - oy) <= r + delta:
                return True
        return False

    def is_collision(self, x1, y1, x2, y2, delta=0.5):
        if self.is_inside_obs(x1, y1, delta) or self.is_inside_obs(x2, y2, delta):
            return True
        length = math.hypot(x2 - x1, y2 - y1)
        if length == 0:
            return self.is_inside_obs(x1, y1, delta)
        steps = max(int(length / 0.5), 1)
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if self.is_inside_obs(x, y, delta):
                return True
        return False
