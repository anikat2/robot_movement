import os
import sys

import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from path_planning.a_star import AStar
from path_planning.env import GridMap
from visualization.plotting import plot_grid, show


def main():
    start = (5, 5)
    goal = (45, 25)
    grid = GridMap()
    planner = AStar(start, goal, heuristic="euclidean", grid_map=grid)
    path, visited = planner.search()

    fig, ax = plt.subplots(figsize=(10, 6))
    plot_grid(ax, grid, start, goal, path=path, visited=visited)
    if path:
        show(f"A*  |  path length={len(path)}  visited={len(visited)}")
    else:
        show("A*  |  no path found")


if __name__ == "__main__":
    main()
