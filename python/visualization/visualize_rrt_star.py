import os
import sys

import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from path_planning.env import ContinuousMap
from path_planning.rrt_star import RRTStar
from visualization.plotting import plot_continuous, show


def main():
    start = (2, 2)
    goal = (49, 24)
    env_map = ContinuousMap()
    planner = RRTStar(
        start,
        goal,
        step_len=0.5,
        goal_sample_rate=0.1,
        search_radius=5.0,
        iter_max=5000,
        env_map=env_map,
    )
    path = planner.plan()

    fig, ax = plt.subplots(figsize=(10, 6))
    plot_continuous(ax, env_map, start, goal, path=path, vertices=planner.vertex)
    if path:
        show(f"RRT*  |  path nodes={len(path)}  tree size={len(planner.vertex)}")
    else:
        show("RRT*  |  no path found")


if __name__ == "__main__":
    main()
