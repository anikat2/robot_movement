import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


def plot_grid(ax, grid_map, start, goal, path=None, visited=None):
    ox = [p[0] for p in grid_map.obs]
    oy = [p[1] for p in grid_map.obs]
    ax.plot(ox, oy, "sk", markersize=3)

    if visited:
        vx = [p[0] for p in visited]
        vy = [p[1] for p in visited]
        ax.plot(vx, vy, ".c", markersize=2)

    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, "-r", linewidth=2)

    ax.plot(start[0], start[1], "gs", markersize=8, label="start")
    ax.plot(goal[0], goal[1], "b*", markersize=12, label="goal")
    ax.set_xlim(-1, grid_map.width)
    ax.set_ylim(-1, grid_map.height)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")


def plot_continuous(ax, env_map, start, goal, path=None, vertices=None):
    for ox, oy, w, h in env_map.obs_boundary:
        ax.add_patch(Rectangle((ox, oy), w, h, color="k"))
    for ox, oy, w, h in env_map.obs_rectangle:
        ax.add_patch(Rectangle((ox, oy), w, h, color="gray"))
    for ox, oy, r in env_map.obs_circle:
        ax.add_patch(Circle((ox, oy), r, color="gray"))

    if vertices:
        for node in vertices:
            if node.parent is not None:
                ax.plot(
                    [node.x, node.parent.x],
                    [node.y, node.parent.y],
                    "-g",
                    linewidth=0.5,
                    alpha=0.5,
                )

    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, "-r", linewidth=2)

    ax.plot(start[0], start[1], "gs", markersize=8, label="start")
    ax.plot(goal[0], goal[1], "b*", markersize=12, label="goal")
    ax.set_xlim(env_map.x_range[0], env_map.x_range[1] + 1)
    ax.set_ylim(env_map.y_range[0], env_map.y_range[1] + 1)
    ax.set_aspect("equal")
    ax.legend(loc="upper right")


def show(title="Path Planning"):
    plt.title(title)
    plt.tight_layout()
    plt.show()
