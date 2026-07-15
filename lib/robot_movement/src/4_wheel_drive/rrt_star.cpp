#include "rrt_star.h"
#include <math.h>

RRTStarPlanner::RRTStarPlanner() : RRTPlanner(), _search_radius(2.0f) {}

void RRTStarPlanner::init(float x_min, float x_max, float y_min, float y_max,
                          float step_len, float goal_sample_rate,
                          float search_radius, uint16_t iter_max) {
    RRTPlanner::init(x_min, x_max, y_min, y_max, step_len, goal_sample_rate, iter_max);
    _search_radius = search_radius;
}

float RRTStarPlanner::pathCost(int16_t idx) const {
    float cost = 0.0f;
    int16_t cur = idx;
    while (cur >= 0) {
        const int16_t p = _nodes[cur].parent;
        if (p < 0) {
            break;
        }
        cost += dist(_nodes[cur].x, _nodes[cur].y, _nodes[p].x, _nodes[p].y);
        cur = p;
    }
    return cost;
}

void RRTStarPlanner::nearNeighbors(float x, float y, int16_t *out, uint8_t &out_n) const {
    out_n = 0;
    const float n = (float)_node_count + 1.0f;
    float radius = _search_radius * sqrtf(logf(n) / n);
    if (radius > _step_len * 5.0f) {
        radius = _step_len * 5.0f;
    }
    if (radius < _step_len) {
        radius = _step_len;
    }
    for (uint8_t i = 0; i < _node_count && out_n < 16; i++) {
        if (dist(_nodes[i].x, _nodes[i].y, x, y) <= radius) {
            if (collisionFree(_nodes[i].x, _nodes[i].y, x, y)) {
                out[out_n++] = (int16_t)i;
            }
        }
    }
}

int16_t RRTStarPlanner::chooseParent(float x, float y, const int16_t *near, uint8_t near_n) const {
    int16_t best = nearest(x, y);
    float best_cost = pathCost(best) + dist(_nodes[best].x, _nodes[best].y, x, y);
    for (uint8_t i = 0; i < near_n; i++) {
        const int16_t ni = near[i];
        const float c = pathCost(ni) + dist(_nodes[ni].x, _nodes[ni].y, x, y);
        if (c < best_cost) {
            best_cost = c;
            best = ni;
        }
    }
    return best;
}

void RRTStarPlanner::rewire(int16_t new_i, const int16_t *near, uint8_t near_n) {
    for (uint8_t i = 0; i < near_n; i++) {
        const int16_t ni = near[i];
        if (ni == new_i || ni == _nodes[new_i].parent) {
            continue;
        }
        const float through_new =
            pathCost(new_i) + dist(_nodes[new_i].x, _nodes[new_i].y, _nodes[ni].x, _nodes[ni].y);
        if (through_new < pathCost(ni)) {
            if (collisionFree(_nodes[new_i].x, _nodes[new_i].y, _nodes[ni].x, _nodes[ni].y)) {
                _nodes[ni].parent = new_i;
            }
        }
    }
}

int16_t RRTStarPlanner::bestGoalParent() const {
    int16_t best = -1;
    float best_cost = 1e30f;
    for (uint8_t i = 0; i < _node_count; i++) {
        const float d = dist(_nodes[i].x, _nodes[i].y, _goal_x, _goal_y);
        if (d <= _step_len && collisionFree(_nodes[i].x, _nodes[i].y, _goal_x, _goal_y)) {
            const float c = pathCost((int16_t)i) + d;
            if (c < best_cost) {
                best_cost = c;
                best = (int16_t)i;
            }
        }
    }
    return best;
}

bool RRTStarPlanner::plan(float start_x, float start_y, float goal_x, float goal_y) {
    _path_len = 0;
    _node_count = 0;
    _goal_x = goal_x;
    _goal_y = goal_y;

    if (insideObstacle(start_x, start_y) || insideObstacle(goal_x, goal_y)) {
        return false;
    }

    _nodes[0].x = start_x;
    _nodes[0].y = start_y;
    _nodes[0].parent = -1;
    _node_count = 1;

    for (uint16_t it = 0; it < _iter_max; it++) {
        if (_node_count >= RM_RRT_MAX_NODES) {
            break;
        }
        float rx, ry;
        sample(rx, ry);
        const int16_t near_i = nearest(rx, ry);
        float nx, ny;
        if (!steer(_nodes[near_i].x, _nodes[near_i].y, rx, ry, nx, ny)) {
            continue;
        }
        if (!collisionFree(_nodes[near_i].x, _nodes[near_i].y, nx, ny)) {
            continue;
        }

        int16_t near_list[16];
        uint8_t near_n = 0;
        nearNeighbors(nx, ny, near_list, near_n);
        const int16_t parent = chooseParent(nx, ny, near_list, near_n);
        if (!collisionFree(_nodes[parent].x, _nodes[parent].y, nx, ny)) {
            continue;
        }

        const int16_t new_i = (int16_t)_node_count;
        _nodes[new_i].x = nx;
        _nodes[new_i].y = ny;
        _nodes[new_i].parent = parent;
        _node_count++;
        rewire(new_i, near_list, near_n);
    }

    const int16_t goal_parent = bestGoalParent();
    if (goal_parent < 0) {
        return false;
    }
    if (_node_count < RM_RRT_MAX_NODES) {
        _nodes[_node_count].x = goal_x;
        _nodes[_node_count].y = goal_y;
        _nodes[_node_count].parent = goal_parent;
        extractPath((int16_t)_node_count);
        _node_count++;
    } else {
        extractPath(goal_parent);
    }
    return _path_len > 0;
}
