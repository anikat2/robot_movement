#include "rrt.h"
#include <math.h>

RRTPlanner::RRTPlanner()
    : _x_min(0), _x_max(10), _y_min(0), _y_max(10),
      _step_len(0.5f), _goal_sample_rate(0.05f), _iter_max(500),
      _node_count(0), _path_len(0), _circle_count(0), _rect_count(0),
      _goal_x(0), _goal_y(0) {}

void RRTPlanner::init(float x_min, float x_max, float y_min, float y_max,
                      float step_len, float goal_sample_rate, uint16_t iter_max) {
    _x_min = x_min;
    _x_max = x_max;
    _y_min = y_min;
    _y_max = y_max;
    _step_len = step_len;
    _goal_sample_rate = goal_sample_rate;
    _iter_max = iter_max;
    _node_count = 0;
    _path_len = 0;
}

void RRTPlanner::clearObstacles() {
    _circle_count = 0;
    _rect_count = 0;
}

bool RRTPlanner::addCircleObstacle(float x, float y, float r) {
    if (_circle_count >= RM_MAX_CIRCLE_OBS) {
        return false;
    }
    _circles[_circle_count].x = x;
    _circles[_circle_count].y = y;
    _circles[_circle_count].r = r;
    _circle_count++;
    return true;
}

bool RRTPlanner::addRectObstacle(float x, float y, float w, float h) {
    if (_rect_count >= RM_MAX_RECT_OBS) {
        return false;
    }
    _rects[_rect_count].x = x;
    _rects[_rect_count].y = y;
    _rects[_rect_count].w = w;
    _rects[_rect_count].h = h;
    _rect_count++;
    return true;
}

uint8_t RRTPlanner::pathLength() const {
    return _path_len;
}

bool RRTPlanner::getPathPoint(uint8_t index, float &x, float &y) const {
    if (index >= _path_len) {
        return false;
    }
    x = _path_x[index];
    y = _path_y[index];
    return true;
}

uint8_t RRTPlanner::nodeCount() const {
    return _node_count;
}

float RRTPlanner::dist(float x1, float y1, float x2, float y2) const {
    const float dx = x2 - x1;
    const float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

bool RRTPlanner::insideObstacle(float x, float y) const {
    for (uint8_t i = 0; i < _circle_count; i++) {
        if (dist(x, y, _circles[i].x, _circles[i].y) <= _circles[i].r) {
            return true;
        }
    }
    for (uint8_t i = 0; i < _rect_count; i++) {
        if (x >= _rects[i].x && x <= _rects[i].x + _rects[i].w &&
            y >= _rects[i].y && y <= _rects[i].y + _rects[i].h) {
            return true;
        }
    }
    return false;
}

bool RRTPlanner::collisionFree(float x1, float y1, float x2, float y2) const {
    if (insideObstacle(x1, y1) || insideObstacle(x2, y2)) {
        return false;
    }
    const float length = dist(x1, y1, x2, y2);
    if (length < 1e-6f) {
        return !insideObstacle(x1, y1);
    }
    const uint8_t steps = (uint8_t)constrain((int)(length / (_step_len * 0.5f)) + 1, 1, 40);
    for (uint8_t i = 0; i <= steps; i++) {
        const float t = (float)i / (float)steps;
        const float x = x1 + t * (x2 - x1);
        const float y = y1 + t * (y2 - y1);
        if (insideObstacle(x, y)) {
            return false;
        }
    }
    return true;
}

int16_t RRTPlanner::nearest(float x, float y) const {
    int16_t best = 0;
    float best_d = dist(_nodes[0].x, _nodes[0].y, x, y);
    for (uint8_t i = 1; i < _node_count; i++) {
        const float d = dist(_nodes[i].x, _nodes[i].y, x, y);
        if (d < best_d) {
            best_d = d;
            best = (int16_t)i;
        }
    }
    return best;
}

bool RRTPlanner::steer(float from_x, float from_y, float to_x, float to_y,
                       float &out_x, float &out_y) const {
    const float d = dist(from_x, from_y, to_x, to_y);
    if (d < 1e-6f) {
        return false;
    }
    const float step = d < _step_len ? d : _step_len;
    out_x = from_x + (to_x - from_x) * (step / d);
    out_y = from_y + (to_y - from_y) * (step / d);
    return true;
}

void RRTPlanner::sample(float &x, float &y) {
    if ((float)random(0, 10000) / 10000.0f < _goal_sample_rate) {
        x = _goal_x;
        y = _goal_y;
        return;
    }
    x = _x_min + ((float)random(0, 10000) / 10000.0f) * (_x_max - _x_min);
    y = _y_min + ((float)random(0, 10000) / 10000.0f) * (_y_max - _y_min);
}

void RRTPlanner::extractPath(int16_t end_idx) {
    float rev_x[RM_PATH_MAX];
    float rev_y[RM_PATH_MAX];
    uint8_t n = 0;
    int16_t cur = end_idx;
    while (cur >= 0 && n < RM_PATH_MAX) {
        rev_x[n] = _nodes[cur].x;
        rev_y[n] = _nodes[cur].y;
        n++;
        cur = _nodes[cur].parent;
    }
    _path_len = n;
    for (uint8_t i = 0; i < n; i++) {
        _path_x[i] = rev_x[n - 1 - i];
        _path_y[i] = rev_y[n - 1 - i];
    }
}

bool RRTPlanner::plan(float start_x, float start_y, float goal_x, float goal_y) {
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
        const uint8_t new_i = _node_count;
        _nodes[new_i].x = nx;
        _nodes[new_i].y = ny;
        _nodes[new_i].parent = near_i;
        _node_count++;

        if (dist(nx, ny, goal_x, goal_y) <= _step_len &&
            collisionFree(nx, ny, goal_x, goal_y)) {
            if (_node_count < RM_RRT_MAX_NODES) {
                _nodes[_node_count].x = goal_x;
                _nodes[_node_count].y = goal_y;
                _nodes[_node_count].parent = (int16_t)new_i;
                extractPath((int16_t)_node_count);
                _node_count++;
            } else {
                extractPath((int16_t)new_i);
            }
            return _path_len > 0;
        }
    }
    return false;
}
