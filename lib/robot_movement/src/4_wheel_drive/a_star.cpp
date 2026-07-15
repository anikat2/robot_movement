#include "a_star.h"
#include <string.h>

static const int8_t kDx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
static const int8_t kDy[8] = {0, 0, -1, 1, -1, 1, -1, 1};

AStarPlanner::AStarPlanner()
    : _w(0), _h(0), _path_len(0), _open_n(0) {
    clearObstacles();
}

void AStarPlanner::init(uint8_t width, uint8_t height) {
    if (width == 0 || height == 0) {
        _w = 0;
        _h = 0;
        return;
    }
    if (width > RM_ASTAR_MAX_W) {
        width = RM_ASTAR_MAX_W;
    }
    if (height > RM_ASTAR_MAX_H) {
        height = RM_ASTAR_MAX_H;
    }
    if ((uint16_t)width * (uint16_t)height > RM_ASTAR_MAX_CELLS) {
        height = RM_ASTAR_MAX_CELLS / width;
    }
    _w = width;
    _h = height;
    clearObstacles();
    _path_len = 0;
}

void AStarPlanner::clearObstacles() {
    memset(_obs, 0, sizeof(_obs));
}

void AStarPlanner::setObstacle(uint8_t x, uint8_t y, bool blocked) {
    if (!inBounds(x, y)) {
        return;
    }
    setBit(_obs, idx(x, y), blocked);
}

bool AStarPlanner::isObstacle(uint8_t x, uint8_t y) const {
    if (!inBounds(x, y)) {
        return true;
    }
    return getBit(_obs, idx(x, y));
}

uint8_t AStarPlanner::pathLength() const {
    return _path_len;
}

bool AStarPlanner::getPathPoint(uint8_t index, uint8_t &x, uint8_t &y) const {
    if (index >= _path_len) {
        return false;
    }
    x = _path_x[index];
    y = _path_y[index];
    return true;
}

uint8_t AStarPlanner::width() const {
    return _w;
}

uint8_t AStarPlanner::height() const {
    return _h;
}

int16_t AStarPlanner::idx(uint8_t x, uint8_t y) const {
    return (int16_t)y * (int16_t)_w + (int16_t)x;
}

bool AStarPlanner::inBounds(uint8_t x, uint8_t y) const {
    return x < _w && y < _h;
}

bool AStarPlanner::getBit(const uint8_t *bits, int16_t i) const {
    return (bits[i >> 3] & (1u << (i & 7))) != 0;
}

void AStarPlanner::setBit(uint8_t *bits, int16_t i, bool v) {
    if (v) {
        bits[i >> 3] |= (uint8_t)(1u << (i & 7));
    } else {
        bits[i >> 3] &= (uint8_t)~(1u << (i & 7));
    }
}

uint16_t AStarPlanner::heuristic(uint8_t x, uint8_t y, uint8_t gx, uint8_t gy) const {
    int16_t dx = (int16_t)gx - (int16_t)x;
    int16_t dy = (int16_t)gy - (int16_t)y;
    if (dx < 0) {
        dx = -dx;
    }
    if (dy < 0) {
        dy = -dy;
    }
    uint16_t mn = dx < dy ? (uint16_t)dx : (uint16_t)dy;
    uint16_t mx = dx > dy ? (uint16_t)dx : (uint16_t)dy;
    return (uint16_t)(10 * mx + 4 * mn);
}

uint16_t AStarPlanner::stepCost(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) const {
    if (x0 != x1 && y0 != y1) {
        return 14;
    }
    return 10;
}

void AStarPlanner::openPush(uint16_t f, int16_t i) {
    if (_open_n >= RM_ASTAR_OPEN_SIZE) {
        uint8_t worst = 0;
        for (uint8_t k = 1; k < _open_n; k++) {
            if (_open_f[k] > _open_f[worst]) {
                worst = k;
            }
        }
        if (f >= _open_f[worst]) {
            return;
        }
        _open_f[worst] = f;
        _open_i[worst] = i;
        return;
    }
    _open_f[_open_n] = f;
    _open_i[_open_n] = i;
    _open_n++;
}

bool AStarPlanner::openPop(int16_t &i) {
    if (_open_n == 0) {
        return false;
    }
    uint8_t best = 0;
    for (uint8_t k = 1; k < _open_n; k++) {
        if (_open_f[k] < _open_f[best]) {
            best = k;
        }
    }
    i = _open_i[best];
    _open_n--;
    _open_f[best] = _open_f[_open_n];
    _open_i[best] = _open_i[_open_n];
    return true;
}

void AStarPlanner::rebuildPath(int16_t goal_i, uint8_t gx, uint8_t gy) {
    int16_t cur = goal_i;
    uint8_t rev_x[RM_PATH_MAX];
    uint8_t rev_y[RM_PATH_MAX];
    uint8_t n = 0;
    while (cur >= 0 && n < RM_PATH_MAX) {
        rev_x[n] = (uint8_t)(cur % _w);
        rev_y[n] = (uint8_t)(cur / _w);
        n++;
        if (_parent[cur] == cur) {
            break;
        }
        cur = _parent[cur];
    }
    _path_len = n;
    for (uint8_t i = 0; i < n; i++) {
        _path_x[i] = rev_x[n - 1 - i];
        _path_y[i] = rev_y[n - 1 - i];
    }
    (void)gx;
    (void)gy;
}

bool AStarPlanner::plan(uint8_t start_x, uint8_t start_y, uint8_t goal_x, uint8_t goal_y) {
    _path_len = 0;
    if (_w == 0 || _h == 0) {
        return false;
    }
    if (!inBounds(start_x, start_y) || !inBounds(goal_x, goal_y)) {
        return false;
    }
    if (isObstacle(start_x, start_y) || isObstacle(goal_x, goal_y)) {
        return false;
    }

    const int16_t cells = (int16_t)_w * (int16_t)_h;
    for (int16_t i = 0; i < cells; i++) {
        _g[i] = 0xFFFF;
        _parent[i] = -1;
    }
    memset(_closed, 0, sizeof(_closed));
    _open_n = 0;

    const int16_t s = idx(start_x, start_y);
    const int16_t g = idx(goal_x, goal_y);
    _g[s] = 0;
    _parent[s] = s;
    openPush(heuristic(start_x, start_y, goal_x, goal_y), s);

    while (_open_n > 0) {
        int16_t cur = -1;
        if (!openPop(cur)) {
            break;
        }
        if (getBit(_closed, cur)) {
            continue;
        }
        setBit(_closed, cur, true);
        if (cur == g) {
            rebuildPath(g, goal_x, goal_y);
            return _path_len > 0;
        }

        const uint8_t cx = (uint8_t)(cur % _w);
        const uint8_t cy = (uint8_t)(cur / _w);
        for (uint8_t m = 0; m < 8; m++) {
            const int16_t nx = (int16_t)cx + kDx[m];
            const int16_t ny = (int16_t)cy + kDy[m];
            if (nx < 0 || ny < 0 || nx >= _w || ny >= _h) {
                continue;
            }
            const uint8_t ux = (uint8_t)nx;
            const uint8_t uy = (uint8_t)ny;
            if (isObstacle(ux, uy)) {
                continue;
            }
            if (kDx[m] != 0 && kDy[m] != 0) {
                if (isObstacle(cx, uy) || isObstacle(ux, cy)) {
                    continue;
                }
            }
            const int16_t ni = idx(ux, uy);
            if (getBit(_closed, ni)) {
                continue;
            }
            const uint16_t tentative = _g[cur] + stepCost(cx, cy, ux, uy);
            if (tentative < _g[ni]) {
                _g[ni] = tentative;
                _parent[ni] = cur;
                openPush(tentative + heuristic(ux, uy, goal_x, goal_y), ni);
            }
        }
    }
    return false;
}
