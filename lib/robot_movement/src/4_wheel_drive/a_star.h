#ifndef RM_A_STAR_H
#define RM_A_STAR_H

#include <Arduino.h>
#include "path_config.h"

class AStarPlanner {
public:
    AStarPlanner();

    void init(uint8_t width, uint8_t height);
    void clearObstacles();
    void setObstacle(uint8_t x, uint8_t y, bool blocked = true);
    bool isObstacle(uint8_t x, uint8_t y) const;

    bool plan(uint8_t start_x, uint8_t start_y, uint8_t goal_x, uint8_t goal_y);

    uint8_t pathLength() const;
    bool getPathPoint(uint8_t index, uint8_t &x, uint8_t &y) const;
    uint8_t width() const;
    uint8_t height() const;

private:
    uint8_t _w;
    uint8_t _h;
    uint8_t _path_len;
    uint8_t _path_x[RM_PATH_MAX];
    uint8_t _path_y[RM_PATH_MAX];
    uint8_t _obs[(RM_ASTAR_MAX_CELLS + 7) / 8];
    uint16_t _g[RM_ASTAR_MAX_CELLS];
    int16_t _parent[RM_ASTAR_MAX_CELLS];
    uint8_t _closed[(RM_ASTAR_MAX_CELLS + 7) / 8];
    uint16_t _open_f[RM_ASTAR_OPEN_SIZE];
    int16_t _open_i[RM_ASTAR_OPEN_SIZE];
    uint8_t _open_n;

    int16_t idx(uint8_t x, uint8_t y) const;
    bool inBounds(uint8_t x, uint8_t y) const;
    bool getBit(const uint8_t *bits, int16_t i) const;
    void setBit(uint8_t *bits, int16_t i, bool v);
    uint16_t heuristic(uint8_t x, uint8_t y, uint8_t gx, uint8_t gy) const;
    uint16_t stepCost(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) const;
    void openPush(uint16_t f, int16_t i);
    bool openPop(int16_t &i);
    void rebuildPath(int16_t goal_i, uint8_t gx, uint8_t gy);
};

#endif
