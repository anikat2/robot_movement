#ifndef RM_RRT_H
#define RM_RRT_H

#include <Arduino.h>
#include "path_config.h"

struct RrtCircleObs {
    float x;
    float y;
    float r;
};

struct RrtRectObs {
    float x;
    float y;
    float w;
    float h;
};

class RRTPlanner {
public:
    RRTPlanner();

    void init(float x_min, float x_max, float y_min, float y_max,
              float step_len = 0.5f, float goal_sample_rate = 0.05f,
              uint16_t iter_max = 500);

    void clearObstacles();
    bool addCircleObstacle(float x, float y, float r);
    bool addRectObstacle(float x, float y, float w, float h);

    bool plan(float start_x, float start_y, float goal_x, float goal_y);

    uint8_t pathLength() const;
    bool getPathPoint(uint8_t index, float &x, float &y) const;
    uint8_t nodeCount() const;

protected:
    struct Node {
        float x;
        float y;
        int16_t parent;
    };

    float _x_min;
    float _x_max;
    float _y_min;
    float _y_max;
    float _step_len;
    float _goal_sample_rate;
    uint16_t _iter_max;

    Node _nodes[RM_RRT_MAX_NODES];
    uint8_t _node_count;

    float _path_x[RM_PATH_MAX];
    float _path_y[RM_PATH_MAX];
    uint8_t _path_len;

    RrtCircleObs _circles[RM_MAX_CIRCLE_OBS];
    uint8_t _circle_count;
    RrtRectObs _rects[RM_MAX_RECT_OBS];
    uint8_t _rect_count;

    float _goal_x;
    float _goal_y;

    bool collisionFree(float x1, float y1, float x2, float y2) const;
    bool insideObstacle(float x, float y) const;
    int16_t nearest(float x, float y) const;
    bool steer(float from_x, float from_y, float to_x, float to_y,
               float &out_x, float &out_y) const;
    void sample(float &x, float &y);
    void extractPath(int16_t end_idx);
    float dist(float x1, float y1, float x2, float y2) const;
};

#endif
