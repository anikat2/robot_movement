#ifndef RM_RRT_STAR_H
#define RM_RRT_STAR_H

#include "rrt.h"

class RRTStarPlanner : public RRTPlanner {
public:
    RRTStarPlanner();

    void init(float x_min, float x_max, float y_min, float y_max,
              float step_len = 0.5f, float goal_sample_rate = 0.1f,
              float search_radius = 2.0f, uint16_t iter_max = 500);

    bool plan(float start_x, float start_y, float goal_x, float goal_y);

private:
    float _search_radius;

    float pathCost(int16_t idx) const;
    void nearNeighbors(float x, float y, int16_t *out, uint8_t &out_n) const;
    int16_t chooseParent(float x, float y, const int16_t *near, uint8_t near_n) const;
    void rewire(int16_t new_i, const int16_t *near, uint8_t near_n);
    int16_t bestGoalParent() const;
};

#endif
