#ifndef FOUR_WHEEL_DRIVE_H
#define FOUR_WHEEL_DRIVE_H

#include <Arduino.h>
#include "a_star.h"
#include "rrt_star.h"

class FourWheelDrive {
public:
    FourWheelDrive(int l_front_pin, int l_back_pin, int r_front_pin, int r_back_pin,
                   int odometry_pin = -1);

    void a_star_init(uint8_t width, uint8_t height);
    void set_grid_obstacle(uint8_t x, uint8_t y, bool blocked = true);
    void clear_grid_obstacles();
    bool a_star(uint8_t start_x, uint8_t start_y, uint8_t goal_x, uint8_t goal_y);

    void rrt_init(float x_min, float x_max, float y_min, float y_max,
                  float step_len = 0.5f, float goal_sample_rate = 0.05f,
                  uint16_t iter_max = 400);
    void rrt_star_init(float x_min, float x_max, float y_min, float y_max,
                       float step_len = 0.5f, float goal_sample_rate = 0.1f,
                       float search_radius = 2.0f, uint16_t iter_max = 400);
    void clear_continuous_obstacles();
    bool add_circle_obstacle(float x, float y, float r);
    bool add_rect_obstacle(float x, float y, float w, float h);
    bool rrt(float start_x, float start_y, float goal_x, float goal_y);
    bool rrt_star(float start_x, float start_y, float goal_x, float goal_y);

    uint8_t path_length() const;
    bool get_path_point(uint8_t index, float &x, float &y) const;
    bool get_grid_path_point(uint8_t index, uint8_t &x, uint8_t &y) const;

    void bang_bang_init();
    void pid_init(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f);
    void pure_pursuit_init(float lookahead = 0.3f);

private:
    int _l_front_pin;
    int _l_back_pin;
    int _r_front_pin;
    int _r_back_pin;
    int _odometry_pin;

    AStarPlanner _astar;
    RRTStarPlanner _sampler;

    enum PathSource : uint8_t { PATH_NONE = 0, PATH_ASTAR = 1, PATH_RRT = 2, PATH_RRT_STAR = 3 };
    PathSource _path_source;
    bool _sampler_is_star;

    float _pid_kp;
    float _pid_ki;
    float _pid_kd;
    float _pure_pursuit_lookahead;
};

#endif
