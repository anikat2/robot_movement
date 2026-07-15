#include "4_wheel_drive.h"

FourWheelDrive::FourWheelDrive(int l_front_pin, int l_back_pin, int r_front_pin,
                               int r_back_pin, int odometry_pin)
    : _l_front_pin(l_front_pin),
      _l_back_pin(l_back_pin),
      _r_front_pin(r_front_pin),
      _r_back_pin(r_back_pin),
      _odometry_pin(odometry_pin),
      _path_source(PATH_NONE),
      _sampler_is_star(false),
      _pid_kp(1.0f),
      _pid_ki(0.0f),
      _pid_kd(0.0f),
      _pure_pursuit_lookahead(0.3f) {
    if (_l_front_pin >= 0) {
        pinMode(_l_front_pin, OUTPUT);
    }
    if (_l_back_pin >= 0) {
        pinMode(_l_back_pin, OUTPUT);
    }
    if (_r_front_pin >= 0) {
        pinMode(_r_front_pin, OUTPUT);
    }
    if (_r_back_pin >= 0) {
        pinMode(_r_back_pin, OUTPUT);
    }
    if (_odometry_pin >= 0) {
        pinMode(_odometry_pin, INPUT);
    }
}

void FourWheelDrive::a_star_init(uint8_t width, uint8_t height) {
    _astar.init(width, height);
    _path_source = PATH_NONE;
}

void FourWheelDrive::set_grid_obstacle(uint8_t x, uint8_t y, bool blocked) {
    _astar.setObstacle(x, y, blocked);
}

void FourWheelDrive::clear_grid_obstacles() {
    _astar.clearObstacles();
}

bool FourWheelDrive::a_star(uint8_t start_x, uint8_t start_y, uint8_t goal_x, uint8_t goal_y) {
    const bool ok = _astar.plan(start_x, start_y, goal_x, goal_y);
    _path_source = ok ? PATH_ASTAR : PATH_NONE;
    return ok;
}

void FourWheelDrive::rrt_init(float x_min, float x_max, float y_min, float y_max,
                              float step_len, float goal_sample_rate, uint16_t iter_max) {
    _sampler.init(x_min, x_max, y_min, y_max, step_len, goal_sample_rate, 2.0f, iter_max);
    _sampler_is_star = false;
    _path_source = PATH_NONE;
}

void FourWheelDrive::rrt_star_init(float x_min, float x_max, float y_min, float y_max,
                                   float step_len, float goal_sample_rate,
                                   float search_radius, uint16_t iter_max) {
    _sampler.init(x_min, x_max, y_min, y_max, step_len, goal_sample_rate, search_radius,
                  iter_max);
    _sampler_is_star = true;
    _path_source = PATH_NONE;
}

void FourWheelDrive::clear_continuous_obstacles() {
    _sampler.clearObstacles();
}

bool FourWheelDrive::add_circle_obstacle(float x, float y, float r) {
    return _sampler.addCircleObstacle(x, y, r);
}

bool FourWheelDrive::add_rect_obstacle(float x, float y, float w, float h) {
    return _sampler.addRectObstacle(x, y, w, h);
}

bool FourWheelDrive::rrt(float start_x, float start_y, float goal_x, float goal_y) {
    const bool found =
        static_cast<RRTPlanner &>(_sampler).plan(start_x, start_y, goal_x, goal_y);
    _sampler_is_star = false;
    _path_source = found ? PATH_RRT : PATH_NONE;
    return found;
}

bool FourWheelDrive::rrt_star(float start_x, float start_y, float goal_x, float goal_y) {
    const bool found = _sampler.plan(start_x, start_y, goal_x, goal_y);
    _sampler_is_star = true;
    _path_source = found ? PATH_RRT_STAR : PATH_NONE;
    return found;
}

uint8_t FourWheelDrive::path_length() const {
    if (_path_source == PATH_ASTAR) {
        return _astar.pathLength();
    }
    if (_path_source == PATH_RRT || _path_source == PATH_RRT_STAR) {
        return _sampler.pathLength();
    }
    return 0;
}

bool FourWheelDrive::get_path_point(uint8_t index, float &x, float &y) const {
    if (_path_source == PATH_ASTAR) {
        uint8_t gx = 0;
        uint8_t gy = 0;
        if (!_astar.getPathPoint(index, gx, gy)) {
            return false;
        }
        x = (float)gx;
        y = (float)gy;
        return true;
    }
    if (_path_source == PATH_RRT || _path_source == PATH_RRT_STAR) {
        return _sampler.getPathPoint(index, x, y);
    }
    return false;
}

bool FourWheelDrive::get_grid_path_point(uint8_t index, uint8_t &x, uint8_t &y) const {
    if (_path_source != PATH_ASTAR) {
        return false;
    }
    return _astar.getPathPoint(index, x, y);
}

void FourWheelDrive::bang_bang_init() {}

void FourWheelDrive::pid_init(float kp, float ki, float kd) {
    _pid_kp = kp;
    _pid_ki = ki;
    _pid_kd = kd;
}

void FourWheelDrive::pure_pursuit_init(float lookahead) {
    _pure_pursuit_lookahead = lookahead;
}
