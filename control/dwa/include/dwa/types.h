#pragma once

#include <utils/types.h>

struct Obstacle {
    double x = 0.0;
    double y = 0.0;
};
struct DynamicWindow {
    double min_lin_vel_limit = 0.0;
    double max_lin_vel_limit = 0.0;
    double min_ang_vel_limit = 0.0;
    double max_ang_vel_limit = 0.0;
};
struct Config {
    double max_lin_vel = 0.0;
    double min_lin_vel = 0.0;
    double max_ang_vel = 0.0;
    double max_acc = 0.0;
    double robot_radius = 0.0;
    double max_delta_ang_vel = 0.0;
    double lin_vel_resolution = 0.0;
    double ang_vel_resolution = 0.0;
    double dt = 0.0;
    double prediction_time = 0.0;
    double to_goal_cost_gain = 0.0;
    double lin_vel_cost_gain = 0.0;
};

using Obstacles = std::vector<Obstacle>;