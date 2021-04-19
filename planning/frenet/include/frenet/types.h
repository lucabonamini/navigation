#pragma once

#include "cubic_spline_planner/cubic_spline_planner.h"
#include <vector>

struct FrenetPath {
  double costLat = 0.0;
  double costLong = 0.0;
  double costObs = 0.0;
  double costTotal = 0.0;
  double finalTime = 0.0;
  double finalLateralDist = 0.0;

  std::vector<double> t;
  std::vector<double> d;
  std::vector<double> d_d;
  std::vector<double> d_dd;
  std::vector<double> d_ddd;
  std::vector<double> s;
  std::vector<double> s_d;
  std::vector<double> s_dd;
  std::vector<double> s_ddd;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> ds;
  std::vector<double> k;

  double max_speed;
  double max_accel;
  double max_curvature;
};

struct RobotState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double s_long = 0.0;
    double v_long = 0.0;
    double a_long = 0.0;
    double s_lat = 0.0;
    double v_lat = 0.0;
    double a_lat = 0.0;
};

struct Obstacle {
    double x = 0.0;
    double y = 0.0;
};

struct Input {
    RobotState robot_state;
    double target_speed = 0.0;
    std::vector<Obstacle> obstacles;
    CubicSplinePlanner::Spline2D* csp;  // reference path
};

struct Output {
    FrenetPath output_path;
};