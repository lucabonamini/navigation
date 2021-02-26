#ifndef _PID_H_
#define _PID_H_

#include <eigen3/Eigen/Core>
#include <vector>
#include <optional>
#include <utility>
#include <fstream>
#include <sstream>
#include <iostream>
#include "cubic_spline_planner/cubic_spline_planner.h"

constexpr double dt = 0.1;
constexpr double rw = 0.5; // robot width

struct State {
    double x;
    double y;
    double theta;
    double v;
};

std::pair<int,double> findClosestIndex(const int& index,
    const State state,
    const std::vector<double> x,
    const std::vector<double> y) {
        int closest_index = 0;
        double closest_dist = std::numeric_limits<double>::max();
        auto fx = state.x + rw * std::cos(state.theta);
        auto fy = state.y + rw * std::sin(state.theta);
        for (size_t i = index ; i < x.size(); i++) {
            double g_x = x.at(i);
            double g_y = y.at(i);
            double curr_closest_dist = (fx - g_x)*(fx - g_x) + (fy - g_y)*(fy - g_y);
            if (curr_closest_dist < closest_dist) {
                closest_dist = curr_closest_dist;
                closest_index = i;
            }
        }
        std::array<double,2> front_axle_vec = {-std::cos(state.theta + M_PI_2),-std::sin(state.theta + M_PI_2)};
        double error_from_axle = ((fx-x.at(closest_index))*front_axle_vec.at(0)) + ((fy-y.at(closest_index))*front_axle_vec.at(1));
        return std::make_pair(closest_index,error_from_axle);
}

void normalizeAngle(double& angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return;
}

class RobotModel {
    public:
        RobotModel(State state) : state_(state){};
        void updateState(const double& steer,
            const double& velocity);
        State state_;
    private:
};

void RobotModel::updateState(const double& steer,
    const double& velocity) {
        state_.v = velocity;
        state_.theta += state_.v / rw  * std::tan(steer) * dt;
        normalizeAngle(state_.theta);
        state_.x += state_.v * std::cos(state_.theta) * dt;
        state_.y += state_.v * std::sin(state_.theta) * dt;
}

class Pid {
    public:
        Pid(double Kp, double Ki, double Kd) : Kp_(Kp), Ki_(Ki), Kd_(Kd){};
        double Kp_;
        double Ki_;
        double Kd_;
        double updateError(const double& error);
        double calcError(const State& state,
            const std::pair<int,double>& track_error,
            const std::vector<double>& ryaw);
        int closest_index = 0;
    private:
        double p_err_;
        double i_err_;
        double d_err_ = 0.0;
};



#endif