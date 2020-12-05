#ifndef _PID_H_
#define _PID_H_

#include <eigen3/Eigen/Core>
#include <vector>
#include <optional>
#include "cubic_spline_planner/cubic_spline_planner.h"

constexpr double dt = 0.1;
constexpr double rw = 0.5; // robot width

struct State {
    double x;
    double y;
    double theta;
    double v;
};

void findClosestIndex(int& index,
    const State state,
    const std::vector<double> x,
    const std::vector<double> y) {
        double closest_dist = std::numeric_limits<double>::max();
        for (size_t i = index ; i < x.size(); i++) {
            double g_x = x.at(i);
            double g_y = y.at(i);
            double curr_closest_dist = (state.x - g_x)*(state.x - g_x) + (state.y - g_y)*(state.y - g_y);
            if (curr_closest_dist < closest_dist) {
                closest_dist = curr_closest_dist;
                index = i;
            }
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
            const double& closest_x,
            const double& closest_y);
        int closest_index = 0;
    private:
        double p_err_;
        double i_err_;
        double d_err_ = 0.0;
};



#endif