#include "pid.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

double Pid::updateError(const double& error) {
    d_err_ = error - p_err_;
    p_err_ = error;
    i_err_ += error;
    double steer = (Kp_ * p_err_ + Kd_ * d_err_ + Ki_ * i_err_);
    return steer;
}

double Pid::calcError(const State& state,
    const double& closest_x,
    const double& closest_y) {
        return std::sqrt((state.x - closest_x)*(state.x - closest_x) + (state.y - closest_y)*(state.y - closest_y));
        // return std::atan2((state.y - closest_y),(state.x - closest_x));
}