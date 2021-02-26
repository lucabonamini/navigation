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
    const std::pair<int,double>& track_error,
    const std::vector<double>& ryaw) {
        double theta_e = ryaw.at(track_error.first)-state.theta;
        normalizeAngle(theta_e);
        double theta_d = std::atan2(0.5*track_error.second,state.v);
        double steer = theta_e+theta_d;
        return(updateError(steer));
}