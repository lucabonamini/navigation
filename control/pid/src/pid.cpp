#include "pid/pid.h"

namespace control {
double Pid::calcCommand(const double& value) {
    d_err_ = value - p_err_;
    p_err_ = value;
    i_err_ += value;
    double cmd = (Kp_ * p_err_ + Kd_ * d_err_ + Ki_ * i_err_);
    return cmd;
}
} // namespace control