#pragma once 

namespace control {
class Pid {
    public:
        Pid(double Kp, double Ki, double Kd) : Kp_(Kp), Ki_(Ki), Kd_(Kd){};
        ~Pid(){};
        double Kp_;
        double Ki_;
        double Kd_;
        double calcCommand(const double& value);
    private:
        double p_err_;
        double i_err_;
        double d_err_ = 0.0;
};
} // namespace control