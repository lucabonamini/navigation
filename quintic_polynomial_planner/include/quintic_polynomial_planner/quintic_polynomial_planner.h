#ifndef _QUINTIC_POLYNOMIAL_PLANNER_H_
#define _QUINTIC_POLYNOMIAL_PLANNER_H_

#include <eigen3/Eigen/Core>
#include <vector>
#include <optional>

namespace QuinticPolynomialPlanner {
    /**
     * @brief Input structure needed by plan function
     * @param sx Start x position [m]
     * @param sy Start y position [m]
     * @param syaw Start yaw angle [rad]
     * @param sv Start velocity [m/s]
     * @param sa Start acceleration [m/ss]
     * @param gx Goal x position [m]
     * @param gy Goal y position [m]
     * @param gyaw Goal yaw angle [rad]
     * @param gv Goal velocity [m/s]
     * @param ga Goal acceleration [m/ss]
     * @param min_t Minimum time to goal [s]
     * @param max_t Maximum time to goal [s]
     * @param dt Time tick [s]
     * @param max_accel Maximum acceleration [m/ss]
     * @param max_jerk Maximum jerk [m/sss]
     */
    struct Input {
        double sx;
        double sy;
        double syaw = 0.0;
        double sv = 0.0;
        double sa = 0.0;
        double gx;
        double gy;
        double gyaw = 0.0;
        double gv = 0.0;
        double ga = 0.0;
        double min_t;
        double max_t;
        double dt = 0.1;
        double max_accel = 1.0;
        double max_jerk = 1.0;
    };
    /**
     * @brief Output structure of plan function
     * @param time Time result
     * @param rx x position result vector
     * @param ry y position result vector
     * @param ryaw yaw angle result vector
     * @param rv velocity result vector
     * @param ra acceleration result vector
     * @param rj jerk result vector
     */
    struct Output {
        std::vector<double> time;
        std::vector<double> rx;
        std::vector<double> ry;
        std::vector<double> ryaw;
        std::vector<double> rv;
        std::vector<double> ra;
        std::vector<double> rj;
    };
    /**
     * @brief Calc quintic polynomial path
     * @param input List of start and end conditions
     * @return Quinti polynomial path
     */
    std::optional<Output> plan(const Input& input);
} // QuinticPolynomialPlanner

/**
 * @brief Class for Quintic Polynomial calculation
 * @param xs_ Start position
 * @param vxs_ Start velocity
 * @param axs_ Start acceleration
 * @param xe_ End desired position
 * @param vxe_ End desired velocity
 * @param axe_ End desired acceleration
 * @param T Time
 */
class QuinticPolynomial {
    public:
    // current parameter at t=0
    double xs;
    double vxs;
    double axs;
    // parameters at target t=t_j
    double xe;
    double vxe;
    double axe;

    // function parameters
    double a0, a1, a2, a3, a4, a5;

    QuinticPolynomial(){};
    QuinticPolynomial(double xs_, double vxs_, double axs_, double xe_, double vxe_, double axe_, double T);

    double calc_point(double t) {
        return a0 + a1*t + a2*std::pow(t, 2) + a3*std::pow(t, 3) + a4*std::pow(t, 4) + a5*std::pow(t, 5);
    };
    double calc_first_derivative(double t) {
        return a1 + 2*a2*t + 3*a3*std::pow(t, 2) + 4*a4*std::pow(t, 3) + 5*a5*std::pow(t, 4);
    };
    double calc_second_derivative(double t) {
        return 2*a2 + 6*a3*t + 12*a4*std::pow(t, 2) + 20*a5*std::pow(t, 3);
    };
    double calc_third_derivative(double t) {
        return 6*a3 + 24*a4*t + 60*a5*std::pow(t, 2);
    };
};


#endif