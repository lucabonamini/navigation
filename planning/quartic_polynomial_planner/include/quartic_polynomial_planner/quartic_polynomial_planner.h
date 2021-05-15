#pragma once

#include <eigen3/Eigen/Core>
#include <vector>
#include <optional>

namespace QuarticPolynomialPlanner {

} // QuarticPolynomialPlanner

/**
 * @brief Class for Quartic Polynomial calculation
 * @param xs_ Start position
 * @param vxs_ Start velocity
 * @param axs_ Start acceleration
 * @param vxe_ End desired velocity
 * @param axe_ End desired acceleration
 * @param T Time
 */
class QuarticPolynomial {
    public:
    // current parameter at t=0
    double xs = 0.0;
    double vxs = 0.0;
    double axs = 0.0;
    // parameters at target t=t_j
    double vxe = 0.0;
    double axe = 0.0;
    // function parameters
    double a0 = 0.0;
    double a1 = 0.0;
    double a2 = 0.0;
    double a3 = 0.0;
    double a4 = 0.0;

    QuarticPolynomial(){};
    QuarticPolynomial(
        double xs_, double vxs_, double axs_, double vxe_, double axe_, double T);

    double calc_point(double t) {
        return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
            a4 * std::pow(t, 4);
    };
    double calc_first_derivative(double t) {
        return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3);
    };
    double calc_second_derivative(double t) {
        return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2);
    };
    double calc_third_derivative(double t) { return 6 * a3 + 24 * a4 * t; };
};