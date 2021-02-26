#include "quintic_polynomial_planner.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

namespace QuinticPolynomialPlanner {
    std::optional<Output> plan(const Input& input) {
        double vxs = input.sv * std::cos(input.syaw);
        double vys = input.sv * std::sin(input.syaw);
        double vxg = input.gv * std::cos(input.gyaw);
        double vyg = input.gv * std::sin(input.gyaw);
        double axs = input.sa * std::cos(input.syaw);
        double ays = input.sa * std::sin(input.syaw);
        double axg = input.ga * std::cos(input.gyaw);
        double ayg = input.ga * std::sin(input.gyaw);

        for (int T = input.min_t; T < input.max_t; T += input.min_t) {
            QuinticPolynomial xqp(input.sx, vxs, axs, input.gx, vxg, axg, T);
            QuinticPolynomial yqp(input.sy, vys, ays, input.gy, vyg, ayg, T);
            std::optional<Output> output = std::make_optional<Output>();

            for (double t = 0; t < T+input.dt; t += input.dt) {
                output->time.push_back(t);
                output->rx.push_back(xqp.calc_point(t));
                output->ry.push_back(yqp.calc_point(t));

                double vx = xqp.calc_first_derivative(t);
                double vy = yqp.calc_first_derivative(t);
                double v = sqrt(pow(vx,2) + pow(vy,2));
                double yaw = atan2(vy,vx);
                output->rv.push_back(v);
                output->ryaw.push_back(yaw);

                double ax = xqp.calc_second_derivative(t);
                double ay = yqp.calc_second_derivative(t);
                double a = sqrt(pow(ax,2)+pow(ay,2));
                int v_size = output->rv.size();
                if (v_size >= 2 && output->rv.back()-output->rv.at(v_size-1) < 0.0) {
                    a *= -1;
                }
                output->ra.push_back(a);

                double jx = xqp.calc_third_derivative(t);
                double jy = yqp.calc_third_derivative(t);
                double j = sqrt(pow(jx,2)+pow(jy,2));
                int a_size = output->rv.size();
                if(a_size >= 2 && output->rv.back()-output->rv.at(a_size-1) < 0.0) {
                    j *= -1;
                }
                output->rj.push_back(j);
            }
            if (*std::max_element(output->ra.begin(),output->ra.end()) <= input.max_accel &&
                *std::max_element(output->rj.begin(),output->rj.end()) <= input.max_jerk) {
                    return output;
            }
        }
        return std::nullopt;
    }
} // QuinticPolynomialPlanner

QuinticPolynomial::QuinticPolynomial(double xs_, double vxs_, double axs_, double xe_, double vxe_, double axe_, double T): xs(xs_), vxs(vxs_), axs(axs_), xe(xe_), vxe(vxe_), axe(axe_), a0(xs_), a1(vxs_), a2(axs_/2.0){
    Eigen::Matrix3d A;
    A << std::pow(T, 3),		std::pow(T, 4),     std::pow(T, 5),
        3*std::pow(T, 2),  4*std::pow(T, 3),   5*std::pow(T, 4),
        6*T,								12*std::pow(T, 2),  20*std::pow(T, 3);
    Eigen::Vector3d B;
    B<< xe - a0 - a1 * T - a2 * std::pow(T, 2),
        vxe - a1 - 2 * a2 * T,
        axe - 2 * a2;

    Eigen::Vector3d c_eigen = A.colPivHouseholderQr().solve(B);
    a3 = c_eigen[0];
    a4 = c_eigen[1];
    a5 = c_eigen[2];
}
