#include "quartic_polynomial_planner.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

namespace QuarticPolynomialPlanner {

} // QuarticPolynomialPlanner

QuarticPolynomial::QuarticPolynomial(
    double xs_, double vxs_, double axs_, double vxe_, double axe_, double T)
    : xs(xs_)
    , vxs(vxs_)
    , axs(axs_)
    , vxe(vxe_)
    , axe(axe_)
    , a0(xs_)
    , a1(vxs_)
    , a2(axs_ / 2.0) {
  Eigen::Matrix2d A;
  A(0, 0) = 3 * std::pow(T, 2);
  A(0, 1) = 4 * std::pow(T, 3);
  A(1, 0) = 6 * T;
  A(1, 1) = 12 * std::pow(T, 2);
  Eigen::Vector2d B;
  B(0) = vxe - a1 - 2 * a2 * T;
  B(1) = axe - 2 * a2;

  Eigen::Vector2d c_eigen = A.colPivHouseholderQr().solve(B);
  a3 = c_eigen[0];
  a4 = c_eigen[1];
}
