#include "stanley_steer_control/stanley_steer_control.h"

using namespace std;

namespace control {
double StanleySteerControl::calcCommand(const State &state,
    const double &cte,
    const double &ref_yaw) {
        double theta_e = ref_yaw - state.yaw;
        utils::normalizeAngle(theta_e);
        double theta_d = atan2(0.5 * cte, state.v);
        double steer = theta_e + theta_d;
        return steer;
}
} // namespace control