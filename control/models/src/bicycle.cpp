#include "models/bicycle.h"

using namespace std;

namespace model {
Bicycle::Bicycle(const double &frequency, const double &wb) :
    frequency_(frequency),
    wb_(wb) {}
void Bicycle::updateState(State &state, const Controls &controls) {
    state.v += controls.a * 1.0 / frequency_;
    state.yaw += state.v / wb_ * tan(controls.steer) * 1.0 / frequency_;
    utils::normalizeAngle(state.yaw);
    state.x += state.v * cos(state.yaw) * 1.0 / frequency_;
    state.y += state.v * sin(state.yaw) * 1.0 / frequency_;
}
void Bicycle::calcFrontAxleDist(const State &state) {
    fx_ = state.x + wb_ * cos(state.yaw);
    fy_ = state.y + wb_ * sin(state.yaw);
}
double Bicycle::calcTrackError(const State &state,
    const double &ref_x,
    const double &ref_y) {
        array<double,2> front_axle_vec = {-cos(state.yaw + M_PI_2),-sin(state.yaw + M_PI_2)};
        double error_from_axle = ((fx_-ref_x)*front_axle_vec.at(0)) + ((fy_-ref_y)*front_axle_vec.at(1));
        return error_from_axle;
}

} // namespace model