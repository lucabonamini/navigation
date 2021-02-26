#include "models/bicycle.h"

using namespace std;

namespace model {
Bicycle::Bicycle(const double &frequency, const double &wb) : 
    frequency_(frequency),
    wb_(wb) {};
void Bicycle::updateState(State &state, const Controls &controls) {
    state.v += controls.a * 1.0 / frequency_;
    state.yaw += state.v / wb_ * tan(controls.steer) * 1.0 / frequency_;
    utils::normalizeAngle(state.yaw);
    state.x += state.v * cos(state.yaw) * 1.0 / frequency_;
    state.y += state.v * sin(state.yaw) * 1.0 / frequency_;
}
} // namespace model