#include "models/unicycle.h"

using namespace std;

namespace model {
Unicycle::Unicycle(const double &frequency) : frequency_(frequency) {};
void Unicycle::updateState(State &state, const Controls &controls) {
    state.v = controls.v;
    state.yaw += controls.steer * 1.0 / frequency_;
    utils::normalizeAngle(state.yaw);
    state.x += state.v * cos(state.yaw) * 1.0 / frequency_;
    state.y += state.v * sin(state.yaw) * 1.0 / frequency_;
}
} //  namespace model