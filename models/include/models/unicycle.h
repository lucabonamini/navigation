#pragma once

#include "model.h"

namespace model {
class Unicycle : public RobotModel {
    public:
    Unicycle(const double &frequency);
    ~Unicycle(){};
    void updateState(State &state, const Controls &controls);
    double calcTrackError(const State &state, const double &ref_x, const double &ref_y);
    private:
    double frequency_ = 0.0;
};
} // namespace model