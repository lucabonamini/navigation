#pragma once

#include "model.h"

namespace model {
class Unicycle : public RobotModel {
    public:
    Unicycle(const double &frequency);
    ~Unicycle(){};
    void updateState(types::State &state, const types::Controls &controls);
    double calcTrackError(const types::State &state, const double &ref_x, const double &ref_y);
    private:
    double frequency_ = 0.0;
};
} // namespace model