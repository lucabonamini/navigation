#pragma once

#include "model.h"

namespace model {
class DynamicUnicycle : public RobotModel {
    public:
    DynamicUnicycle(const double &frequency);
    ~DynamicUnicycle(){};
    void updateState(State &state, const Controls &controls) override;
    double calcTrackError(const State &state, const double &ref_x, const double &ref_y) override;
    private:
    double frequency_ = 0.0;
};
} // namespace model