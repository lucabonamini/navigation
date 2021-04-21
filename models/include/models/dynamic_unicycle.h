#pragma once

#include "model.h"

namespace model {
class DynamicUnicycle : public RobotModel {
    public:
    DynamicUnicycle(const double &frequency);
    ~DynamicUnicycle(){};
    void updateState(types::State &state, const types::Controls &controls) override;
    double calcTrackError(const types::State &state, const double &ref_x, const double &ref_y) override;
    private:
    double frequency_ = 0.0;
};
} // namespace model