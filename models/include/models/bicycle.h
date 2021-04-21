#pragma once

#include "model.h"

namespace model {
class Bicycle : public RobotModel {
    public:
    Bicycle(const double &frequency, const double &wb);
    ~Bicycle(){};
    void updateState(types::State &state, const types::Controls &controls) override;
    double calcTrackError(const types::State &state, const double &ref_x, const double &ref_y) override;
    void calcFrontAxleDist(const types::State &state);
    double fx_ = 0.0;
    double fy_ = 0.0;
    private:
    double frequency_ = 0.0;
    double wb_ = 0.0;  // wheel base length
};
} // namespace model