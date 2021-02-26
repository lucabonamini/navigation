#pragma once

#include "model.h"

namespace model {
class Bicycle : public RobotModel {
    public:
    Bicycle(const double &frequency, const double &wb){};
    ~Bicycle(){};
    void updateState(State &state, const Controls &controls) override;
    private:
    double frequency_ = 0.0;
    double wb_ = 0.0;  // wheel base length
};
} // namespace model