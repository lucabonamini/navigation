#pragma once

#include "model.h"

namespace model {
class Unicycle : public RobotModel {
    public:
    Unicycle(const double &frequency);
    ~Unicycle(){};
    void updateState(State &state, const Controls &controls) override;
    private:
    double frequency_ = 0.0;
};
} // namespace model