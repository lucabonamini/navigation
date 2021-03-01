#pragma once

#include "utils/types.h"
#include "utils/utils.h"
#include <math.h>

namespace model {
class RobotModel {
    public:
    RobotModel(){};
    ~RobotModel(){};
    virtual void updateState(State &state, const Controls &controls) = 0;
    virtual double calcTrackError(const State &state, const double &ref_x, const double &ref_y) = 0;
};
} // namespace model