#pragma once

#include "utils/types.h"
#include "utils/utils.h"

namespace model {
class RobotModel {
    public:
    virtual void updateState(types::State &state, const types::Controls &controls) = 0;
    virtual double calcTrackError(const types::State &state, const double &ref_x, const double &ref_y) = 0;
};
} // namespace model