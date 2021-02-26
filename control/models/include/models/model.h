#pragma once

#include "utils/types.h"
#include "utils/utils.h"
#include <math.h>

namespace model {
class RobotModel {
    public:
    RobotModel(){};
    virtual ~RobotModel(){};
    virtual void updateState(State &state, const Controls &controls) = 0;
};
} // namespace model