// Ref: https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
#pragma once

#include "utils/types.h"
#include "utils/utils.h"

using namespace types;

namespace control {
class StanleySteerControl {
    public:
    StanleySteerControl(){};
    ~StanleySteerControl(){};
    double calcCommand(const State &state, const double &cte, const double &ref_yaw);
};
} // namespace control