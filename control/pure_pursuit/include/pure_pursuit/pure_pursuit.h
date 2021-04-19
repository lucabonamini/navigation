#pragma once

#include "utils/types.h"
#include "utils/utils.h"
#include <string>

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Input {
    types::State robot_state;
    Point target_point;
    double cte = 0.0;
    double desired_velocity = 0.0;
};

namespace control {
    class PurePursuit {
    public:
    static std::string model_type;
    PurePursuit(){};
    ~PurePursuit(){};
    types::Controls calcCommands(const Input &input) ;
    };

    // class AdaptivePurePursuit : public PurePursuit {
    // public:
    // AdaptivePurePursuit(){};
    // types::Controls calcCommands(const Input &input) override;
    // private:
    // double calcLookaheadDistance(const double &actual_velocity);
    // };
}