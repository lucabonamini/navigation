#pragma once

#include "utils/types.h"
#include "utils/utils.h"
#include <string>

struct Input {
    types::State robot_state;
};

namespace control {
    class PurePursuit {
    public:
    static std::string model_type;
    PurePursuit(const double& target_velocity,
        const double& lookahead_distance,
        const double& resolution,
        const types::Path& path);
    ~PurePursuit(){};
    types::Point calcTargetPoint(const types::State& robot_state);
    types::Controls calculate(const types::Point &target_point,
    const types::State& robot_state);
    types::Controls computeCommands(const Input& input);
    private:
    double target_velocity_ = 0.0;
    double lookahead_distance_ = 0.0;
    double resolution_ = 0.0;
    types::Path path_;
    int closest_index_ = 0;
    };

    // class AdaptivePurePursuit : public PurePursuit {
    // public:
    // AdaptivePurePursuit(){};
    // types::Controls calcCommands(const Input &input) override;
    // private:
    // double calcLookaheadDistance(const double &actual_velocity);
    // };
}