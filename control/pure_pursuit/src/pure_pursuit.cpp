#include "pure_pursuit/pure_pursuit.h"

namespace control {
    types::Controls PurePursuit::calcCommands(const Input &input) {
        double dist = sqrt(pow((input.target_point.x - input.robot_state.x),2) +
                pow((input.target_point.y - input.robot_state.y),2));
        double desired_yaw = atan2((input.target_point.y - input.robot_state.y),
            (input.target_point.x - input.robot_state.x));
        double diff_orientation = utils::shortest_angular_distance(input.robot_state.yaw,
            desired_yaw);
        double omega = atan2(2 * sin(diff_orientation) / dist, 1.0);
        return types::Controls{.steer = omega, .v = input.desired_velocity, .a = 0.0};
    }
} // namespace control