#include "pure_pursuit/pure_pursuit.h"

namespace control {
    PurePursuit::PurePursuit(const double& target_velocity,
        const double& lookahead_distance,
        const double& resolution,
        const types::Path& path) :
        target_velocity_(target_velocity),
        lookahead_distance_(lookahead_distance),
        resolution_(resolution),
        path_(path) {}
    types::Point PurePursuit::calcTargetPoint(const types::State& robot_state) {
        utils::findClosestIndex(closest_index_,{robot_state.x,robot_state.y},path_);
        auto id = round(lookahead_distance_ / resolution_) + closest_index_;
        if (id > path_.size() - 1) {
            id = path_.size() - 1;
        }
        return types::Point{path_.at(id).point.x,path_.at(id).point.y};
    }
    types::Controls PurePursuit::calculate(const types::Point &target_point,
        const types::State& robot_state) {
        double dist = sqrt(pow((target_point.x - robot_state.x),2) +
                pow((target_point.y - robot_state.y),2));
        double desired_yaw = atan2((target_point.y - robot_state.y),
            (target_point.x - robot_state.x));
        double diff_orientation = utils::shortestAngularDistance(robot_state.yaw,
            desired_yaw);
        double omega = atan2(2 * sin(diff_orientation) / dist, 1.0);
        return types::Controls{.steer = omega, .v = target_velocity_, .a = 0.0};
    }
    types::Controls PurePursuit::computeCommands(const Input& input) {
        auto target_point = calcTargetPoint(input.robot_state);
        auto controls = calculate(target_point,input.robot_state);
        return controls;
    }

} // namespace control