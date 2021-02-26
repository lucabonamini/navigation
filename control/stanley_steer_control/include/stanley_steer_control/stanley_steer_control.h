// Ref: https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
#pragma once

#include "utils/types.h"
#include "utils/utils.h"
// std::pair<int,double> findClosestIndex(const int& index,
//     const State state,
//     const std::vector<double> x,
//     const std::vector<double> y) {
//         int closest_index = 0;
//         double closest_dist = std::numeric_limits<double>::max();
//         auto fx = state.x + rw * std::cos(state.theta);
//         auto fy = state.y + rw * std::sin(state.theta);
//         for (size_t i = index ; i < x.size(); i++) {
//             double g_x = x.at(i);
//             double g_y = y.at(i);
//             double curr_closest_dist = (fx - g_x)*(fx - g_x) + (fy - g_y)*(fy - g_y);
//             if (curr_closest_dist < closest_dist) {
//                 closest_dist = curr_closest_dist;
//                 closest_index = i;
//             }
//         }
//         std::array<double,2> front_axle_vec = {-std::cos(state.theta + M_PI_2),-std::sin(state.theta + M_PI_2)};
//         double error_from_axle = ((fx-x.at(closest_index))*front_axle_vec.at(0)) + ((fy-y.at(closest_index))*front_axle_vec.at(1));
//         return std::make_pair(closest_index,error_from_axle);
// }

// double Pid::calcError(const State& state,
//     const std::pair<int,double>& track_error,
//     const std::vector<double>& ryaw) {
//         double theta_e = ryaw.at(track_error.first)-state.theta;
//         normalizeAngle(theta_e);
//         double theta_d = std::atan2(0.5*track_error.second,state.v);
//         double steer = theta_e+theta_d;
//         // return(updateError(steer));
//         return(steer);
// }


namespace control {
class StanleySteerControl {
    public:
    StanleySteerControl(){};
    ~StanleySteerControl(){};
    double calcCommand(const State &state, const double &cte, const double &ref_yaw);
};
} // namespace control