#include "frenet/frenet.h"
#include "matplotlibcpp.h"
#include <iostream>

constexpr int MAX_ITERATION = 5000;
constexpr double TARGET_SPEED = 0.5; // m/s

int main() {
    std::vector<double> wx {-2.5,0.0,2.5,5.0,7.5,3.0,-1.0};
    std::vector<double> wy {0.7,-6.0,5.0,6.5,0.0,5.0,-2.0};

    CubicSplinePlanner::Spline2D csp(wx,wy);
    std::vector<double> rx,ry,ryaw;
    for (double i=0; i<csp.s.back(); i+=0.1) {
        auto p = csp.calc_position(i);
        rx.push_back(p.at(0));
        ry.push_back(p.at(1));
        ryaw.push_back(csp.calc_yaw(i));
    }

    int iteration = 0;

    Input input;
    input.csp = &csp;
    input.robot_state = RobotState{
            .x=0.0,
            .y=0.0,
            .yaw=0.0,
            .s_long=0.0,
            .v_long=1.0,
            .a_long=0.0,
            .s_lat=0.0,
            .v_lat=0.0,
            .a_lat=0.0};

    std::vector<RobotState> states;
    while (iteration < MAX_ITERATION) {
        input.target_speed = TARGET_SPEED;
        auto best_path = Frenet::planFrenetPath(input);
        auto dist_from_goal = std::sqrt((best_path->x.at(0) - rx.back())*(best_path->x.at(0) - rx.back()) +
            (best_path->y.at(0) - ry.back())*(best_path->y.at(0) - ry.back()));

        std::cout << "dist_from_goal: " << dist_from_goal << std::endl;

        if (dist_from_goal < 0.3) {
            std::cout << "==== GOAL ====" << std::endl;
            break;
        }
        input.robot_state = RobotState{
            .x=best_path->x.at(1),
            .y=best_path->y.at(1),
            .yaw=best_path->yaw.at(1),
            .s_long=best_path->s.at(1),
            .v_long=best_path->s_d.at(1),
            .a_long=best_path->s_dd.at(1),
            .s_lat=best_path->d.at(1),
            .v_lat=best_path->d_d.at(1),
            .a_lat=best_path->d_dd.at(1)};
        iteration++;
        states.push_back(input.robot_state);
    }
    std::vector<double> state_x,state_y;
    for (auto s:states) {
        state_x.push_back(s.x);
        state_y.push_back(s.y);
    }
    matplotlibcpp::figure();
    matplotlibcpp::plot(rx,ry,"-k");
    matplotlibcpp::plot(wx,wy,"ob");
    matplotlibcpp::plot(state_x,state_y,"xr");
    matplotlibcpp::show();
    return 0;
}