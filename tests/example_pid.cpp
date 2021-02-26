#include "pid/pid.h"
#include "cubic_spline_planner/cubic_spline_planner.h"
#include "utils/types.h"
#include "utils/utils.h"
#include "matplotlibcpp.h"

constexpr int MAX_TIME = 5000;

int main(int argc, char** argv) {

    std::vector<double> wx {-2.5,0.0,2.5,5.0,7.5,3.0,-1.0};
    std::vector<double> wy {0.7,-6.0,2.0,-4.0,0.0,5.0,-2.0};
    int time = 0;
    std::vector<State> states;

    std::vector<double> rx,ry,ryaw;

    CubicSplinePlanner::Spline2D csp(wx,wy);
    for (double i=0; i<csp.s.back(); i+=0.1) {
        auto p = csp.calc_position(i);
        rx.push_back(p.at(0));
        ry.push_back(p.at(1));
        ryaw.push_back(csp.calc_yaw(i));
    }

    // Initial conditions
    State init;
    init.x = rx.front();
    init.y = ry.front();
    init.yaw = std::atan2((ry.at(1)-ry.at(0)),(rx.at(1)-rx.at(0)));
    init.v = 0.1;
    RobotModel rm(init);
    std::vector<double> offtrack_errors;

    Pid pid(1.26836,5.37114e-07,1.26836);
    while(time < MAX_TIME) {
        double velocity = 0.1;
        auto track_error = findClosestIndex(pid.closest_index,rm.state_,rx,ry);
        pid.closest_index = track_error.first;

        auto steer = pid.calcError(rm.state_,track_error,ryaw);

        if (steer < -30*M_PI/180) {
            steer = -30*M_PI/180;
        } else if (steer > 30*M_PI/180) {
            steer = 30*M_PI/180;
        }
        rm.updateState(steer,velocity);

        auto dist_from_goal = std::sqrt((rm.state_.x - rx.back())*(rm.state_.x - rx.back()) +
            (rm.state_.y - ry.back())*(rm.state_.y - ry.back()));
        double curr_closest_dist = std::sqrt((rm.state_.x - rx.at(track_error.first))*(rm.state_.x - rx.at(track_error.first)) +
            (rm.state_.y - ry.at(track_error.second))*(rm.state_.y - ry.at(track_error.second)));
        offtrack_errors.push_back(curr_closest_dist);

        if (dist_from_goal < 0.1) {
            std::cout << "==== GOAL ====" << std::endl;
            break;
        }
        time++;
        states.push_back(rm.state_);
    }

    std::vector<double> state_x,state_y;
    for (auto state:states) {
        state_x.push_back(state.x);
        state_y.push_back(state.y);
    }
    matplotlibcpp::plot(rx,ry,"-k");
    matplotlibcpp::plot(wx,wy,"ob");
    matplotlibcpp::plot(state_x,state_y,"xr");
    matplotlibcpp::title("PID Controller");
    matplotlibcpp::show();

    return 0;
}
