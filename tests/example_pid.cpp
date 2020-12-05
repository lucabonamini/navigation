#include "pid/pid.h"
#include "matplotlibcpp.h"

constexpr int MAX_TIME = 20000;

int main(int argc, char** argv) {

    std::vector<double> wx {-2.5,0.0,2.5,5.0,7.5,3.0,-1.0};
    std::vector<double> wy {0.7,-6.0,5.0,6.5,0.0,5.0,-2.0};
    int time = 0;
    std::vector<State> states;

    std::vector<double> rx,ry;

    CubicSplinePlanner::Spline2D csp(wx,wy);
    for (double i=0; i<csp.s.back(); i+=0.1) {
        auto p = csp.calc_position(i);
        rx.push_back(p.at(0));
        ry.push_back(p.at(1));
    }

    // Initial conditions
    State init;
    init.x = rx.front();
    init.y = ry.front();
    init.theta = std::atan2((ry.at(1)-ry.at(0)),(rx.at(1)-rx.at(0)));
    init.v = 0.1;
    RobotModel rm(init);
    std::cout << rm.state_.x << " , "
        << rm.state_.y << " , "
        << rm.state_.theta << " , "
        << rm.state_.v << std::endl;
    Pid pid(0.0,0.0,15.0);
    while(time < MAX_TIME) {
        double velocity = 0.1;
        findClosestIndex(pid.closest_index,rm.state_,rx,ry);
        std::cout << "idx: "  << pid.closest_index << " , "
            << "state.x: " << rm.state_.x << " , "
            << "state.y: " << rm.state_.y << " , "
            << "rx: " << rx.at(pid.closest_index) << " , "
            << "ry: " << ry.at(pid.closest_index) << std::endl;
        auto error = pid.calcError(rm.state_,rx.at(pid.closest_index),ry.at(pid.closest_index));
        std::cout << "=====" << error << "=====" << std::endl;
        auto steer = pid.updateError(error);
        if (steer < -1.0) {
            steer = -1.0;
        } else if (steer > 1.0) {
            steer = 1.0;
        }
        rm.updateState(steer,velocity);
        std::cout << rm.state_.x << " , "
            << rm.state_.y << " , "
            << rm.state_.theta << " , "
            << rm.state_.v << std::endl;
        auto dist_from_goal = std::sqrt((rm.state_.x - rx.back())*(rm.state_.x - rx.back()) + (rm.state_.y - ry.back())*(rm.state_.y - ry.back()));
        std::cout << "######" << dist_from_goal << "######" << std::endl;
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
    matplotlibcpp::title("Cubic Spline Path");
    matplotlibcpp::show();

    return 0;
}
