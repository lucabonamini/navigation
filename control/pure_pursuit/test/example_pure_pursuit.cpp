#include "models/bicycle.h"
#include "models/unicycle.h"
#include "pure_pursuit/pure_pursuit.h"
#include "cubic_spline_planner/cubic_spline_planner.h"
#include "utils/types.h"
#include "utils/utils.h"
#include "matplotlibcpp.h"

constexpr int MAX_TIME = 5000;

int main() {

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
    State state;
    state.x = rx.front();
    state.y = ry.front();
    state.yaw = std::atan2((ry.at(1)-ry.at(0)),(rx.at(1)-rx.at(0)));
    state.v = 0.1;
    // model::DynamicUnicycle d_uni(50.0);
    model::Unicycle d_uni(10.0);
    control::PurePursuit pp;

    // double ref_velocity = 30/3.6; // m/s
    int closest_ind = 0;
    while(time < MAX_TIME) {
        std::array<double,2> point {state.x,state.y};
        utils::findClosestIndex(closest_ind,point,rx,ry);
        std::cout << "id: " << closest_ind << " , "
            << "x: " << rx.at(closest_ind) << " , "
            << "y: " << ry.at(closest_ind) << " , "
            << "px: " << point.at(0) << " , "
            << "py: " << point.at(1) << std::endl;
        // auto cte = d_uni.calcTrackError(state,rx.at(closest_ind),ry.at(closest_ind));
        auto lad = state.v * 1.0;
        // auto lad = 1.0;
        auto id = round(lad / 0.1) + closest_ind;
        if (id > rx.size() - 1) {
            id = rx.size() - 1;
        }
        Input input;
        input.robot_state = state;
        input.target_point = Point{.x = rx.at(id),
            .y = ry.at(id)};
        input.desired_velocity = 0.5;
        auto cc = pp.calcCommands(input);

        Controls controls {.steer = cc.steer, .v = cc.v, .a = 0.0};

        d_uni.updateState(state,controls);

        auto dist_from_goal = std::sqrt((state.x - rx.back())*(state.x - rx.back()) +
            (state.y - ry.back())*(state.y - ry.back()));

        std::cout << "dist_from_goal: " << dist_from_goal << std::endl;

        if (dist_from_goal < 0.1) {
            std::cout << "==== GOAL ====" << std::endl;
            break;
        }
        time++;
        states.push_back(state);
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
    matplotlibcpp::title("PID Controller");
    matplotlibcpp::show();

    return 0;
}
