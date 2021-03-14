#include "mpc/mpc.h"
#include "cubic_spline_planner/cubic_spline_planner.h"
#include "matplotlibcpp.h"

using namespace matplotlibcpp;

constexpr int MAX_TIME = 5000;

void smoothYaw(std::vector<double> &cyaw){
  for(size_t i=0; i<cyaw.size()-1; i++){
    double dyaw = cyaw[i+1] - cyaw[i];

    while (dyaw > M_PI/2.0){
      cyaw[i+1] -= M_PI*2.0;
      dyaw = cyaw[i+1] - cyaw[i];
    }
    while (dyaw < -M_PI/2.0){
      cyaw[i+1] += M_PI*2.0;
      dyaw = cyaw[i+1] - cyaw[i];
    }
  }
}

int main() {

    // std::vector<double> wx {-2.5,0.0,2.5,5.0,7.5,3.0,-1.0};
    // std::vector<double> wy {0.7,-6.0,2.0,-4.0,0.0,5.0,-2.0};
    std::vector<double> wx({0.0, 60.0, 125.0,  50.0,   75.0,  35.0,  -10.0});
    std::vector<double> wy({0.0,  0.0,  50.0,  65.0,   30.0,  50.0,  -20.0});

    std::vector<State> states;

    std::vector<double> rx,ry,ryaw,rk,rs;

    CubicSplinePlanner::Spline2D csp(wx,wy);
    for (double i=0; i<csp.s.back(); i+=0.1) {
        auto p = csp.calc_position(i);
        rx.push_back(p.at(0));
        ry.push_back(p.at(1));
        ryaw.push_back(csp.calc_yaw(i));
        rk.push_back(csp.calc_curvature(i));
        rs.push_back(i);
    }

    auto speed_profile = calcSpeedProfile(rs,rk);
    State initial_state {.x=rx.at(0),.y=ry.at(0),.yaw=ryaw.at(0),.v=speed_profile.at(0)};

    smoothYaw(ryaw);

    MPC mpc(initial_state,rx,ry,ryaw,rk,rs,speed_profile);
    mpc.run();

    vector<double> x,y;
    for (size_t i = 0; i < mpc.states_.size(); i++) {
        x.push_back(mpc.states_.at(i).x);
        y.push_back(mpc.states_.at(i).y);
    }
    figure();
    plot(rx,ry,"-k");
    title("MPC Controller");
    plot(x,y,"or");
    show();


    return 0;
}
