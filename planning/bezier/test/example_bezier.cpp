#include "bezier/bezier.h"
#include "matplotlibcpp.h"

using namespace matplotlibcpp;

int main() {
    double start_x = 5.0;
    double start_y = 1.0;
    double end_x = -6.0;
    double end_y = -8.0;
    std::vector<std::array<double,2>> control_points = { {{5.0,1.0}},
                                                         {{-2.78,1.0}},
                                                         {{-11.5,-4.5}},
                                                         {{-3.0,-3.0}},
                                                         {{-6.0,-8.0}} };
    auto path = bezier::calcBezierPath(control_points);

    // Visualization
    std::vector<double> sx, sy;
    std::vector<double> ex, ey;
    std::vector<double> wx, wy;
    std::vector<double> ctrl_x, ctrl_y;
    sx.push_back(start_x);
    sy.push_back(start_y);
    ex.push_back(end_x);
    ey.push_back(end_y);
    for (auto &p : path) {
        wx.push_back(p.at(0));
        wy.push_back(p.at(1));
    }
    for (auto &c : control_points) {
        ctrl_x.push_back(c.at(0));
        ctrl_y.push_back(c.at(1));
    }
    figure();
    plot(sx,sy,"or");
    plot(ex,ey,"or");
    plot(wx,wy,"-k");
    plot(ctrl_x,ctrl_y,"xb");
    grid(true);
    show();

    return 0;
}