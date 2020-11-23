#include <cubic_spline_planner/cubic_spline_planner.h>
#include "cubic_spline_planner/matplotlibcpp.h"

using namespace CubicSplinePlanner;

int main(int argc, char** argv) {
    
    std::vector<double> wx {-2.5,0.0,2.5,5.0,7.5,3.0,-1.0};
    std::vector<double> wy {0.7,-6.0,5.0,6.5,0.0,5.0,-2.0};

    std::vector<double> res_x,res_y;

    Spline2D csp(wx,wy);

    for (double i=0; i<csp.s.back(); i+=0.1) {
        auto p = csp.calc_position(i);
        res_x.push_back(p.at(0));
        res_y.push_back(p.at(1));
    }

    matplotlibcpp::plot(res_x,res_y,"-k");
    matplotlibcpp::plot(wx,wy,"ob");
    matplotlibcpp::title("Cubic Spline Path");
    matplotlibcpp::show();

    return 0;
}
