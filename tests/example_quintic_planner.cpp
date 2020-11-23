#include <cubic_spline_planner/cubic_spline_planner.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "cubic_spline_planner/matplotlibcpp.h"

using namespace QuinticPolynomialPlanner;

DEFINE_double(start_x, NAN, "Start X position.");
DEFINE_double(start_y, NAN, "Start Y position.");
DEFINE_double(goal_x, NAN, "Goal X position.");
DEFINE_double(goal_y, NAN, "Goal Y position.");
DEFINE_double(min_t, 1.0, "Minimum sample time.");
DEFINE_double(max_t, 10.0, "Maximum sample time.");

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc,&argv,true);
    CHECK(!std::isnan(FLAGS_start_x)) << "Please specify --start_x";
    CHECK(!std::isnan(FLAGS_start_y)) << "Please specify --start_y";
    CHECK(!std::isnan(FLAGS_goal_x)) << "Please specify --goal_x";
    CHECK(!std::isnan(FLAGS_goal_y)) << "Please specify --goal_y";
    CHECK(!std::isnan(FLAGS_min_t)) << "Please specify --min_t";
    CHECK(!std::isnan(FLAGS_max_t)) << "Please specify --max_t";

    Input input;
    input.sx = FLAGS_start_x;
    input.sy = FLAGS_start_y;
    input.gx = FLAGS_goal_x;
    input.gy = FLAGS_goal_y;
    input.min_t = FLAGS_min_t;
    input.max_t = FLAGS_max_t;

    auto output = plan(input);

    if (!output) {
        LOG(ERROR) << "No path found.";
    } else {
        std::vector<double> x,y;
        x.push_back(FLAGS_start_x);
        x.push_back(FLAGS_goal_x);
        y.push_back(FLAGS_start_y);
        y.push_back(FLAGS_goal_y);
        matplotlibcpp::plot(output->rx,output->ry,"-k");
        matplotlibcpp::plot(x,y,"ob");
        matplotlibcpp::title("Quintic Polynomial Path");
        matplotlibcpp::show();
    }
    return 0;
}
