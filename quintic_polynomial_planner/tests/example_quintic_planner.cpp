#include <quintic_polynomial_planner/quintic_polynomial_planner.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace QuinticPolynomialPlanner;

DEFINE_double(start_x, NAN, "Start X position.");
DEFINE_double(start_y, NAN, "Start Y position.");
DEFINE_double(goal_x, NAN, "Goal X position.");
DEFINE_double(goal_y, NAN, "Goal Y position.");
DEFINE_double(min_t, NAN, "Minimum sample time.");
DEFINE_double(max_t, NAN, "Maximum sample time.");

cv::Point2i cv_offset(
    float x, float y,
    int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + image_width/2;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
}

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
          cv::Mat bg(2000, 2000, CV_8UC3, cv::Scalar(255, 255, 255));
            for(unsigned int i=1; i<output->rx.size(); i++){
                cv::line(
                bg,
                cv_offset(output->rx[i-1], output->ry[i-1], bg.cols, bg.rows),
                cv_offset(output->rx[i], output->ry[i], bg.cols, bg.rows),
                cv::Scalar(0, 0, 0),
                10);
            }
            cv::circle(bg, cv_offset(FLAGS_start_x, FLAGS_start_y, bg.cols, bg.rows),
                30, cv::Scalar(255,0,0), 5);
            cv::circle(bg, cv_offset(FLAGS_goal_x, FLAGS_goal_y, bg.cols, bg.rows),
                30, cv::Scalar(255,0,0), 5);

              cv::imwrite("./csp.png", bg);
    }
    return 0;
}
