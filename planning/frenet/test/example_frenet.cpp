#include "frenet/frenet.h"
#include "matplotlibcpp.h"
#include <iostream>

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#define REALTIME_VISUALIZATION

constexpr int MAX_ITERATION = 5000;
constexpr double TARGET_SPEED = 0.5; // m/s

#ifdef REALTIME_VISUALIZATION
cv::Point2i cv_offset(
    float x, float y, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + 300;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
}
#endif

int main() {
    std::vector<double> wx {0.0, 10.0, 20.5, 35.0, 70.5};
    std::vector<double> wy {0.0, -6.0, 5.0, 6.5, 0.0};

    std::vector<Obstacle> obstacles{
        Obstacle{.x=20.0,.y=10.0},
        Obstacle{.x=30.0,.y=6.0},
        Obstacle{.x=30.0,.y=8.0},
        Obstacle{.x=35.0,.y=8.0},
        Obstacle{.x=50.0,.y=3.0}
    };

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

    input.obstacles = obstacles;

    #ifdef REALTIME_VISUALIZATION
    cv::namedWindow("frenet", cv::WINDOW_NORMAL);
    #else
    std::vector<RobotState> states;
    #endif

    while (iteration < MAX_ITERATION) {
        input.target_speed = TARGET_SPEED;
        auto best_path = Frenet::planFrenetPath(input);
        auto dist_from_goal = std::sqrt((best_path->x.at(0) - rx.back())*(best_path->x.at(0) - rx.back()) +
            (best_path->y.at(0) - ry.back())*(best_path->y.at(0) - ry.back()));

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

        #ifdef REALTIME_VISUALIZATION
        cv::Mat bg(2000, 8000, CV_8UC3, cv::Scalar(255, 255, 255));
        for(size_t i = 1; i < rx.size(); ++i){
        cv::line(
            bg,
            cv_offset(rx.at(i-1), ry.at(i-1), bg.rows),
            cv_offset(rx.at(i), ry.at(i), bg.rows),
            cv::Scalar(0, 0, 0),
            10);
        }
        for(size_t i = 0; i < best_path->x.size(); ++i){
        cv::circle(
            bg,
            cv_offset(best_path->x.at(i), best_path->y.at(i), bg.rows),
            40, cv::Scalar(255, 0, 0), -1);
        }
        cv::circle(
            bg,
            cv_offset(best_path->x.front(), best_path->y.front(), bg.rows),
            50, cv::Scalar(0, 255, 0), -1);
        for(size_t i = 0; i < obstacles.size(); ++i){
            cv::circle(
                bg,
                cv_offset(obstacles.at(i).x, obstacles.at(i).y, bg.rows),
                40, cv::Scalar(0, 0, 255), 5);
        }
        cv::putText(
            bg,
            "Speed: " + std::to_string(best_path->s_d.at(1)*3.6).substr(0, 4) + "km/h",
            cv::Point2i((int)bg.cols*0.5, (int)bg.rows*0.1),
            cv::FONT_HERSHEY_SIMPLEX,
            5,
            cv::Scalar(0, 0, 0),
            10);
        cv::imshow("frenet", bg);
        cv::waitKey(5);
        #else
        states.push_back(input.robot_state);
        #endif

    }

    #ifndef REALTIME_VISUALIZATION
    std::vector<double> state_x,state_y;
    for (auto s:states) {
        state_x.push_back(s.x);
        state_y.push_back(s.y);
    }
    std::vector<double> obsx, obsy;
    for (auto ob:obstacles) {
        obsx.push_back(ob.x);
        obsy.push_back(ob.y);
    }
    matplotlibcpp::figure();
    matplotlibcpp::plot(rx,ry,"-k");
    matplotlibcpp::plot(wx,wy,"ob");
    matplotlibcpp::plot(state_x,state_y,"xr");
    matplotlibcpp::plot(obsx,obsy,"xk");
    matplotlibcpp::show();
    #endif

    return 0;
}