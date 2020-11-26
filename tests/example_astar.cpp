#include "astar/astar.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main() {
    // Start point
    float sx = 10.0;
    float sy = 10.0;
    // Goal point
    float gx = 50.0;
    float gy = 50.0;

    float resolution = 1.0;
    float robot_size = 1.0;

    std::vector<float> obx,oby;

    // Create map
    for(int i=0; i<60; i++) {
        obx.push_back(i);
        oby.push_back(60);
    }
    for(int i=0; i<60; i++) {
        obx.push_back(60);
        oby.push_back(i);
    }
    for(int i=0; i<61; i++) {
        obx.push_back(i);
        oby.push_back(60);
    }
    for(int i=0; i<61; i++) {
        obx.push_back(0);
        oby.push_back(i);
    }
    for(int i=0; i<40; i++) {
        obx.push_back(20);
        oby.push_back(i);
    }
    for(int i=0; i<40; i++) {
        obx.push_back(40);
        oby.push_back(60 - i);
    }

    astar::plan(sx,sy,gx,gy,obx,oby,resolution,robot_size);

    return 0;
}
