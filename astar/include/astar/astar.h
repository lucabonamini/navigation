#ifndef _A_STAR_H_
#define _A_STAR_H_

#include <iostream>
#include <vector>
#include <math.h>
#include <queue>
#include <tuple>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

struct Node {
    int x; // px
    int y; // px
    float sum_cost = 0.0;
    Node* p_node = NULL;
};

namespace astar {
    /**
     * @brief Run A* planning algorithm
     * @param start_x Start point X coordinate
     * @param start_y Start point Y coordinate
     * @param goal_x Goal point X coordinate
     * @param goal_y Goal point Y coordinate
     * @param map_x Map points X coordinates
     * @param map_y Map points Y coordinates
     * @param resolution Grid size
     * @param robot_size Robot dimension
     */
    void plan(
        const float& start_x,
        const float& start_y,
        const float& goal_x,
        const float& goal_y,
        const std::vector<float>& map_x,
        const std::vector<float>& map_y,
        const int& resolution,
        const float& robot_size);

    std::vector<std::vector<int>> calcObstacleMap(
        const std::vector<int>& px,
        const std::vector<int>& py,
        long unsigned int min_ox,
        long unsigned int max_ox,
        long unsigned int min_oy,
        long unsigned int max_oy,
        const float& resolution,
        const float& robot_size,
        cv::Mat& bg,
        long unsigned int img_reso);
}

#endif