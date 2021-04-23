#include "rrt/rrt.h"

int main() {

    Config config {
        .expand_distance = 0.5,
        .max_iter = 500,
        .goal_sample_rate = 5,
        .path_resolution = 1.0,
        .min_rand = -2.0,
        .max_rand = 15.0
    };

    Node* start_node = new Node(0.0,0.0);
    Node* end_node = new Node(6.0,9.0);

    std::vector<Obstacle> obstacle_list{
		Obstacle {5, 5, 1},
		Obstacle {3, 6, 2},
		Obstacle {3, 8, 2},
		Obstacle {3, 10, 2},
		Obstacle {7, 5, 2},
		Obstacle {9, 5, 2}};

    Input input {
        .start_node = start_node,
        .end_node = end_node,
        .obstacles = obstacle_list};

    RRT rrt(config);
    auto path = rrt.planning(input);

    return 0;
}