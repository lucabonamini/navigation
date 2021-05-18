#pragma once

#include <array>
#include <random>
#include <vector>

class Node {
    public:
    Node(){};
    Node(double x_, double y_): x(x_), y(y_), parent(NULL) {};
    double x = 0.0;
    double y = 0.0;
    std::vector<double> path_x;
    std::vector<double> path_y;
    Node* parent;
};

// ? config parameters may come from a file
struct Config {
    double expand_distance = 0.0;
    int max_iter = 0;
    int goal_sample_rate = 0;
    double path_resolution = 0.0;
    double min_rand = 0.0;
    double max_rand = 0.0;
};

struct Obstacle {
    double x = 0.0;
    double y = 0.0;
    double size = 0.0;
};

struct Input {
    Node* start_node;
    Node* end_node;
    std::vector<Obstacle> obstacles;
};

class RRT {
    public:
    RRT();
    RRT(const Config& config);
    std::vector<Node*> planning(const Input& input);
    Node* generateRandomNode(Node* end_node);
    int getNearestNodeIndex(const std::vector<Node*>& nodes_list,
        Node* node);
    std::pair<double,double> calcDistanceAndAngle(
        Node* node1,
        Node* node2);
    Node* expandTree(Node* from_node,
        Node* to_node,
        const double& expand_distance);
    bool checkCollision(Node* node,
        const std::vector<Obstacle>& obstacles);
    bool calcDistToGoal(const std::vector<Node*>& nodes_list,
        Node* end_node);
    std::vector<Node*> generateFinalPath(const std::vector<Node*>& nodes_list,
        const size_t& goal_index,
        Node* end_node);
    Config config_;
    std::random_device goal_rd_;
    std::mt19937 goal_gen_;
    std::uniform_int_distribution<int> goal_dis_;

	std::random_device area_rd_;
    std::mt19937 area_gen_;
    std::uniform_real_distribution<double> area_dis_;
};