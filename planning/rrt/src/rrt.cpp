#include "rrt/rrt.h"
#include <iostream>

RRT::RRT() {}

RRT::RRT(const Config& config) :
    config_(config),
    goal_gen_(goal_rd_()),
    goal_dis_(std::uniform_int_distribution<int>(0,100)),
    area_gen_(area_rd_()),
    area_dis_(std::uniform_real_distribution<double>(config_.min_rand,
        config_.max_rand)) {}

Node* RRT::generateRandomNode(Node* end_node) {
    Node* random_node = new Node();
    if (goal_dis_(goal_gen_) > config_.goal_sample_rate) {
        random_node->x = area_dis_(goal_gen_);
        random_node->y = area_dis_(goal_gen_);
    } else {
        random_node->x = end_node->x;
        random_node->y = end_node->y;
    }
    return random_node;
}

int RRT::getNearestNodeIndex(const std::vector<Node*>& nodes_list,
    Node* node) {
        double closest_distance = std::numeric_limits<double>::max();
        int closest_index = 0;
        for (size_t i = 0; i < nodes_list.size(); ++i) {
            double distance = pow((nodes_list.at(i)->x - node->x),2) +
                pow((nodes_list.at(i)->y - node->y),2);
            if (distance < closest_distance) {
                closest_distance = distance;
                closest_index = i;
            }
        }
        return closest_index;
}

std::pair<double,double> RRT::calcDistanceAndAngle(Node* node1,
    Node* node2) {
        double distance = sqrt(pow((node2->x - node1->x),2) +
            pow((node2->y - node1->y),2));
        double theta = atan2((node2->y-node1->y),(node2->x-node1->x));
        return std::make_pair(distance, theta);
}

Node* RRT::expandTree(Node* from_node,
    Node* to_node,
    const double& expand_distance) {
        Node* new_node = new Node(from_node->x,from_node->y);
        new_node->path_x.push_back(from_node->x);
        new_node->path_y.push_back(from_node->y);
        new_node->parent = NULL;

        auto dist_and_angle = calcDistanceAndAngle(from_node,to_node);

        auto extend_length = expand_distance;
        // if (extend_length > dist_and_angle.first) {
            extend_length = dist_and_angle.first;
        // }

        auto n_expand = floor(extend_length  / config_.path_resolution);

        for (int i = 0; i < n_expand; ++i) {
            new_node->x += config_.path_resolution * cos(dist_and_angle.second);
            new_node->y += config_.path_resolution * sin(dist_and_angle.second);
            new_node->path_x.push_back(new_node->x);
            new_node->path_y.push_back(new_node->y);
        }

        dist_and_angle = calcDistanceAndAngle(new_node,to_node);

        if (dist_and_angle.first <= config_.path_resolution) {
            new_node->path_x.push_back(to_node->x);
            new_node->path_y.push_back(to_node->y);
            new_node->x = to_node->x;
            new_node->y = to_node->y;
        }

        new_node->parent = from_node;

        return new_node;
}

bool RRT::checkCollision(Node* node,
    const std::vector<Obstacle>& obstacles) {
        for (const auto& ob : obstacles) {
            auto dist = sqrt(pow((ob.x-node->x),2) +
                pow((ob.y-node->y),2));
            if (dist <= ob.size) {
                return true;
            }
        }
        return false;
}

bool RRT::calcDistToGoal(const std::vector<Node*>& nodes_list,
    Node* end_node) {
        double dist_to_goal = sqrt(pow((nodes_list.back()->x - end_node->x),2) +
            pow((nodes_list.back()->y - end_node->y),2));
        if (dist_to_goal <= config_.expand_distance) {
            return true;
        }
        return false;
}

std::vector<Node*> RRT::generateFinalPath(const std::vector<Node*>& nodes_list,
    const size_t& goal_index,
    Node* end_node,
    const int& img_reso) {
        std::vector<Node*> path;
        path.push_back(end_node);
        Node* node = nodes_list.at(goal_index);
        while (node->parent != NULL) {
            // cv::line(
            //     bg,
            //     cv::Point((int)((node->x-config_.min_rand)*img_reso), (int)((node->y-config_.min_rand)*img_reso)),
            //     cv::Point((int)((node->parent->x-config_.min_rand)*img_reso), (int)((node->parent->y-config_.min_rand)*img_reso)),
            //     cv::Scalar(255,0,255), 5);
            // cv::imshow("rrt", bg);
            // cv::waitKey(50);
            path.push_back(node);
            node = node->parent;
        }
        return path;
}

std::vector<Node*> RRT::planning(const Input& input) {

    // //visualization
	// cv::namedWindow("rrt", cv::WINDOW_NORMAL);
	// int img_size = (int)(config_.max_rand - config_.min_rand);
	// int img_reso = 50;
    // cv::Mat bg(img_size * img_reso, img_size * img_reso,
    //          CV_8UC3, cv::Scalar(255,255,255));

    // cv::circle(bg, cv::Point((int)((input.start_node->x-config_.min_rand)*img_reso), (int)((input.start_node->y-config_.min_rand)*img_reso)), 20, cv::Scalar(0,0,255), -1);
	// cv::circle(bg, cv::Point((int)((input.end_node->x-config_.min_rand)*img_reso), (int)((input.end_node->y-config_.min_rand)*img_reso)), 20, cv::Scalar(255,0,0), -1);
	// for(auto item:input.obstacles){
    //   cv::circle(bg, cv::Point((int)((item.x-config_.min_rand)*img_reso), (int)((item.y-config_.min_rand)*img_reso)), (int)item.size * img_reso, cv::Scalar(0,0,0), -1);
	// }

    std::vector<Node*> nodes_list;
    nodes_list.push_back(input.start_node);
    for (int i = 0; i < config_.max_iter; ++i) {
        auto random_node = generateRandomNode(input.end_node);
        auto nearest_index = getNearestNodeIndex(nodes_list,random_node);
        auto nearest_node = nodes_list.at(nearest_index);

        // ? Unnecessary?
        // auto new_node = expandTree(nearest_node,random_node,config_.expand_distance);
        double theta = std::atan2(random_node->y-nearest_node->y, random_node->x-nearest_node->x); 
        Node* new_node = new Node(nearest_node->x+config_.expand_distance*std::cos(theta), nearest_node->y+config_.expand_distance*std::sin(theta));
        new_node->parent = nearest_node;
        if (!checkCollision(new_node,input.obstacles)) {
            nodes_list.push_back(new_node);
            // //visualization
            // cv::line(
            //     bg,
            //     cv::Point((int)((new_node->x-config_.min_rand)*img_reso), (int)((new_node->y-config_.min_rand)*img_reso)),
            //     cv::Point((int)((nearest_node->x-config_.min_rand)*img_reso), (int)((nearest_node->y-config_.min_rand)*img_reso)),
            //     cv::Scalar(0,255,0), 5);
            // cv::imshow("rrt", bg);
            // cv::waitKey(50);
        }

        if (calcDistToGoal(nodes_list,input.end_node)) {
            auto final_node = expandTree(nodes_list.back(),input.end_node,config_.expand_distance);
            if (!checkCollision(final_node,input.obstacles)) {
                auto path = generateFinalPath(nodes_list,nodes_list.size()-1,input.end_node,bg,img_reso);
                return path;
            }
        }
    }
    return std::vector<Node*>{};
}