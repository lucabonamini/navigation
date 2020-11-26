#include "astar.h"

namespace astar {
    std::vector<std::vector<float> > calcFinalPath(Node goal, float reso){
        std::vector<float> rx;
        std::vector<float> ry;
        Node node = goal;
        while (node.p_node != NULL){
            node = *node.p_node;
            rx.push_back(node.x * reso);
            ry.push_back(node.y * reso);
        }
        return {rx,ry};
    }
    std::vector<std::vector<int>> calcObstacleMap(
        const std::vector<int>& px,
        const std::vector<int>& py,
        long unsigned int min_ox,
        long unsigned int max_ox,
        long unsigned int min_oy,
        long unsigned int max_oy,
        const float& resolution,
        const float& robot_size) {
            long unsigned int xwidth = max_ox-min_ox;
            long unsigned int ywidth = max_oy-min_oy;

            std::vector<std::vector<int> > obmap(static_cast<long unsigned int>(ywidth),
                std::vector<int>(static_cast<long unsigned int>(xwidth), 0));

            for(size_t i=0; i<xwidth; i++) {
                int x = i + min_ox;
                for(size_t j=0; j<ywidth; j++) {
                    int y = j + min_oy;
                    for(size_t k=0; k<px.size(); k++) {
                        float d = std::sqrt(std::pow((px[k]-x), 2)+std::pow((py[k]-y), 2));
                        if (d <= robot_size/resolution){
                            obmap[i][j] = 1;
                            break;
                        }
                    }
                }
            }
            return obmap;
    }

    std::vector<Node> getMotionModel() {
        return {Node{.x=1,.y=0,.sum_cost=1,.p_node=NULL},
        Node{.x=0,.y=1,.sum_cost=1,.p_node=NULL},
        Node{.x=-1,.y=0,.sum_cost=1,.p_node=NULL},
        Node{.x=0,.y=-1,.sum_cost=1,.p_node=NULL},
        Node{.x=-1,.y=-1,.sum_cost=std::sqrt(2),.p_node=NULL},
        Node{.x=-1,.y=1,.sum_cost=std::sqrt(2),.p_node=NULL},
        Node{.x=1,.y=-1,.sum_cost=std::sqrt(2),.p_node=NULL},
        Node{.x=1,.y=1,.sum_cost=std::sqrt(2),.p_node=NULL}};
    }


    float calcHeristic(Node n1, Node n2, float w=1.0) {
        return w * std::sqrt(std::pow(n1.x-n2.x, 2)+std::pow(n1.y-n2.y, 2));
    }

    bool verify_node (Node node,
                std::vector<std::vector<int> > obmap,
                int min_ox, int max_ox,
                int min_oy, int max_oy) {
        if (node.x < min_ox || node.y < min_oy || node.x >= max_ox || node.y >= max_oy) {
            return false;
        }

        if (obmap[node.x-min_ox][node.y-min_oy]) return false;

        return true;
    }

    void plan (
        const float& start_x,
        const float& start_y,
        const float& goal_x,
        const float& goal_y,
        const std::vector<float>& map_x,
        const std::vector<float>& map_y,
        const int& resolution,
        const float& robot_size) {
            std::vector<int> px,py; // Map in pixel coordinates
            Node start;
            start.x = static_cast<int>(std::round(start_x/resolution));
            start.y = static_cast<int>(std::round(start_y/resolution));
            Node goal;
            goal.x = static_cast<int>(std::round(goal_x/resolution));
            goal.y = static_cast<int>(std::round(goal_y/resolution));

            int min_ox = std::numeric_limits<int>::max();
            int max_ox = std::numeric_limits<int>::min();
            int min_oy = std::numeric_limits<int>::max();
            int max_oy = std::numeric_limits<int>::min();

            for (float x : map_x) {
                int map_x_px = static_cast<int>(std::round(x*1.0/resolution));
                px.push_back(map_x_px);
                min_ox = std::min(map_x_px, min_ox);
                max_ox = std::max(map_x_px, max_ox);
            }

            for (float y : map_y) {
                int map_y_py = static_cast<int>(std::round(y*1.0/resolution));
                py.push_back(map_y_py);
                min_oy = std::min(map_y_py, min_oy);
                max_oy = std::max(map_y_py, max_oy);
            }

            int xwidth = max_ox-min_ox;
            int ywidth = max_oy-min_oy;

            std::vector<std::vector<int>> visited_map(static_cast<long unsigned int>(xwidth),
                std::vector<int>(static_cast<long unsigned int>(ywidth),0));

            std::vector<std::vector<float>> path_cost(static_cast<long unsigned int>(xwidth),
                std::vector<float>(static_cast<long unsigned int>(ywidth), std::numeric_limits<float>::max()));

            path_cost.at(start.x).at(start_y) = 0;

            std::vector<std::vector<int>> obmap = calcObstacleMap(
                px,
                py,
                static_cast<long unsigned int>(min_ox),
                static_cast<long unsigned int>(max_ox),
                static_cast<long unsigned int>(min_oy),
                static_cast<long unsigned int>(max_oy),
                resolution,
                robot_size);

            auto cmp = [](const Node left, const Node right){return left.sum_cost > right.sum_cost;};
            std::priority_queue<Node,std::vector<Node>, decltype(cmp)> pq(cmp);

            pq.push(start);
            std::vector<Node> motion = getMotionModel();

            while(true) {
                Node node = pq.top();
                if (visited_map.at(node.x-min_ox).at(node.y-min_oy)==1) {
                    pq.pop();
                    continue;
                }
                else {
                    pq.pop();
                    visited_map.at(node.x-min_ox).at(node.y-min_oy)=1;
                }
                if (node.x == goal.x && node.y == goal.y) {
                    goal.sum_cost = node.sum_cost;
                    goal.p_node = new Node();
                    *goal.p_node=node;
                    break;
                }
                for (size_t i=0; i<motion.size(); i++) {
                    Node new_node = Node{.x=node.x+motion.at(i).x,
                        .y=node.y+motion.at(i).y,
                        .sum_cost=path_cost.at(node.x).at(node.y)+motion.at(i).sum_cost+calcHeristic(goal,node),
                        .p_node=new Node()};
                    *new_node.p_node = node;

                    if (!verify_node(new_node, obmap, static_cast<int>(min_ox), static_cast<int>(max_ox), static_cast<int>(min_oy), static_cast<int>(max_oy))) {
                        continue;
                    }

                    if (visited_map.at(new_node.x-min_ox).at(new_node.y-min_oy)) {
                        continue;
                    }

                    if (path_cost.at(node.x).at(node.y)+motion.at(i).sum_cost < path_cost.at(new_node.x).at(new_node.y)) {
                        path_cost.at(new_node.x).at(new_node.y)=path_cost.at(node.x).at(node.y)+motion.at(i).sum_cost;
                        pq.push(new_node);
                    }
                }
            }

            calcFinalPath(goal, resolution);
    }
}
