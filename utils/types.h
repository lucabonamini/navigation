#pragma once

#include <vector>

namespace types {
    struct State {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        double v = 0.0;
    };
    struct Controls {
        double steer = 0.0;
        double v = 0.0;
        double a = 0.0;
    };

    struct Point {
        double x = 0.0;
        double y = 0.0;
    };
    struct Position {
        Point point;
        double yaw = 0.0;
    };

    using Track = std::vector<Point>;
    using Path = std::vector<Position>;
}