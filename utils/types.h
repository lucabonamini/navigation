#pragma once

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
}