#pragma once

#include "types.h"
#include <array>
#include <limits>
#include <math.h>

namespace utils {

static inline void findClosestIndex(int &index,
    const types::Point& point,
    const types::Path& path) {
        double closest_distance = std::numeric_limits<double>::max();
        for (size_t i = index; i < path.size(); i++) {
            double g_x = path.at(i).point.x;
            double g_y = path.at(i).point.y;
            double current_closest_distance = pow((point.x - g_x),2) +
                pow((point.y - g_y),2);
            if (current_closest_distance < closest_distance) {
                closest_distance = current_closest_distance;
                index = i;
            }
        }
}

template <typename T>
T normalizeAnglePositive(T angle) {
    return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
}

template <typename T>
T normalizeAngle(T angle) {
    T a = normalizeAnglePositive(angle);
    if (a > M_PI)
        a -= 2.0 *M_PI;
    return a;
}

template <typename T>
T shortestAngularDistance(T from, T to) {
    return normalizeAngle(to-from);
}

template <typename T>
T sumOfPower(std::vector<T> value_list) {
    T sum = 0;
    for(T item:value_list)
        sum += item*item;
    return sum;
}
} // namespace utils