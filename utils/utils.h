#pragma once

#include "types.h"
#include <array>
#include <limits>
#include <math.h>
#include <vector>

namespace utils {
template <typename T>
void findClosestIndex(int &index,
    const std::array<T,2> point,
    const std::vector<T> &x,
    const std::vector<T> &y) {
        T closest_distance = std::numeric_limits<T>::max();
        for (size_t i = index; i < x.size(); i++) {
            T g_x = x.at(i);
            T g_y = y.at(i);
            T current_closest_distance = pow((point.at(0) - g_x),2) +
                pow((point.at(1) - g_y),2);
            if (current_closest_distance < closest_distance) {
                closest_distance = current_closest_distance;
                index = i;
            }
        }
}

template <typename T>
void normalizeAngle(T &angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return;
}
} // namespace utils