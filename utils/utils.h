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

/*!
* \brief normalize_angle_positive
*
*        Normalizes the angle to be 0 to 2*M_PI
*        It takes and returns radians.
*/
static inline double normalize_angle_positive(double angle) {
    return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
}


/*!
* \brief normalize
*
* Normalizes the angle to be -M_PI circle to +M_PI circle
* It takes and returns radians.
*
*/
static inline double normalize_angle(double angle) {
    double a = normalize_angle_positive(angle);
    if (a > M_PI)
        a -= 2.0 *M_PI;
    return a;
}


/*!
* \function
* \brief shortest_angular_distance
*
* Given 2 angles, this returns the shortest angular
* difference.  The inputs and ouputs are of course radians.
*
* The result
* would always be -pi <= result <= pi.  Adding the result
* to "from" will always get you an equivelent angle to "to".
*/

static inline double shortest_angular_distance(double from, double to) {
    return normalize_angle(to-from);
}

template <typename T>
T sum_of_power(std::vector<T> value_list) {
    T sum = 0;
    for(T item:value_list)
        sum += item*item;
    return sum;
}
} // namespace utils