#ifndef _BEZIER_H_
#define _BEZIER_H_

#include <array>
#include <iostream>
#include <cmath>
#include <vector>

template<typename T>
std::array<T,2> operator*(std::array<T,2> arr,
                          T i) {
    std::array<T,2> res;
    res.at(0) = arr.at(0) * i;
    res.at(1) = arr.at(1) * i;
    return res;
}

template<typename T>
T fact(const T &n) {
    T res = 1.0;
    for (double i = 2.0; i < n; i++) {
        res = res * i;
    }
    return res;
}

template<typename T>
T nCr(const T &n,
    const T &r) {
        return fact(n) / (fact(r) * fact(n - r));
}

template<typename T>
std::vector<T> linspace(const T& start_in,
            const T& end_in,
            T num_in) {
    // Define the vector to return
    std::vector<T> linspaced;
    T start = static_cast<T>(start_in);
    T end = static_cast<T>(end_in);
    // If you want to divide the vector in 1 only element, return the starting point
    if (num_in == 1) {
        linspaced.push_back(start);
        return linspaced;
    }
    // else define the delta between two consecutive points
    T delta = (end - start) / num_in ;
    for(int i=0; i < num_in; ++i)
        linspaced.push_back(start + delta * i);
    // I want to ensure that start and end are exactly the same as the input
    linspaced.push_back(end);
    return linspaced;
}

namespace bezier {
double bernsteinPoly(const size_t &n,
                     const size_t &i,
                     const double &t);

std::array<double,2> bezier(const double &t,
                            const std::vector<std::array<double,2>> &control_points);

std::vector<std::array<double,2>> calcBezierPath(const std::vector<std::array<double,2>> &control_points,
                                                const int &n_points = 100);
} // namespace bezier
#endif