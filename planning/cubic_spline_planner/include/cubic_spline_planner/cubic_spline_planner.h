#ifndef _CUBIC_SPLINE_PLANNER_H_
#define _CUBIC_SPLINE_PLANNER_H_

#include <eigen3/Eigen/Core>
#include <vector>
#include <optional>

inline std::vector<double> cumSum(const std::vector<double>& input) {
    std::vector<double> output;
    double temp = 0;
    for (size_t i=0; i<input.size(); i++){
        temp += input.at(i);
        output.push_back(temp);
    }
    return output;
}

std::vector<double> vecDiff(const std::vector<double>& input)
{
    std::vector<double> output;
    for (size_t i=1; i<input.size(); i++){
        output.push_back(input.at(i) - input.at(i-1));
    }
    return output;
}

namespace CubicSplinePlanner {
    class Spline{

        public:

            std::vector<double> x;
            std::vector<double> y;
            int nx;
            std::vector<double> h;
            std::vector<double> a;
            std::vector<double> b;
            std::vector<double> c;
            std::vector<double> d;

            Spline(){};

            Spline(std::vector<double> x_, std::vector<double> y_);

            double calc(double t);

            double calc_d(double t);

            double calc_dd(double t);

        private:

            Eigen::MatrixXd calc_A();
            Eigen::VectorXd calc_B();

            int bisect(double t, int start, int end);
    };

    class Spline2D{

        public:

            Spline sx;
            Spline sy;
            std::vector<double> s;

            Spline2D(std::vector<double> x, std::vector<double> y);

            std::array<double,2> calc_position(double s_t);

            double calc_curvature(double s_t);

            double calc_yaw(double s_t);

            std::array<double,2> calcCartesianCoordinates(const double& s,
                const double& d);

        private:

            std::vector<double> calc_s(std::vector<double> x, std::vector<double> y);
    };
} // CubicSplinePlanner



#endif