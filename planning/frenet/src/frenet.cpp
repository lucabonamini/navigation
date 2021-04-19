#include "frenet/frenet.h"

namespace Frenet {
    bool isPathInCollision(FrenetPath& path, const Input& input) {
        for (auto& obstacle : input.obstacles) {
            for (size_t i = 0; i < path.x.size(); ++i) {
                double distance = std::sqrt(std::pow((path.x.at(i)-obstacle.x),2) +
                    std::pow((path.y.at(i)-obstacle.y),2));
                if (distance <= 0.5) {
                    return true;
                }
            }
        }
        return false;
    }
    void calculatePathCost(FrenetPath& path, const Input& input) {
        double n = path.s.size();
        double m = path.d.size();
        double Jp = utils::sum_of_power(path.d_ddd) / m;
        double Js = utils::sum_of_power(path.s_ddd) / n;
        double Jd = utils::sum_of_power(path.d) / m;
        double ds = std::pow((input.target_speed - path.s_d.back()), 2);

        double lateral_distance = 0.0;
        if (path.finalLateralDist < 0.0) {
            lateral_distance = fabs(path.finalLateralDist) * 1.1;
        } else {
            lateral_distance = fabs(path.finalLateralDist);
        }
        path.costLat = kj * Jp + kt * path.finalTime + kd * Jd + lateral_distance;
        path.costLong = kj * Js + kt * path.finalTime + ks * ds;

        path.costTotal = klat * path.costLat + klon * path.costLong;
        return;
    }
    std::vector<FrenetPath> calculatePathsInFrenetCoordinates(const Input& input) {
        std::vector<FrenetPath> frenet_paths_list;
        int d_size = sizeof(d_vec)/sizeof(d_vec[0]);
        int t_size = sizeof(t_vec)/sizeof(t_vec[0]);
        for (int di = 0; di < d_size; ++di) {
            for (int ti = 0; ti < t_size; ++ti) {
                FrenetPath fp;
                fp.finalTime = t_vec[ti];
                fp.finalLateralDist = d_vec[di];
                QuinticPolynomial lat_qp(input.robot_state.s_lat,
                    input.robot_state.v_lat,
                    input.robot_state.a_lat,
                    d_vec[di],
                    0.0,
                    0.0,
                    t_vec[ti]);
                for (double t = 0; t < t_vec[ti]; t += 0.2) {
                    fp.t.push_back(t);
                    fp.d.push_back(lat_qp.calc_point(t));
                    fp.d_d.push_back(lat_qp.calc_first_derivative(t));
                    fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
                    fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
                }
                QuarticPolynomial lon_qp(input.robot_state.s_long,
                    input.robot_state.v_long,
                    0.0,
                    input.target_speed,
                    0.0,
                    t_vec[ti]);
                fp.max_speed = std::numeric_limits<double>::min();
                fp.max_accel = std::numeric_limits<double>::min();
                for (const auto& t : fp.t) {
                    fp.s.push_back(lon_qp.calc_point(t));
                    fp.s_d.push_back(lon_qp.calc_first_derivative(t));
                    fp.s_dd.push_back(lon_qp.calc_second_derivative(t));
                    fp.s_ddd.push_back(lon_qp.calc_third_derivative(t));
                    if (fp.s_d.back() > fp.max_speed) {
                        fp.max_speed = fp.s_d.back();
                    }
                    if (fp.s_dd.back() > fp.max_accel) {
                        fp.max_accel = fp.s_dd.back();
                    }
                }
                frenet_paths_list.push_back(fp);
            }
        }
        return frenet_paths_list;
    }
    void convertPathsToCartesianCoordinates(
        std::vector<FrenetPath>& paths_list,
        const Input& input) {
            double iyaw = 0.0;
            for (auto path = paths_list.begin(); path != paths_list.end(); ++path) {
                for (size_t i = 0; i < path->s.size(); ++i) {
                    if (path->s.at(i) >= input.csp->s.back() ||
                        path->s.at(i) < 0) {
                            break;
                    }
                    auto p = input.csp->calc_position(path->s.at(i));
                    std::array<double,2> position = {{p.at(0),p.at(1)}};
                    iyaw = input.csp->calc_yaw(path->s.at(i));
                    double di = path->d.at(i);
                    double x = position.at(0) + di * std::cos(iyaw + M_PI_2);
                    double y = position.at(1) + di * std::sin(iyaw + M_PI_2);
                    path->x.push_back(x);
                    path->y.push_back(y);
                }
                if (path->x.size() == 1) {
                    break;
                }
                for (size_t j = 0; j < path->x.size() - 1; ++j) {
                    double dx = path->x.at(j+1) - path->x.at(j);
                    double dy = path->y.at(j+1) - path->y.at(j);
                    path->yaw.push_back(std::atan2(dy,dx));
                    path->ds.push_back(std::sqrt(dx*dx+dy*dy));
                }
                path->yaw.push_back(path->yaw.back());
                path->ds.push_back(path->ds.back());
                path->max_curvature = std::numeric_limits<double>::min();
                for (size_t k = 0; k < path->x.size() - 1; ++k) {
                    if (path->yaw.at(k+1) - path->yaw.at(k) == 0.0 &&
                        path->ds.at(k) == 0.0) {
                            path->k.push_back(0.0);
                    } else {
                        path->k.push_back((path->yaw.at(k+1) - path->yaw.at(k)) /
                            path->ds.at(k));
                    }
                    if (path->k.back() > path->max_curvature) {
                        path->max_curvature = path->k.back();
                    }
                }
                path->k.push_back(path->k.back());

                if (!isPathInCollision(*path,input)) {
                    calculatePathCost(*path,input);
                }
            }
            return;
    }
    std::optional<FrenetPath> findBestPath(std::vector<FrenetPath>& paths_list) {
        std::optional<FrenetPath> best_path;
        paths_list.erase(
        std::remove_if(paths_list.begin(),
                       paths_list.end(),
                       [](const auto& i) { return i.costTotal == 0.0; }),
        paths_list.end());
        double min_cost = std::numeric_limits<double>::max();
        for (auto& path : paths_list) {
            if (min_cost >= path.costTotal) {
                min_cost = path.costTotal;
                best_path = path;
            }
        }
        return best_path;
    }
    std::optional<FrenetPath> planFrenetPath(const Input& input) {
        auto paths_list = calculatePathsInFrenetCoordinates(input);
        CHECK_GT(paths_list.size(), 0) << "No list of paths found";
        convertPathsToCartesianCoordinates(
            paths_list,
            input);
        auto best_path = findBestPath(paths_list);
        CHECK(best_path) << "No path found";
        return best_path;
    }
} // namespace Frenet