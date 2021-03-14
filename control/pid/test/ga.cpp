#include "pid/pid.h"
#include "ga.h"
#include <random>
#include <memory>
#include <chrono>

constexpr int MAX_GENERATION = 300;
constexpr int MAX_ITERATION = 4500;
constexpr double KP_MAX = 20.0;
constexpr double KI_MAX = 1.0;
constexpr double KD_MAX = 20.0;


int main(int argc, char** argv) {

    std::vector<double> wx {-2.5,0.0,2.5,5.0,7.5,3.0,-1.0};
    std::vector<double> wy {0.7,-6.0,2.0,-4.0,0.0,5.0,-2.0};

    std::vector<std::vector<State>> states;
    std::vector<double> rx,ry,ryaw;
    CubicSplinePlanner::Spline2D csp(wx,wy);
    for (double i=0; i<csp.s.back(); i+=0.1) {
        auto p = csp.calc_position(i);
        rx.push_back(p.at(0));
        ry.push_back(p.at(1));
        ryaw.push_back(csp.calc_yaw(i));
    }

    State init;
    init.x = rx.front();
    init.y = ry.front();
    init.theta = std::atan2((ry.at(1)-ry.at(0)),(rx.at(1)-rx.at(0)));
    init.v = 0.1;
    std::unique_ptr<RobotModel> rm;
    int generation = 1;
    Population pop;
    pop.chromosomes.resize(100);

    std::mt19937 rng;
    rng.seed(std::random_device{}());
    std::uniform_real_distribution<double> dist_p(0.0,20.0);
    std::uniform_real_distribution<double> dist_i(0.0,1.0);
    std::uniform_real_distribution<double> dist_d(0.0,20.0);

    for (size_t i = 0; i < pop.chromosomes.size(); i++) {
        pop.chromosomes.at(i).Kp = dist_p(rng);
        pop.chromosomes.at(i).Ki = dist_i(rng)*0.00001;
        pop.chromosomes.at(i).Kd = dist_d(rng);
        pop.chromosomes.at(i).fitness = 0.0;
    }

    std::unique_ptr<Pid> pid;
    std::vector<double> offtrack_errors;
    std::vector<Chromosome> best_gains;
    bool goal = false;

    while (generation < MAX_GENERATION) {
        for (size_t i = 0; i < pop.chromosomes.size(); i++) {
            auto start = std::chrono::high_resolution_clock::now();
            int iteration = 0;
            offtrack_errors.clear();
            std::vector<State> single_run;
            while (iteration < MAX_ITERATION) {
                if (iteration == 0) {
                    rm = std::make_unique<RobotModel>(init);
                    pid = std::make_unique<Pid>(pop.chromosomes.at(i).Kp,
                        pop.chromosomes.at(i).Ki,
                        pop.chromosomes.at(i).Kd);
                }
                double velocity = 0.1;
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = duration_cast<std::chrono::microseconds>(stop - start);
                auto track_error = findClosestIndex(pid->closest_index,rm->state_,rx,ry);
                stop = std::chrono::high_resolution_clock::now();
                duration = duration_cast<std::chrono::microseconds>(stop - start);

                pid->closest_index = track_error.first;

                auto steer = pid->calcError(rm->state_,track_error,ryaw);

                if (steer < -60*M_PI/180) {
                    steer = -60*M_PI/180;
                } else if (steer > 60  *M_PI/180) {
                    steer = 60 *M_PI/180;
                }
                rm->updateState(steer,velocity);

                auto dist_from_goal = std::sqrt((rm->state_.x - rx.back())*(rm->state_.x - rx.back()) +
                    (rm->state_.y - ry.back())*(rm->state_.y - ry.back()));
                double curr_closest_dist = std::sqrt((rm->state_.x - rx.at(track_error.first))*(rm->state_.x - rx.at(track_error.first)) +
                    (rm->state_.y - ry.at(track_error.first))*(rm->state_.y - ry.at(track_error.first)));
                offtrack_errors.push_back(curr_closest_dist);

                if (dist_from_goal < 0.1) {
                    goal = true;
                    break;
                }
                single_run.push_back(rm->state_);
                iteration++;
                stop = std::chrono::high_resolution_clock::now();
                duration = duration_cast<std::chrono::microseconds>(stop - start);
            }
            pid.reset();
            rm.reset();
            states.push_back(single_run);
            pop.chromosomes.at(i).fitness = *std::max_element(offtrack_errors.begin(),offtrack_errors.end());
        }
        std::stable_sort(pop.chromosomes.begin(),pop.chromosomes.end(), [](const auto& a, const auto& b) {
            return  a.fitness < b.fitness;
        });
        std::vector<Chromosome> parents {pop.chromosomes.at(0),pop.chromosomes.at(1)};

        std::cout << "############  FITNESS: "<< " " << pop.chromosomes.at(0).fitness << "############" << std::endl;

        best_gains.push_back(pop.chromosomes.at(0));
        for (size_t i = 0; i < pop.chromosomes.size(); i++) {
            double cross_idx = static_cast<double> (rand()) / static_cast<double> (RAND_MAX); // crossover index
            double cross_kp = cross_idx * parents.at(0).Kp + (1-cross_idx) * parents.at(1).Kp;
            double cross_ki = cross_idx * parents.at(0).Ki + (1-cross_idx) * parents.at(1).Ki;
            double cross_kd = cross_idx * parents.at(0).Kd + (1-cross_idx) * parents.at(1).Kd;
            double mut_idx = static_cast<double> (rand()) / static_cast<double> (RAND_MAX);
            double mut_kp = cross_kp + (KP_MAX*mut_idx);
            double mut_ki = cross_ki + (KI_MAX*mut_idx*0.00001);
            double mut_kd = cross_kd + (KD_MAX*mut_idx);
            pop.chromosomes.at(i).Kp = mut_kp;
            pop.chromosomes.at(i).Ki = mut_ki;
            pop.chromosomes.at(i).Kd = mut_kd;
            pop.chromosomes.at(i).fitness = 0.0;
        }
        generation++;
    }

    std::stable_sort(best_gains.begin(),best_gains.end(), [](const auto& a, const auto& b) {
            return  a.fitness < b.fitness;
    });

    std::cout << "Best fitness: " << best_gains.at(0).fitness << std::endl;
    std::cout << "Best chromosome: " << best_gains.at(0).Kp << " , "
        << best_gains.at(0).Ki << " , "
        << best_gains.at(0).Kd << " , "
        << best_gains.at(0).fitness << std::endl;

    return 0;
}
