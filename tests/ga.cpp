#include "pid/pid.h"
#include "ga.h"
#include <random>
#include <memory>
#include "matplotlibcpp.h"

constexpr int MAX_GENERATION = 1;
constexpr int MAX_ITERATION = 2500;

double round(double var) 
{ 
    // 37.66666 * 100 =3766.66 
    // 3766.66 + .5 =3767.16    for rounding off value 
    // then type cast to int so value is 3767 
    // then divided by 100 so the value converted into 37.67 
    double value = (int)(var * 100 + .5); 
    return (double)value / 100; 
} 

int main(int argc, char** argv) {

    std::mt19937 rng;
    rng.seed(std::random_device{}());
    std::uniform_real_distribution<double> dist_p(0.0,20.0);
    std::uniform_real_distribution<double> dist_i(0.0,1.0);
    std::uniform_real_distribution<double> dist_d(0.0,20.0);

    std::vector<double> wx {-2.5,0.0,2.5,5.0,7.5,3.0,-1.0};
    std::vector<double> wy {0.7,-6.0,2.0,-4.0,0.0,5.0,-2.0};
    std::vector<State> states;
    std::vector<double> rx,ry,ryaw;
    CubicSplinePlanner::Spline2D csp(wx,wy);
    for (double i=0; i<csp.s.back(); i+=0.1) {
        auto p = csp.calc_position(i);
        rx.push_back(p.at(0));
        ry.push_back(p.at(1));
        ryaw.push_back(csp.calc_yaw(i));
    }

    // Initial conditions
    State init;
    init.x = rx.front();
    init.y = ry.front();
    init.theta = std::atan2((ry.at(1)-ry.at(0)),(rx.at(1)-rx.at(0)));
    init.v = 0.1;
    std::unique_ptr<RobotModel> rm;
    int generation = 0;
    Population pop;
    pop.chromosomes.resize(100);
    for (size_t i = 0; i < pop.chromosomes.size(); i++) {
        pop.chromosomes.at(i).Kp = dist_p(rng);
        pop.chromosomes.at(i).Ki = dist_i(rng)*0.00001;
        pop.chromosomes.at(i).Kd = dist_d(rng);
        pop.chromosomes.at(i).fitness = 0.0;
    }

    std::unique_ptr<Pid> pid;
    std::vector<double> offtrack_errors;
    std::vector<double> fitness_values;
    bool goal = false;

    while (generation < MAX_GENERATION) {
        generation++;
        int iteration = 0;
        for (size_t i = 0; i < pop.chromosomes.size(); i++) {
            iteration = 0;
            offtrack_errors.clear();
            states.clear();
            while (iteration < MAX_ITERATION) {
                if (iteration == 0) {
                    rm = std::make_unique<RobotModel>(init);
                    pid = std::make_unique<Pid>(pop.chromosomes.at(i).Kp,
                        pop.chromosomes.at(i).Ki,
                        pop.chromosomes.at(i).Kd);
                }
                double velocity = 0.1;
                auto track_error = findClosestIndex(pid->closest_index,rm->state_,rx,ry);
                pid->closest_index = track_error.first;
                
                auto steer = pid->calcError(rm->state_,track_error,ryaw);

                if (steer < -30*M_PI/180) {
                    steer = -30*M_PI/180;
                } else if (steer > 30*M_PI/180) {
                    steer = 30*M_PI/180; 
                }
                rm->updateState(steer,velocity);

                auto dist_from_goal = std::sqrt((rm->state_.x - rx.back())*(rm->state_.x - rx.back()) + (rm->state_.y - ry.back())*(rm->state_.y - ry.back()));
                offtrack_errors.push_back(track_error.second);
                // std::cout << "dist_from_goal: " << dist_from_goal << std::endl;
                if (dist_from_goal < 0.1) {
                    std::cout << "==== GOAL ====" << std::endl;
                    goal = true;
                    break;
                }
                states.push_back(rm->state_);
                iteration++;
            }
            double value = 0.0;
            for (auto err:offtrack_errors) {
                value+=abs(err);
            }
            pop.chromosomes.at(i).fitness = value;
            // if (goal) {
                fitness_values.push_back(pop.chromosomes.at(i).fitness);
            // } else {
            //     fitness_values.push_back(1.0);
            // }
            std::cout << "i: " << i << " , "
                << "goal: " << goal << " , "
                << "Kp: " << pop.chromosomes.at(i).Kp << " , "
                << "Ki: " << pop.chromosomes.at(i).Ki << " , "
                << "Kd: " << pop.chromosomes.at(i).Kd << " , "
                << "fitness: " << fitness_values.back() << std::endl;
        }
        std::stable_sort(pop.chromosomes.begin(),pop.chromosomes.end(), [](const auto& a, const auto& b) {
            return  a.fitness < b.fitness;
        });
        std::vector<Chromosome> parents {pop.chromosomes.at(0),pop.chromosomes.at(1)};
    }

    std::cout << "Best fitness: " << *std::min_element(fitness_values.begin(),fitness_values.end()) << std::endl;

    // std::vector<double> state_x,state_y;
    // for (auto state:states) {
    //     state_x.push_back(state.x);
    //     state_y.push_back(state.y);
    // }
    // matplotlibcpp::plot(rx,ry,"-k");
    // matplotlibcpp::plot(wx,wy,"ob");
    // matplotlibcpp::plot(state_x,state_y,"xr");
    // matplotlibcpp::title("PID Controller");
    // matplotlibcpp::show();

    return 0;
}
