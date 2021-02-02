#ifndef _MPC_H_
#define _MPC_H_

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>

using namespace std;
using CppAD::AD;

constexpr int MAX_ITERATION = 5000;
constexpr size_t T = 6;  // Timestamp length
constexpr size_t N = 4;  // Number of states
constexpr double dt = 0.1;  // Time step
constexpr double max_acc = 1.0;  // m/s^2
constexpr double max_dec = -1.0;
constexpr double max_vel = 3.0;  // m/s

typedef Eigen::Matrix<double,N,T> REF_M;

int x_start = 0;
int y_start = x_start + T;
int yaw_start = y_start + T;
int v_start = yaw_start + T;

int delta_start = v_start + T;
int a_start = delta_start + T-1;

struct State {
    double x;
    double y;
    double yaw;
    double v;
};

struct Control {
    double a;
    double omega;
};

void updateState(State &state,
    const vector<double> &control) {
        state.v = state.v + control.at(a_start) * dt;
        state.yaw = state.yaw + control.at(delta_start) * dt;
        state.x = state.x + state.v * cos(state.yaw) * dt;
        state.y = state.y + state.v * sin(state.yaw) * dt;
}

// Reference: https://arxiv.org/pdf/1902.00606v1.pdf
vector<double> calcSpeedProfile(const vector<double> &s,
    const vector<double> &k) {
        vector<double> tmp1,tmp2,tmp3,speed_profile;
        tmp1.reserve(s.size());
        tmp2.reserve(s.size());
        tmp3.reserve(s.size());
        speed_profile.reserve(s.size());
        double velocity = 0.0;
        // Calc maximum permissible steady state vehicle speed
        for (size_t i = 0; i < k.size(); i++) {
            if (k.at(i) < 0.0) {
                velocity = max_vel;
            } else {
                velocity = sqrt(max_acc / abs(k.at(i)));
                if (velocity > max_vel) {
                    velocity = max_vel;
                }
            }
            tmp1.push_back(velocity);
        }
        // Forward integration step
        for (size_t i = 0; i < tmp1.size()-1; i++) {
            tmp2.push_back(min(sqrt(pow(tmp1.at(i),2)+2*max_acc*s.at(i)),
                tmp1.at(i)));
        }
        // TO DO fix backward step
        // Backward integration step
        for (size_t i = tmp2.size()-1; i > 0; i--) {
            tmp3.push_back(min(sqrt(abs(pow(tmp2.at(i),2)-2*max_acc*s.at(i))),
                tmp2.at(i)));
        }
        reverse(tmp3.begin(),tmp3.end());
        speed_profile = std::move(tmp3);
        return tmp1;
}


int calcClosestIndex(int &closest_index,
    const vector<double> &x,
    const vector<double> &y,
    const State &state) {
        double closest_distance = std::numeric_limits<double>::max();
        for (size_t i = closest_index; i < x.size(); i++) {
            double current_closest_distance = pow(x.at(i)-state.x,2)+
                pow(y.at(i)-state.y,2);
            if (current_closest_distance < closest_distance) {
                closest_distance = current_closest_distance;
                closest_index = i;
            }
        }
        return closest_index;
}

class MPC {
    public:
        MPC(State &init_state,
            vector<double> &wx,
            vector<double> &wy,
            vector<double> &wyaw,
            vector<double> &wk,
            vector<double> &ws,
            vector<double> &speed_profile) : state_(init_state), wx_(wx), wy_(wy), wyaw_(wyaw), wk_(wk), ws_(ws), speed_profile_(speed_profile){};
        void run();
        vector<State> states_;
        vector<REF_M> ref_chunks_;
    private:
        void calcReferenceTrajectory_();
        vector<double> solveMpc_();
        State &state_;
        vector<double> &wx_;
        vector<double> &wy_;
        vector<double> &wyaw_;
        vector<double> &wk_;
        vector<double> &ws_;
        vector<double> &speed_profile_;
        int closest_index_ = 0;
        REF_M ref_;
        int iteration_counter_ = 0;
};

class FG_EVAL{
public:
  REF_M traj_ref_;

  FG_EVAL(REF_M traj_ref){
    traj_ref_ = traj_ref;
  }

  typedef CppAD::vector<CppAD::AD<double>> ADvector;

  void operator()(ADvector& fg, const ADvector& vars){
    fg[0] = 0;

    for(size_t i=0; i<T-1; i++){
      fg[0] +=  0.01 * CppAD::pow(vars[a_start+i], 2);
      fg[0] += 0.01 * CppAD::pow(vars[delta_start+i], 2);
    }

    for(size_t i=0; i<T-2; i++){
      fg[0] += 0.01 * CppAD::pow(vars[a_start+i+1] - vars[a_start+i], 2);
      fg[0] += 1 * CppAD::pow(vars[delta_start+i+1] - vars[delta_start+i], 2);
    }

    // fix the initial state as a constraint
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + yaw_start] = vars[yaw_start];
    fg[1 + v_start] = vars[v_start];

    // fg[0] += CppAD::pow(traj_ref(0, 0) - vars[x_start], 2);
    // fg[0] += CppAD::pow(traj_ref(1, 0) - vars[y_start], 2);
    // fg[0] += 0.5 * CppAD::pow(traj_ref(2, 0) - vars[yaw_start], 2);
    // fg[0] += 0.5 * CppAD::pow(traj_ref(3, 0) - vars[v_start], 2);

    // The rest of the constraints
    for (size_t i = 0; i < T - 1; i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> yaw1 = vars[yaw_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> yaw0 = vars[yaw_start + i];
      AD<double> v0 = vars[v_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      // constraint with the dynamic model
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(yaw0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(yaw0) * dt);
      fg[2 + yaw_start + i] = yaw1 - (yaw0 + delta0 * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      // cost with the ref traj
      fg[0] += CppAD::pow(traj_ref_(0, i+1) - (x0 + v0 * CppAD::cos(yaw0) * dt), 2);
      fg[0] += CppAD::pow(traj_ref_(1, i+1) - (y0 + v0 * CppAD::sin(yaw0) * dt), 2);
      fg[0] += 0.5 * CppAD::pow(traj_ref_(2, i+1) - (yaw0 + delta0 * dt), 2);
      fg[0] += 0.5 * CppAD::pow(traj_ref_(3, i+1) - (v0 + a0 * dt), 2);
    }
  }
};

#endif