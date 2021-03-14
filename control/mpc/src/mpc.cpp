#include "mpc/mpc.h"

void MPC::run() {
    while (MAX_ITERATION > iteration_counter_) {
        calcReferenceTrajectory_();
        auto control = solveMpc_();
        updateState(state_,control);
        double dx = state_.x - wx_.back();
        double dy = state_.y - wy_.back();
        states_.push_back(state_);
        ref_chunks_.push_back(ref_);
        if (sqrt(dx*dx + dy*dy) <= 0.2) {
            cout<<("Goal")<<endl;
            break;
        }
        iteration_counter_++;
    }
}

void MPC::calcReferenceTrajectory_() {
        ref_ = REF_M::Zero();
        calcClosestIndex(closest_index_,wx_,wy_,state_);
        ref_(0,0) = wx_.at(closest_index_);
        ref_(1,0) = wy_.at(closest_index_);
        ref_(2,0) = wyaw_.at(closest_index_);
        ref_(3,0) = speed_profile_.at(closest_index_);

        double predicted_distance = 0.0;

        for (size_t i = 0; i < T; i++) {
            predicted_distance += abs(state_.v) * dt;
            size_t dind = static_cast<int>(round(predicted_distance/1.0));
            if ((closest_index_+dind)<wx_.size()) {
                ref_(0, i) = wx_.at(closest_index_ + dind);
                ref_(1, i) = wy_.at(closest_index_ + dind);
                ref_(2, i) = wyaw_.at(closest_index_ + dind);
                ref_(3, i) = speed_profile_.at(closest_index_ + dind);
            } else {
                ref_(0, i) = wx_.at(wx_.size() - 1);
                ref_(1, i) = wy_.at(wx_.size() - 1);
                ref_(2, i) = wyaw_.at(wx_.size() - 1);
                ref_(3, i) = speed_profile_.at(wx_.size() - 1);
            }
        }
}

vector<double> MPC::solveMpc_() {
    typedef CPPAD_TESTVECTOR(double) Dvector;
    double x = state_.x;
    double y = state_.y;
    double yaw = state_.yaw;
    double v = state_.v;

    size_t n_vars = T * 4 + (T - 1) * 2;
    size_t n_constraints = T * 4;

    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++){
        vars[i] = 0.0;
    }

    vars[x_start] = x;
    vars[y_start] = y;
    vars[yaw_start] = yaw;
    vars[v_start] = v;

    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    // NOTE there mush be both lower and upper bounds for all vars!!!!!
    for (size_t i = 0; i < n_vars; i++) {
        vars_lowerbound[i] = -10000000.0;
        vars_upperbound[i] = 10000000.0;
    }

    // for (size_t i = delta_start; i < delta_start+T-1; i++) {
    //     vars_lowerbound[i] = -MAX_STEER;
    //     vars_upperbound[i] = MAX_STEER;
    // }

    for (size_t i = a_start; i < a_start+T-1; i++) {
        vars_lowerbound[i] = -max_acc;
        vars_upperbound[i] = max_acc;
    }

    for (size_t i = v_start; i < v_start+T; i++) {
        vars_lowerbound[i] = 0.0;
        vars_upperbound[i] = max_vel;
    }

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (size_t i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[yaw_start] = yaw;
    constraints_lowerbound[v_start] = v;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[yaw_start] = yaw;
    constraints_upperbound[v_start] = v;

    FG_EVAL fg_eval(ref_);

    // options
    std::string options;
    options += "Integer print_level  0\n";
    // options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Integer max_iter      50\n";
    // options += "Numeric tol          1e-6\n";
    options += "Numeric max_cpu_time          0.05\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_EVAL>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    vector<double> result;
    for (size_t i =0 ; i < n_vars; i++) {
        result.push_back(static_cast<double>(solution.x[i]));
    }
    return result;
}