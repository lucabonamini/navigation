#pragma once

#include "dwa/types.h"
#include <models/unicycle.h>
#include <memory>
#include <string>

class DWA {
public:
DWA(const Config &config,
    const Obstacles &obstacles,
    const ::types::Point &goal,
    const ::types::State &init_state);
~DWA(){};
static Config parseConfigFile(const std::string &config_file);
bool dwaControls();
std::unique_ptr<::model::Unicycle> unicycle_;
private:
DynamicWindow calcDynamicWindow();
::types::Controls calcBestControls(const DynamicWindow &dw);
::types::Traj calcTrajectory(const double &lin_vel,
    const double &ang_vel,
    const ::types::State &state);
double calcTrajectoryCost(const ::types::Traj &trajectory);
double calcToGoalCost(const ::types::Traj &trajectory);
double calcSpeedCost(const double& last_lin_vel);
double calcObstacleCost(const ::types::Traj &trajectory);
bool isGoalReached();
Config config_;
Obstacles obstacles_;
::types::Point goal_;
::types::State state_;
};