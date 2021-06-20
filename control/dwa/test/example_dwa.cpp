#include "dwa/dwa.h"
#include <gflags/gflags.h>

DEFINE_string(config_file, "", "Set path to configuration file.");

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    Obstacles obs = {
        Obstacle{
            .x = -1.0,
            .y = -1.0
        },Obstacle{
            .x = 0.0,
            .y = 2.0
        },Obstacle{
            .x = 4.0,
            .y = 2.0
        },Obstacle{
            .x = 5.0,
            .y = 4.0
        },Obstacle{
            .x = 5.0,
            .y = 5.0
        },Obstacle{
            .x = 5.0,
            .y = 6.0
        },Obstacle{
            .x = 5.0,
            .y = 9.0
        },Obstacle{
            .x = 8.0,
            .y = 9.0
        },Obstacle{
            .x = 7.0,
            .y = 9.0
        },Obstacle{
            .x = 12.0,
            .y = 12.0
        }
    };
    ::types::Point goal {
        .x = 10.0,
        .y = 10.0
    };
    ::types::State init_state {
        .x = 0.0,
        .y = 0.0,
        .yaw = M_PI/8.0,
        .v = 0.0
    };
    DWA dwa(DWA::parseConfigFile(FLAGS_config_file),
        obs,
        goal,
        init_state);
    bool goal_reached = false;
    while (!goal_reached) {
        goal_reached = dwa.dwaControls();
    }
    return 0;
}