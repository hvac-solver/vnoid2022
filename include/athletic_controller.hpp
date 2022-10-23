#pragma once

#include <cnoid/BasicSensors>
#include <cnoid/Body>
#include <cnoid/SimpleController>

#include <robotoc/mpc/biped_walk_foot_step_planner.hpp>
#include <robotoc/mpc/mpc_biped_walk.hpp>

#include "athletic_controller.hpp"

#include "Eigen/Core"

#include <vector>
#include <string>
#include <memory>

#include "mpc_params.hpp"
#include "stair_climbing_foot_step_planner.hpp"

struct StairClimbingParams 
{
    // double knee_angle = M_PI / 6.0;
    double knee_angle = M_PI / 3.0;

    Eigen::Vector3d step_length = {0.3, 0.0, 0.2}; 

    double step_height = 0.4;
    double swing_time = 0.5;
    double double_support_time = 0.0; // must be zero with the current StairClimbingFootStepPlanner implementation.
    double swing_start_time = 0.5;

    int num_stair_steps = 10;

    double initial_time = 0.0;
    Eigen::Vector3d initial_base_position = {5.35, 1.5, 0.8}; 
};


class AthleticController : public cnoid::SimpleController
{
private:
    // interfaces for a simulated body
    cnoid::Body* ioBody_;

    // actuated joints
    std::vector<int> jointIds_;

    // parameters
    double dt_;
    double t_;

    // MPC solvers
    robotoc::MPCBipedWalk mpc_stair_climbing_;
    std::shared_ptr<robotoc::StairClimbingFootStepPlanner> stair_climbing_foot_step_planner_;

    void initStairClimbingMPC(const StairClimbingParams& stair_climbing_params, const MPCParams& mpc_params);

public:
    bool initialize(cnoid::SimpleControllerIO* io) override;
    bool start() override;
    bool control() override;
};