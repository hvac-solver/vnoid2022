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
#include <stdexcept>

#include "mpc_params.hpp"
#include "stair_climbing_params.hpp"

#include "stair_climbing_foot_step_planner.hpp"


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
    MPCParams mpc_params_;
    int mpc_inner_loop_count_;

    // MPC solvers
    robotoc::MPCBipedWalk mpc_stair_climbing_;
    std::shared_ptr<robotoc::StairClimbingFootStepPlanner> stair_climbing_foot_step_planner_;

    void initStairClimbingMPC(const StairClimbingParams& stair_climbing_params, const MPCParams& mpc_params);

public:
    bool initialize(cnoid::SimpleControllerIO* io) override;
    bool start() override;
    bool control() override;
};