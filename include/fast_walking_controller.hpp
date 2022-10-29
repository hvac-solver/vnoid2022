#pragma once

#include <cnoid/BasicSensors>
#include <cnoid/Body>
#include <cnoid/SimpleController>

#include <robotoc/mpc/biped_walk_foot_step_planner.hpp>
#include <robotoc/mpc/mpc_biped_walk.hpp>

#include "Eigen/Core"

#include <vector>
#include <string>
#include <memory>

#include "mpc_params.hpp"
#include "fast_walking_params.hpp"

class FastWalkingController : public cnoid::SimpleController
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

    // MPC solver
    robotoc::MPCBipedWalk mpc_;
    std::shared_ptr<robotoc::BipedWalkFootStepPlanner> foot_step_planner_;

    void initMPC(const FastWalkingParams& walking_params, const MPCParams& mpc_params);

public:
    bool initialize(cnoid::SimpleControllerIO* io) override;
    bool start() override;
    bool control() override;
};