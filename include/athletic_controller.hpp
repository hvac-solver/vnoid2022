#pragma once

#include <cnoid/BasicSensors>
#include <cnoid/Body>
#include <cnoid/SimpleController>

#include <robotoc/mpc/mpc_biped_walk.hpp>
// #include <robotoc/mpc/mpc_jump.hpp>
// #include <robotoc/mpc/jump_foot_step_planner.hpp>

#include "athletic_controller.hpp"

#include "Eigen/Core"

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

#include "mpc_params.hpp"
#include "stair_climbing_params.hpp"
#include "jump_params.hpp"

#include "stair_climbing_foot_step_planner.hpp"
#include "biped_jump_foot_step_planner.hpp"
#include "mpc_biped_jump.hpp"


class AthleticController : public cnoid::SimpleController
{
private:
    enum class ControlMode {
        Stair, Jump
    };
    ControlMode control_mode_;
    // interfaces for a simulated body
    cnoid::Body* ioBody_;

    // actuated joints
    std::vector<int> jointIds_;

    // parameters
    double dt_;
    double t_;
    int mpc_inner_loop_count_;
    MPCParams mpc_stair_climbing_params_, mpc_jump_params_;
    StairClimbingParams stair_climbing_params_; 
    JumpParams jump_params_; 

    // MPC solvers
    robotoc::MPCBipedWalk mpc_stair_climbing_;
    std::shared_ptr<robotoc::StairClimbingFootStepPlanner> stair_climbing_foot_step_planner_;
    // robotoc::MPCJump mpc_jump_;
    // std::shared_ptr<robotoc::JumpFootStepPlanner> jump_foot_step_planner_;
    robotoc::MPCBipedJump mpc_jump_;
    std::shared_ptr<robotoc::BipedJumpFootStepPlanner> jump_foot_step_planner_;

    void initMPCStairClimbing(const StairClimbingParams& stair_climbing_params, const MPCParams& mpc_params);
    void initMPCJump(const JumpParams& jump_params, const MPCParams& mpc_params);

public:
    bool initialize(cnoid::SimpleControllerIO* io) override;
    bool start() override;
    bool control() override;
};