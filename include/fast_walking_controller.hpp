#pragma once

#include <cnoid/BasicSensors>
#include <cnoid/Body>
#include <cnoid/SimpleController>

#include <robotoc/mpc/biped_walk_foot_step_planner.hpp>
#include <robotoc/mpc/mpc_biped_walk.hpp>

#include "Eigen/Core"

#include <memory>

struct FastWalkingParams 
{
    double knee_angle = M_PI / 6.0;

    Eigen::Vector3d step_length = {0.7, 0.0, 0.0}; 
    double step_yaw = 0.0;

    double step_height = 0.15;
    double swing_time = 0.7;
    double double_support_time = 0.0;
    double swing_start_time = 0.5;

    bool use_raibert = true;
    double raibert_gain = 0.7;
};


class FastWalkingController : public cnoid::SimpleController
{
private:
    // interfaces for a simulated body
    cnoid::Body* ioBody_;
    cnoid::ForceSensorPtr LFSensor_;
    cnoid::ForceSensorPtr RFSensor_;

    // parameters
    double dt_;
    double t_;

    // MPC solver
    robotoc::MPCBipedWalk mpc_;
    std::shared_ptr<robotoc::BipedWalkFootStepPlanner> foot_step_planner_;

    void initMPC(const FastWalkingParams& params);

public:
    bool initialize(cnoid::SimpleControllerIO* io) override;
    bool start() override;
    bool control() override;
};