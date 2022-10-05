#pragma once

#include <cnoid/BasicSensors>
#include <cnoid/Body>
#include <cnoid/SimpleController>

#include <robotoc/mpc/biped_walk_foot_step_planner.hpp>
#include <robotoc/mpc/mpc_biped_walk.hpp>

#include <memory>

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

    void initMPC();

public:
    bool initialize(cnoid::SimpleControllerIO* io) override;
    bool start() override;
    bool control() override;
};