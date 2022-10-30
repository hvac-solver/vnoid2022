#include "athletic_controller.hpp"
#include "actuated_joints.hpp"
#include "mpc_jump_com_ref.hpp"

#include <cnoid/BasicSensors>
#include <cnoid/Body>
#include <cnoid/EigenTypes>
#include <cnoid/ExecutablePath>
#include <cnoid/SimpleController>

#include <robotoc/cost/com_cost.hpp>
#include <robotoc/cost/periodic_com_ref.hpp>
#include <robotoc/cost/periodic_swing_foot_ref.hpp>

#include <yaml-cpp/yaml.h>

using cnoid::Matrix3;
using cnoid::Vector3;
using cnoid::Vector6;
using cnoid::VectorX;

void AthleticController::initMPCStairClimbing(const StairClimbingParams& climbing_params, const MPCParams& mpc_params)
{
    robotoc::RobotModelInfo model_info; 
    model_info.urdf_path = cnoid::shareDir() + "/model/sample_robot_description/urdf/sample_robot_reduced.urdf";
    // model_info.urdf_path = cnoid::shareDir() + "/model/sample_robot_description/urdf/sample_robot_fixed_upper_body.urdf";
    model_info.base_joint_type = robotoc::BaseJointType::FloatingBase;
    const double baumgarte_time_step = 0.05;
    model_info.surface_contacts = {robotoc::ContactModelInfo("L_FOOT_R", baumgarte_time_step),
                                   robotoc::ContactModelInfo("R_FOOT_R", baumgarte_time_step)};
    robotoc::Robot robot(model_info);

    stair_climbing_foot_step_planner_ = std::make_shared<robotoc::StairClimbingFootStepPlanner>(robot);
    const int extra_floor_steps = 3;
    stair_climbing_foot_step_planner_->setGaitPattern(climbing_params.stair_step_length, climbing_params.num_stair_steps,
                                                      climbing_params.floor_step_length, climbing_params.num_floor_steps+extra_floor_steps);

    mpc_stair_climbing_ = robotoc::MPCBipedWalk(robot, mpc_params.T, mpc_params.N);
    mpc_stair_climbing_.setGaitPattern(stair_climbing_foot_step_planner_, climbing_params.stair_step_height, 
                                       climbing_params.swing_time, climbing_params.double_support_time, climbing_params.swing_start_time);

    mpc_stair_climbing_.getConfigCostHandle()->set_u_weight(Eigen::VectorXd::Constant(robot.dimu(), 1.0e-03));
    mpc_stair_climbing_.getConfigCostHandle()->set_dv_weight_impact(Eigen::VectorXd::Constant(robot.dimv(), 1.0e-03));
    mpc_stair_climbing_.getSwingFootCostHandle()[0]->set_weight(Eigen::Vector3d::Constant(1.0e03));
    mpc_stair_climbing_.getSwingFootCostHandle()[1]->set_weight(Eigen::Vector3d::Constant(1.0e03));
    mpc_stair_climbing_.getCoMCostHandle()->set_weight((Eigen::Vector3d() << 1.0e04, 1.0e04, 1.0e03).finished());

    const double X = 0.08;
    const double Y = 0.04;
    mpc_stair_climbing_.getContactWrenchConeHandle()->setRectangular(X, Y);
    mpc_stair_climbing_.getImpactWrenchConeHandle()->setRectangular(X, Y);

    const double t0 = climbing_params.initial_time;
    Eigen::VectorXd q0(robot.dimq());
    q0 << climbing_params.initial_base_position(0), climbing_params.initial_base_position(1), 0, // base position
          0, 0, 0, 1, // base orientation
          0, // left sholder
          0, // right sholder
          0, 0, -0.5*climbing_params.knee_angle, climbing_params.knee_angle, -0.5*climbing_params.knee_angle, 0, // left leg
          0, 0, -0.5*climbing_params.knee_angle, climbing_params.knee_angle, -0.5*climbing_params.knee_angle, 0; // right leg
    robot.updateFrameKinematics(q0);
    q0[2] = - 0.5 * (robot.framePosition("L_FOOT_R")[2] + robot.framePosition("R_FOOT_R")[2]) + climbing_params.height_offset; // base height
    const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(robot.dimv());

    robotoc::SolverOptions option_init;
    option_init.max_iter = 200;
    option_init.nthreads = mpc_params.nthreads;
    mpc_stair_climbing_.init(t0, q0, v0, option_init);

    robotoc::SolverOptions option_mpc;
    option_mpc.max_iter = mpc_params.iter;
    option_mpc.nthreads = mpc_params.nthreads;
    mpc_stair_climbing_.setSolverOptions(option_mpc);
}

void AthleticController::initMPCJump(const JumpParams& jump_params, const MPCParams& mpc_params)
{
    robotoc::RobotModelInfo model_info; 
    model_info.urdf_path = cnoid::shareDir() + "/model/sample_robot_description/urdf/sample_robot_reduced.urdf";
    // model_info.urdf_path = cnoid::shareDir() + "/model/sample_robot_description/urdf/sample_robot_fixed_upper_body.urdf";
    model_info.base_joint_type = robotoc::BaseJointType::FloatingBase;
    const double baumgarte_time_step = 0.05;
    model_info.surface_contacts = {robotoc::ContactModelInfo("L_FOOT_R", baumgarte_time_step),
                                   robotoc::ContactModelInfo("R_FOOT_R", baumgarte_time_step)};
    robotoc::Robot robot(model_info);

    jump_foot_step_planner_ = std::make_shared<robotoc::JumpFootStepPlanner>(robot);
    jump_foot_step_planner_->setJumpPattern(jump_params.jump_length, 0);

    mpc_jump_ = robotoc::MPCJump(robot, mpc_params.T, mpc_params.N);
    mpc_jump_.setJumpPattern(jump_foot_step_planner_, jump_params.flying_time, jump_params.flying_time, 
                             jump_params.ground_time, jump_params.ground_time);

    mpc_jump_.getConfigCostHandle()->set_u_weight(Eigen::VectorXd::Constant(robot.dimu(), 1.0e-03));

    const double X = 0.08;
    const double Y = 0.04;
    mpc_jump_.getContactWrenchConeHandle()->setRectangular(X, Y);
    // mpc_jump_.getImpactWrenchConeHandle()->setRectangular(X, Y);

    const double t0 = jump_params.initial_time;
    Eigen::VectorXd q0(robot.dimq());
    q0 << jump_params.initial_base_position(0), jump_params.initial_base_position(1), jump_params.initial_base_position(2), // base position
          0, 0, 0, 1, // base orientation
          0, // left sholder
          0, // right sholder
          0, 0, -0.5*jump_params.knee_angle, jump_params.knee_angle, -0.5*jump_params.knee_angle, 0, // left leg
          0, 0, -0.5*jump_params.knee_angle, jump_params.knee_angle, -0.5*jump_params.knee_angle, 0; // right leg
    const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(robot.dimv());

    auto config_cost = mpc_jump_.getConfigCostHandle();
    Eigen::VectorXd q_weight = Eigen::VectorXd::Constant(robot.dimv(), 0.01);
    q_weight.template head<6>() << 0, 0, 0, 100, 100, 100;
    Eigen::VectorXd q_weight_impact = Eigen::VectorXd::Constant(robot.dimv(), 10);
    q_weight_impact.template head<6>() << 0, 0, 0, 1000, 1000, 1000;
    config_cost->set_q_weight(q_weight);
    config_cost->set_q_weight_terminal(q_weight);
    config_cost->set_q_weight_impact(q_weight_impact);
    config_cost->set_v_weight(Eigen::VectorXd::Constant(robot.dimv(), 1.0));
    config_cost->set_v_weight_terminal(Eigen::VectorXd::Constant(robot.dimv(), 1.0));
    config_cost->set_u_weight(Eigen::VectorXd::Constant(robot.dimu(), 1.0e-03));
    config_cost->set_a_weight(Eigen::VectorXd::Constant(robot.dimv(), 0.0));
    config_cost->set_v_weight_impact(Eigen::VectorXd::Constant(robot.dimv(), 10.0));
    config_cost->set_dv_weight_impact(Eigen::VectorXd::Constant(robot.dimv(), 0.0));

    const double jump_start_time = jump_params.initial_time + mpc_params.T - jump_params.ground_time;
    auto com_ref = std::make_shared<robotoc::MPCJumpCoMRef>(jump_start_time, jump_params.flying_time);
    robot.updateFrameKinematics(q0);
    com_ref->setCoMRef(robot.CoM(), jump_params.jump_length, jump_params.jump_height);
    auto com_cost = std::make_shared<robotoc::CoMCost>(robot, com_ref);
    com_cost->set_weight(Eigen::Vector3d::Constant(1.0e03));
    com_cost->set_weight_terminal(Eigen::Vector3d::Constant(1.0e03));
    com_cost->set_weight_impact(Eigen::Vector3d::Constant(1.0e03));
    mpc_jump_.getCostFunctionHangle()->push_back(com_cost);

    robotoc::SolverOptions option_init;
    option_init.max_iter = 100;
    option_init.nthreads = mpc_params.nthreads;
    option_init.enable_line_search = true;
    option_init.line_search_settings.line_search_method = robotoc::LineSearchMethod::MeritBacktracking;
    option_init.line_search_settings.min_step_size = 1.0e-03;
    mpc_jump_.init(t0, q0, v0, option_init);

    robotoc::SolverOptions option_mpc;
    option_mpc.max_iter = mpc_params.iter;
    option_mpc.nthreads = mpc_params.nthreads;
    mpc_jump_.setSolverOptions(option_mpc);
}

bool AthleticController::initialize(cnoid::SimpleControllerIO* io)
{
    // initializes variables
    dt_ = io->timeStep();
    ioBody_ = io->body();

    /*** robot initialization ***/

    // enable interfaces to get the root link states
    io->enableInput(ioBody_->rootLink(),
                    cnoid::Link::JointDisplacement | cnoid::Link::JointVelocity
                        | cnoid::Link::LinkPosition | cnoid::Link::LinkTwist);

    // drives joints with torque input
    for (auto joint : ioBody_->joints()) {
        // input setting
        io->enableInput(joint,
                        cnoid::Link::JointDisplacement
                            | cnoid::Link::JointVelocity);
        // output setting
        joint->setActuationMode(cnoid::Link::JOINT_TORQUE);
        io->enableOutput(joint);
    }

    // gets the actuated joint ids
    const auto actuatedJointNames = getActuatedJointNamesOfReducedModel();
    // const auto actuatedJointNames = getActuatedJointNamesOfFixedUpperBodyModel();
    jointIds_.clear();
    for (const auto& e : actuatedJointNames) {
        jointIds_.push_back(io->body()->joint(e.c_str())->jointId());
    }

    /*** MPC initialization ***/
    const std::string config_path = cnoid::shareDir() + "/project/athletic_controller.config.cnoid";
    const YAML::Node config = YAML::LoadFile(config_path);

    stair_climbing_params_ = StairClimbingParams();
    stair_climbing_params_.loadFromYAML(config["stair_climbing_params"]);
    stair_climbing_params_.check();
    std::cout << stair_climbing_params_ << std::endl;

    mpc_stair_climbing_params_ = MPCParams();
    mpc_stair_climbing_params_.loadFromYAML(config["mpc_stair_climbing_params"]);
    mpc_stair_climbing_params_.check();
    std::cout << mpc_stair_climbing_params_ << std::endl;
    initMPCStairClimbing(stair_climbing_params_, mpc_stair_climbing_params_);

    jump_params_ = JumpParams();
    jump_params_.loadFromYAML(config["jump_params"]);
    jump_params_.check();
    std::cout << jump_params_ << std::endl;

    mpc_jump_params_ = MPCParams();
    mpc_jump_params_.loadFromYAML(config["mpc_jump_params"]);
    mpc_jump_params_.check();
    std::cout << mpc_jump_params_ << std::endl;
    initMPCJump(jump_params_, mpc_jump_params_);

    return true;
}

bool AthleticController::start()
{
    t_ = 0.0;
    mpc_inner_loop_count_ = mpc_stair_climbing_params_.sim_steps_per_mpc_update - 1;
    control_mode_ = ControlMode::Stair;
    return true;
}

bool AthleticController::control()
{
    // gets the root pose
    const cnoid::LinkPtr rootLink = ioBody_->rootLink();
    const Vector3 p = rootLink->p();
    const Matrix3 R = rootLink->R();
    const Eigen::Quaterniond r_orig = Eigen::Quaterniond(R);
    const cnoid::Vector4 r = {r_orig.x(), r_orig.y(), r_orig.z(), r_orig.w()};

    // gets the root velocities
    Vector6 v_root;
    v_root << R.transpose() * rootLink->v(), R.transpose() * rootLink->w();

    // gets the configuration and generalized velocity
    VectorX q(jointIds_.size()+7);
    VectorX v(jointIds_.size()+6);
    q.template head<7>() << p, r;
    v.template head<6>() = v_root;
    for (int i=0; i<jointIds_.size(); ++i) {
        q.coeffRef(i+7) = ioBody_->joint(jointIds_[i])->q();
        v.coeffRef(i+6) = ioBody_->joint(jointIds_[i])->dq();
    }

    switch (control_mode_)
    {
    case ControlMode::Stair: {
        const double stair_end_time = stair_climbing_params_.initial_time + stair_climbing_params_.swing_start_time
                                        + 2 * stair_climbing_params_.num_stair_steps * stair_climbing_params_.swing_time;
        if (t_-dt_ < stair_end_time && t_ >= stair_end_time) {
            mpc_stair_climbing_.setGaitPattern(stair_climbing_foot_step_planner_, stair_climbing_params_.floor_step_height, 
                                               stair_climbing_params_.swing_time, stair_climbing_params_.double_support_time, 
                                               stair_climbing_params_.swing_start_time);
        }
        if (mpc_inner_loop_count_ == 0) {
            mpc_stair_climbing_.updateSolution(t_, dt_, q, v);
            const auto& u = mpc_stair_climbing_.getInitialControlInput();
            // applies the inputs
            for (int i=0; i<jointIds_.size(); ++i) {
                ioBody_->joint(jointIds_[i])->u() = u.coeff(i);
            }
            mpc_inner_loop_count_ = mpc_stair_climbing_params_.sim_steps_per_mpc_update - 1;
        }
        else {
            const auto policy = mpc_stair_climbing_.getControlPolicy(t_);
            const Eigen::VectorXd u = policy.tauJ - policy.Kp * (policy.qJ - q.tail(jointIds_.size()))
                                                - policy.Kd * (policy.dqJ - v.tail(jointIds_.size()));
            // applies the inputs
            for (int i=0; i<jointIds_.size(); ++i) {
                ioBody_->joint(jointIds_[i])->u() = u.coeff(i);
            }
            --mpc_inner_loop_count_;
        }
        break;
    }
    case ControlMode::Jump: {
        if (mpc_inner_loop_count_ == 0) {
            const auto policy = mpc_jump_.getControlPolicy(t_);
            const Eigen::VectorXd u = policy.tauJ;
            // mpc_jump_.updateSolution(t_, dt_, q, v);
            // const auto& u = mpc_jump_.getInitialControlInput();
            // applies the inputs
            for (int i=0; i<jointIds_.size(); ++i) {
                ioBody_->joint(jointIds_[i])->u() = u.coeff(i);
            }
            mpc_inner_loop_count_ = mpc_jump_params_.sim_steps_per_mpc_update - 1;
        }
        else {
            const auto policy = mpc_jump_.getControlPolicy(t_);
            const Eigen::VectorXd u = policy.tauJ;
            // const Eigen::VectorXd u = policy.tauJ - policy.Kp * (policy.qJ - q.tail(jointIds_.size()))
            //                                       - policy.Kd * (policy.dqJ - v.tail(jointIds_.size()));
            std::cout << "u: " << u.transpose() << std::endl;
            // applies the inputs
            for (int i=0; i<jointIds_.size(); ++i) {
                ioBody_->joint(jointIds_[i])->u() = u.coeff(i);
            }
            --mpc_inner_loop_count_;
        }
        break;
    }
    default: {
        for (int i=0; i<jointIds_.size(); ++i) {
            ioBody_->joint(jointIds_[i])->u() = 0;
        }
        break;
    }
    } // switch

    t_ += dt_;
    if (t_ < (stair_climbing_params_.initial_time 
                + stair_climbing_params_.swing_start_time
                + 2 * stair_climbing_params_.num_stair_steps * stair_climbing_params_.swing_time
                + 2 * stair_climbing_params_.num_floor_steps * stair_climbing_params_.swing_time)) {
        control_mode_ = ControlMode::Stair;
    }
    else {
        if (control_mode_ == ControlMode::Stair) {
            mpc_inner_loop_count_ = 0;
        }
        control_mode_ = ControlMode::Jump;
    }

    return true;
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AthleticController)