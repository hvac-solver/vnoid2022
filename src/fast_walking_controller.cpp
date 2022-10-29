#include "fast_walking_controller.hpp"
#include "actuated_joints.hpp"

#include <cnoid/BasicSensors>
#include <cnoid/Body>
#include <cnoid/EigenTypes>
#include <cnoid/ExecutablePath>
#include <cnoid/SimpleController>

#include <yaml-cpp/yaml.h>

using cnoid::Matrix3;
using cnoid::Vector3;
using cnoid::Vector6;
using cnoid::VectorX;

void FastWalkingController::initMPC(const FastWalkingParams& fast_walking_params, const MPCParams& mpc_params)
{
    robotoc::RobotModelInfo model_info; 
    model_info.urdf_path = cnoid::shareDir() + "/model/sample_robot_description/urdf/sample_robot_reduced.urdf";
    model_info.base_joint_type = robotoc::BaseJointType::FloatingBase;
    const double baumgarte_time_step = 0.05;
    model_info.surface_contacts = {robotoc::ContactModelInfo("L_FOOT_R", baumgarte_time_step),
                                   robotoc::ContactModelInfo("R_FOOT_R", baumgarte_time_step)};
    robotoc::Robot robot(model_info);

    foot_step_planner_ = std::make_shared<robotoc::BipedWalkFootStepPlanner>(robot);
    if (fast_walking_params.use_raibert) {
        const Eigen::Vector3d vcom_cmd = 0.5 * fast_walking_params.step_length 
                                            / (fast_walking_params.swing_time+fast_walking_params.double_support_time);
        const double yaw_rate_cmd = fast_walking_params.step_yaw / fast_walking_params.swing_time;
        foot_step_planner_->setRaibertGaitPattern(vcom_cmd, yaw_rate_cmd, fast_walking_params.swing_time, 
                                                   fast_walking_params.double_support_time, fast_walking_params.raibert_gain);
    }
    else {
        foot_step_planner_->setGaitPattern(fast_walking_params.step_length, fast_walking_params.step_yaw, 
                                           (fast_walking_params.double_support_time > 0.));
    }

    mpc_ = robotoc::MPCBipedWalk(robot, mpc_params.T, mpc_params.N);
    mpc_.setGaitPattern(foot_step_planner_, fast_walking_params.step_height, fast_walking_params.swing_time, 
                        fast_walking_params.double_support_time, fast_walking_params.swing_start_time);

    const double X = 0.08;
    const double Y = 0.04;
    mpc_.getContactWrenchConeHandle()->setRectangular(X, Y);
    mpc_.getImpactWrenchConeHandle()->setRectangular(X, Y);

    const double t0 = 0.0;
    Eigen::VectorXd q0(robot.dimq());
    q0 << 0, 0, 0, 0, 0, 0, 1,
          0, // left sholder
          0, // right sholder
          0, 0, -0.5*fast_walking_params.knee_angle, fast_walking_params.knee_angle, -0.5*fast_walking_params.knee_angle, 0, // left leg
          0, 0, -0.5*fast_walking_params.knee_angle, fast_walking_params.knee_angle, -0.5*fast_walking_params.knee_angle, 0; // right leg
    robot.updateFrameKinematics(q0);
    q0[2] = - 0.5 * (robot.framePosition("L_FOOT_R")[2] + robot.framePosition("R_FOOT_R")[2]) + fast_walking_params.height_offset;
    const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(robot.dimv());
    robotoc::SolverOptions option_init;
    option_init.max_iter = 200;
    option_init.nthreads = mpc_params.nthreads;
    mpc_.init(t0, q0, v0, option_init);

    robotoc::SolverOptions option_mpc;
    option_mpc.max_iter = mpc_params.iter;
    option_mpc.nthreads = mpc_params.nthreads;
    mpc_.setSolverOptions(option_mpc);
}


bool FastWalkingController::initialize(cnoid::SimpleControllerIO* io)
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
    jointIds_.clear();
    for (const auto& e : actuatedJointNames) {
        jointIds_.push_back(io->body()->joint(e.c_str())->jointId());
    }

    /*** MPC initialization ***/
    const std::string config_path = cnoid::shareDir() + "/project/fast_walking_controller.config.cnoid";
    const YAML::Node config = YAML::LoadFile(config_path);

    FastWalkingParams fast_walking_params;
    fast_walking_params.loadFromYAML(config["fast_walking_params"]);
    fast_walking_params.check();
    std::cout << fast_walking_params << std::endl;

    mpc_params_ = MPCParams();
    mpc_params_.loadFromYAML(config["mpc_params"]);
    mpc_params_.check();
    std::cout << mpc_params_ << std::endl;
    initMPC(fast_walking_params, mpc_params_);

    return true;
}

bool FastWalkingController::start()
{
    t_ = 0.0;
    mpc_inner_loop_count_ = mpc_params_.sim_steps_per_mpc_update - 1;
    return true;
}

bool FastWalkingController::control()
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

    // applies the MPC inputs
    if (mpc_inner_loop_count_ == 0) {
        mpc_.updateSolution(t_, dt_, q, v);
        const auto& u = mpc_.getInitialControlInput();
        // applies the inputs
        for (int i=0; i<jointIds_.size(); ++i) {
            ioBody_->joint(jointIds_[i])->u() = u.coeff(i);
        }
        mpc_inner_loop_count_ = mpc_params_.sim_steps_per_mpc_update - 1;
    }
    else {
        const auto policy = mpc_.getControlPolicy(t_);
        const Eigen::VectorXd u = policy.tauJ - policy.Kp * (policy.qJ - q.tail(jointIds_.size()))
                                              - policy.Kd * (policy.dqJ - v.tail(jointIds_.size()));
        // applies the inputs
        for (int i=0; i<jointIds_.size(); ++i) {
            ioBody_->joint(jointIds_[i])->u() = u.coeff(i);
        }
        --mpc_inner_loop_count_;
    }

    t_ += dt_;

    return true;
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(FastWalkingController)