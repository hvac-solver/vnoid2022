#include "athletic_controller.hpp"
#include "actuated_joints.hpp"

#include <cnoid/BasicSensors>
#include <cnoid/Body>
#include <cnoid/EigenTypes>
#include <cnoid/ExecutablePath>
#include <cnoid/SimpleController>

using cnoid::Matrix3;
using cnoid::Vector3;
using cnoid::Vector6;
using cnoid::VectorX;

void AthleticController::initStairClimbingMPC(const StairClimbingParams& climbing_params, const MPCParams& mpc_params)
{
    robotoc::RobotModelInfo model_info; 
    model_info.urdf_path = cnoid::shareDir() + "/model/sample_robot_description/urdf/sample_robot_reduced.urdf";
    model_info.base_joint_type = robotoc::BaseJointType::FloatingBase;
    // const double baumgarte_time_step = 0.05;
    const double baumgarte_time_step = 0.1;
    model_info.surface_contacts = {robotoc::ContactModelInfo("L_FOOT_R", baumgarte_time_step),
                                   robotoc::ContactModelInfo("R_FOOT_R", baumgarte_time_step)};
    robotoc::Robot robot(model_info);

    mpc_stair_climbing_ = robotoc::MPCBipedWalk(robot, mpc_params.T, mpc_params.N);

    stair_climbing_foot_step_planner_ = std::make_shared<robotoc::StairClimbingFootStepPlanner>(robot);
    stair_climbing_foot_step_planner_->setGaitPattern(climbing_params.step_length, climbing_params.num_stair_steps);
    mpc_stair_climbing_.setGaitPattern(stair_climbing_foot_step_planner_, climbing_params.step_height, 
                                       climbing_params.swing_time, climbing_params.double_support_time, climbing_params.swing_start_time);

    mpc_stair_climbing_.getConfigCostHandle()->set_u_weight(Eigen::VectorXd::Constant(robot.dimu(), 1.0e-03));
    mpc_stair_climbing_.getConfigCostHandle()->set_dv_weight_impact(Eigen::VectorXd::Constant(robot.dimv(), 1.0e-03));
    mpc_stair_climbing_.getCoMCostHandle()->set_weight((Eigen::Vector3d() << 1.0e04, 1.0e04, 1.0e03).finished());

    const double X = 0.08;
    const double Y = 0.04;
    mpc_stair_climbing_.getContactWrenchConeHandle()->setRectangular(X, Y);
    mpc_stair_climbing_.getImpactWrenchConeHandle()->setRectangular(X, Y);

    const double t0 = climbing_params.initial_time;
    Eigen::VectorXd q0(robot.dimq());
    q0 << climbing_params.initial_base_position(0), climbing_params.initial_base_position(1), climbing_params.initial_base_position(2), // base position
          0, 0, 0, 1, // base orientation
          0, // left sholder
          0, // right sholder
          0, 0, -0.5*climbing_params.knee_angle, climbing_params.knee_angle, -0.5*climbing_params.knee_angle, 0, // left leg
          0, 0, -0.5*climbing_params.knee_angle, climbing_params.knee_angle, -0.5*climbing_params.knee_angle, 0; // right leg
    robot.updateFrameKinematics(q0);
    q0[2] = - 0.5 * (robot.framePosition("L_FOOT_R")[2] + robot.framePosition("R_FOOT_R")[2]);
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
    const auto actuatedJointNames = getActuatedJointNames();
    jointIds_.clear();
    for (const auto& e : actuatedJointNames) {
        jointIds_.push_back(io->body()->joint(e.c_str())->jointId());
    }

    /*** MPC initialization ***/
    StairClimbingParams stair_climbing_params;
    MPCParams mpc_params;
    mpc_params.T = 0.5;
    mpc_params.N = 20;
    mpc_params.iter = 1;
    mpc_params.nthreads = 6;
    initStairClimbingMPC(stair_climbing_params, mpc_params);

    return true;
}

bool AthleticController::start()
{
    t_ = 0.0;
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

    // TODO: introduce switch-case or if to switch the controller
    // stair climbing 
    mpc_stair_climbing_.updateSolution(t_, dt_, q, v);
    const auto& u = mpc_stair_climbing_.getInitialControlInput();

    // applies the inputs
    for (int i=0; i<jointIds_.size(); ++i) {
        ioBody_->joint(jointIds_[i])->u() = u.coeff(i);
    }

    t_ += dt_;

    return true;
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AthleticController)