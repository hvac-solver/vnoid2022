#include "fast_walking_controller.hpp"

#include <cnoid/BasicSensors>
#include <cnoid/Body>
#include <cnoid/EigenTypes>
#include <cnoid/ExecutablePath>
#include <cnoid/SimpleController>

using cnoid::Matrix3;
using cnoid::Vector3;
using cnoid::Vector6;
using cnoid::VectorX;

void FastWalkingController::initMPC(const FastWalkingParams& params) 
{
    robotoc::RobotModelInfo model_info; 
    model_info.urdf_path = "../model/sample_robot_description/urdf/sample_robot_reduced.urdf";
    model_info.base_joint_type = robotoc::BaseJointType::FloatingBase;
    const double baumgarte_time_step = 0.05;
    model_info.surface_contacts = {robotoc::ContactModelInfo("L_FOOT_R", baumgarte_time_step),
                                   robotoc::ContactModelInfo("R_FOOT_R", baumgarte_time_step)};
    robotoc::Robot robot(model_info);

    const Eigen::Vector3d vcom_cmd = 0.5 * params.step_length / (params.swing_time+params.double_support_time);
    const double yaw_rate_cmd = params.step_yaw / params.swing_time;

    const double T = 0.7;
    const int N = 25;
    mpc_ = robotoc::MPCBipedWalk(robot, T, N);

    foot_step_planner_ = std::make_shared<robotoc::BipedWalkFootStepPlanner>(robot);
    if (params.use_raibert) {
        foot_step_planner_->setGaitPattern(params.step_length, params.step_yaw, (params.double_support_time > 0.));
    }
    else {
        foot_step_planner_->setRaibertGaitPattern(vcom_cmd, yaw_rate_cmd, params.swing_time, params.double_support_time, params.raibert_gain);
    }
    mpc_.setGaitPattern(foot_step_planner_, params.step_height, params.swing_time, params.double_support_time, params.swing_start_time);

    const double X = 0.08;
    const double Y = 0.04;
    mpc.getContactWrenchConeHandle().setRectangular(X, Y);
    mpc.getImpactWrenchConeHandle().setRectangular(X, Y);

    const double t0 = 0.0;
    Eigen::VectorXd q0;
    q0 << 0, 0, 0, 0, 0, 0, 1,
          0, // left sholder
          0, // right sholder
          0, 0, -0.5*params.knee_angle, params.knee_angle, -0.5*params.knee_angle, 0, // left leg
          0, 0, -0.5*params.knee_angle, params.knee_angle, -0.5*params.knee_angle, 0; // right leg
    robot.forwardKinematics(q0);
    q0[2] = - 0.5 * (robot.framePposition("L_FOOT_R")[2] + robot.framePosition("R_FOOT_R")[2]);
    const Eigen::VectorXd v0 = Eigen::VectorXd::Zeros(robot.dimv());
    robotoc::SolverOptions option_init;
    option_init.max_iter = 200;
    option_init.nthreads = 4;
    mpc_.init(t0, q0, v0, option_init);

    robotoc::SolverOptions option_mpc;
    option_mpc.max_iter = 1;
    option_mpc.nthreads = 4;
    mpc_.setSolverOptions(option_mpc);
}


bool SolverWalkingController::initialize(cnoid::SimpleControllerIO* io)
{
    // initializes variables
    dt_ = io->timeStep();
    ioBody_ = io->body();

    /*** robot initialization ***/

    // turns sensors on
    RFSensor_ = ioBody_->findDevice<cnoid::ForceSensor>("rfsensor");
    LFSensor_ = ioBody_->findDevice<cnoid::ForceSensor>("lfsensor");
    io->enableInput(RFSensor_);
    io->enableInput(LFSensor_);

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

    /*** MPC initialization ***/
    FastWalkingParams params;
    initMPC(params);

    return success;
}

bool FastWalkingController::start()
{
    t_ = 0.0;
    return true;
}

bool FastWalkingController::control()
{
    // gets force sensor values (local)
    const Vector6 rightWrench = RFSensor_->F();
    const Vector6 leftWrench = LFSensor_->F();

    // gets the root pose
    const cnoid::LinkPtr rootLink = ioBody_->rootLink();
    const Vector3 p = rootLink->p();
    const Matrix3 R = rootLink->R();
    const Eigen::Quaterniond r_orig = Eigen::Quaterniond(R);
    const cnoid::Vector4 r = {r_orig.x(), r_orig.y(), r_orig.z(), r_orig.w()};

    // gets the root velocities
    Vector6 v_root;
    v_root << R.transpose() * rootLink->v(), R.transpose() * rootLink->w();

    // gets the joint positions and velocities
    // warning: the order of the left and right leg in Choreonoid
    //          is DIFFERENT from robotoc framework
    VectorX q_joint(14);
    VectorX v_joint(14);
    constexpr int jointIdLeftSholder  = 1;
    constexpr int jointIdRightSholder = 1;

    constexpr int jointIdOffsetRightFoot = 18;
    constexpr int jointIdOffsetLeftFoot = jointIdOffsetRightFoot + 6;
    for (int jointId = jointIdOffsetRightFoot; jointId < jointIdOffsetRightFoot + 6;
         ++jointId) {
        q_joint(jointId - jointIdOffsetRightFoot + 6) = ioBody_->joint(jointId)->q();
        v_joint(jointId - jointIdOffsetRightRoot + 6) = ioBody_->joint(jointId)->dq();
    }

    for (int jointId = jointIdOffsetLeftFoot; jointId < jointIdOffsetLeftFoot + 6;
         ++jointId) {
        q_joint(jointId - jointIdOffsetLeftFoot) = ioBody_->joint(jointId)->q();
        v_joint(jointId - jointIdOffsetLeftFoot) = ioBody_->joint(jointId)->dq();
    }

    // organizes the position and velocity vectors
    VectorX q(p.rows() + r.rows() + q_joint.rows());
    q << p, r, q_joint;
    VectorX v(v_root.rows() + v_joint.rows());
    v << v_root, v_joint;

    // applies the MPC inputs
    mpc_.updateSolution(t_, q, v);
    const auto& u = mpc_.getInitialControlInput();
    for (int jointId = jointIdOffsetRight; jointId < jointIdOffsetRight + 6;
         ++jointId) {
        ioBody_->joint(jointId)->u() = u(jointId - jointIdOffsetRight + 6);
    }
    for (int jointId = jointIdOffsetLeft; jointId < jointIdOffsetLeft + 6;
         ++jointId) {
        ioBody_->joint(jointId)->u() = u(jointId - jointIdOffsetLeft);
    }

    t_ += dt_;

    return true;
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(FastWalkingController)