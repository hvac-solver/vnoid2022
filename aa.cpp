#include "stair_climbing_mpc.hpp"

#include "Eigen/Core"

#include <cnoid/BasicSensors>
#include <cnoid/Body>
#include <cnoid/EigenTypes>
#include <cnoid/ExecutablePath>
#include <cnoid/SimpleController>

#include <robotoc/constraints/constraints.hpp>
#include <robotoc/constraints/joint_position_lower_limit.hpp>
#include <robotoc/constraints/joint_position_upper_limit.hpp>
#include <robotoc/constraints/joint_torques_lower_limit.hpp>
#include <robotoc/constraints/joint_torques_upper_limit.hpp>
#include <robotoc/constraints/joint_velocity_lower_limit.hpp>
#include <robotoc/constraints/joint_velocity_upper_limit.hpp>
#include <robotoc/constraints/wrench_friction_cone.hpp>
#include <robotoc/constraints/impulse_wrench_friction_cone.hpp>
#include <robotoc/cost/configuration_space_cost.hpp>
#include <robotoc/cost/cost_function.hpp>
#include <robotoc/cost/periodic_com_ref.hpp>
#include <robotoc/cost/periodic_foot_track_ref.hpp>
#include <robotoc/cost/task_space_6d_cost.hpp>
#include <robotoc/cost/time_varying_task_space_3d_cost.hpp>
#include <robotoc/hybrid/contact_sequence.hpp>
#include <robotoc/ocp/ocp.hpp>
#include <robotoc/robot/robot.hpp>

using cnoid::Matrix3;
using cnoid::Vector3;
using cnoid::Vector6;
using cnoid::VectorX;

bool StairClimbingMPC::initialize()
{
    /*** MPC initialization ***/
    // creates a robot model used for MPC
    const std::string urdf_path
        = cnoid::shareDir()
          + "/model/sample_robot/sample_robot_fixed_upper.urdf";
    const int L_foot_id = 54;
    const int R_foot_id = 68;
    const std::vector<int> contact_frames = {L_foot_id, R_foot_id};
    const std::vector<robotoc::ContactType>
        contact_types(contact_frames.size(),
                      robotoc::ContactType::SurfaceContact);
    // const double baumgarte_timestep = 0.05;
    const double baumgarte_timestep = 0.1;
    // const double baumgarte_timestep = 0.5;
    robotoc::Robot robot(urdf_path,
                         robotoc::BaseJointType::FloatingBase,
                         contact_frames,
                         contact_types,
                         baumgarte_timestep);

    // registers robot poses
    const double knee_angle = M_PI / 3.0;
    // const double knee_angle = M_PI / 6.0;
    // const double knee_angle = M_PI / 12.0;
    // const Eigen::Vector3d step_length = {0.4, 0, 0.2};
    // const Eigen::Vector3d step_length = {0.35, 0, 0.2};
    const Eigen::Vector3d step_length = {0.3, 0, 0.2};
    const double swing_height = 0.4; // for stair
    const double yaw_rate = 0;
    const double swing_time = 0.5;
    // const double double_support_time = 0.05;
    const double double_support_time = 0.0;
    const double initial_lift_time = 0.5;

    VectorX q_standing = VectorX::Zero(19);
    q_standing(6) = 1.0;
    // left leg
    q_standing(9) = -0.5 * knee_angle;
    q_standing(10) = knee_angle;
    q_standing(11) = -0.5 * knee_angle;
    // right leg
    q_standing(15) = -0.5 * knee_angle;
    q_standing(16) = knee_angle;
    q_standing(17) = -0.5 * knee_angle;
    robot.updateKinematics(q_standing);

    const double height_offset = 0.05;
    // const double height_offset = 0.0;
    const double height = -0.5
                          * (robot.framePosition(L_foot_id)[2]
                             + robot.framePosition(R_foot_id)[2]) + height_offset;
    q_standing[2] = height;
    robot.updateKinematics(q_standing);

    // defines costs
    std::shared_ptr<robotoc::CostFunction> cost(new robotoc::CostFunction());
    VectorX q_weight(18);
    q_weight << 0.0, 0.0, 0.0, 1000.0, 1000.0, 1000.0, 
                0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 
                0.001, 0.001, 0.001, 0.001, 0.001, 0.001;
    // q_weight << 0.0, 0.0, 0.0, 1000.0, 1000.0, 1000.0, 0.001, 0.001, 0.001,
    //     0.001, 100.0, 100.0, 0.001, 0.001, 0.001, 0.001, 100.0, 100.0;
    const VectorX qf_weight = q_weight;
    const VectorX v_weight = VectorX::Ones(robot.dimv());
    const VectorX u_weight = VectorX::Ones(robot.dimu()) * 1.0e-03;
    VectorX qi_weight(18);
    qi_weight << 0.0, 0.0, 0.0, 1000.0, 1000.0, 1000.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    const VectorX vi_weight = VectorX::Ones(robot.dimv());
    const VectorX dvi_weight = VectorX::Ones(robot.dimv()) * 1.0e-03;
    std::shared_ptr<robotoc::ConfigurationSpaceCost> config_cost(
        new robotoc::ConfigurationSpaceCost(robot));
    config_cost->set_q_ref(q_standing);
    config_cost->set_q_weight(q_weight);
    config_cost->set_qf_weight(q_weight);
    config_cost->set_v_weight(v_weight);
    config_cost->set_vf_weight(v_weight);
    config_cost->set_u_weight(u_weight);
    config_cost->set_qi_weight(qi_weight);
    config_cost->set_vi_weight(vi_weight);
    config_cost->set_dvi_weight(dvi_weight);
    cost->push_back(config_cost);

    const Vector3 x3d0_L = robot.framePosition(L_foot_id);
    const Vector3 x3d0_R = robot.framePosition(R_foot_id);
    const double L_t0 = initial_lift_time + swing_time + double_support_time;
    const double R_t0 = initial_lift_time;
    L_foot_ref_ = std::make_shared<robotoc::VnoidPeriodicFootTrackRef>(
        x3d0_L, step_length, swing_height, L_t0, swing_time, 
        swing_time + 2.0 * double_support_time, false);
    R_foot_ref_ = std::make_shared<robotoc::VnoidPeriodicFootTrackRef>(
        x3d0_R, step_length, swing_height, R_t0, swing_time, 
        swing_time + 2.0 * double_support_time, false);
    std::shared_ptr<robotoc::TimeVaryingTaskSpace3DCost> L_cost(
        new robotoc::TimeVaryingTaskSpace3DCost(robot, L_foot_id, L_foot_ref_));
    std::shared_ptr<robotoc::TimeVaryingTaskSpace3DCost> R_cost(
        new robotoc::TimeVaryingTaskSpace3DCost(robot, R_foot_id, R_foot_ref_));
    const Vector3 foot_track_weight = {1.0e04, 1.0e04, 1.0e04};
    L_cost->set_x3d_weight(foot_track_weight);
    R_cost->set_x3d_weight(foot_track_weight);
    cost->push_back(L_cost);
    cost->push_back(R_cost);

    Vector3 com_ref0 = robot.CoM();
    com_ref0.coeffRef(2) += 0.05;
    const Vector3 vcom_ref = 0.5 * step_length / swing_time;
    com_ref_  = std::make_shared<robotoc::VnoidPeriodicCoMRef>(com_ref0,
                                                               vcom_ref,
                                                               initial_lift_time,
                                                               swing_time,
                                                               double_support_time,
                                                               false);
                                                            //    true);
    std::shared_ptr<robotoc::TimeVaryingCoMCost> com_cost(
        new robotoc::TimeVaryingCoMCost(robot, com_ref_));
    com_cost->set_com_weight({1.0e04, 1.0e04, 1.0e03});
    cost->push_back(com_cost);

    // creates constraints
    std::shared_ptr<robotoc::Constraints> constraints(
        new robotoc::Constraints(1.0e-03, 0.995));
    std::shared_ptr<robotoc::JointPositionLowerLimit> joint_position_lower(
        new robotoc::JointPositionLowerLimit(robot));
    std::shared_ptr<robotoc::JointPositionUpperLimit> joint_position_upper(
        new robotoc::JointPositionUpperLimit(robot));
    std::shared_ptr<robotoc::JointVelocityLowerLimit> joint_velocity_lower(
        new robotoc::JointVelocityLowerLimit(robot));
    std::shared_ptr<robotoc::JointVelocityUpperLimit> joint_velocity_upper(
        new robotoc::JointVelocityUpperLimit(robot));
    std::shared_ptr<robotoc::JointTorquesLowerLimit> joint_torques_lower(
        new robotoc::JointTorquesLowerLimit(robot));
    std::shared_ptr<robotoc::JointTorquesUpperLimit> joint_torques_upper(
        new robotoc::JointTorquesUpperLimit(robot));
    // const double mu = 0.3;
    // const double rect_X = 0.06;
    // const double rect_Y = 0.03;
    const double mu = 0.4;
    const double rect_X = 0.08;
    const double rect_Y = 0.04;
    std::shared_ptr<robotoc::WrenchFrictionCone> wrench_friction_cone(
        new robotoc::WrenchFrictionCone(robot, mu, rect_X, rect_Y));
    std::shared_ptr<robotoc::ImpulseWrenchFrictionCone> impulse_wrench_friction_cone(
        new robotoc::ImpulseWrenchFrictionCone(robot, mu, rect_X, rect_Y));
    constraints->push_back(joint_position_lower);
    constraints->push_back(joint_position_upper);
    constraints->push_back(joint_velocity_lower);
    constraints->push_back(joint_velocity_upper);
    constraints->push_back(joint_torques_lower);
    constraints->push_back(joint_torques_upper);
    constraints->push_back(wrench_friction_cone);
    constraints->push_back(impulse_wrench_friction_cone);

    // defines the optimal control problem
    const double T = 0.5;
    const int N = 20;  
    const int max_steps = 3;
    robotoc::OCP ocp = robotoc::OCP(robot, cost, constraints, T, N, max_steps);

    foot_step_planner_ = std::make_shared<robotoc::VnoidFootStepPlanner>(robot);
    const int num_planning_steps = 40;
    // const double step_height = 0.2;
    const double step_height = 0.0;
    foot_step_planner_->setGaitPattern(step_length, step_height, (yaw_rate*swing_time), num_planning_steps, (double_support_time > 0.));

    const int nthreads = 4;
    std::shared_ptr<robotoc::MPCWalking> mpc(
        new robotoc::MPCWalking(ocp, nthreads));
    mpc_ = mpc;
    mpc_->setGaitPattern(foot_step_planner_, 
                         swing_time,
                         double_support_time,
                         initial_lift_time);
    const VectorX q = q_standing;
    const VectorX v = VectorX::Zero(robot.dimv());
    const double t = 0.0;

    auto option_init = robotoc::SolverOptions::defaultOptions();
    option_init.max_iter = 200;
    mpc_->init(t, q, v, option_init);

    auto option_mpc = robotoc::SolverOptions::defaultOptions();
    option_mpc.max_iter = 1;
    mpc_->setSolverOptions(option_mpc);

    t_ = 0.0;
    dt_ = 0.001;

    return true;
}

bool StairClimbingMPC::start()
{
    t_ = 0.0;
    dt_ = 0.001;
    return true;
}

Eigen::VectorXd StairClimbingMPC::control(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const bool debug)
{
    // applies the MPC inputs
    mpc_->updateSolution(t_, dt_, q, v);
    const VectorX u = mpc_->getInitialControlInput();

    if (debug) {
        // prints to debug...
        std::cout << "t: " << t_ << std::endl;
        std::cout << "q: " << q.transpose() << std::endl;
        // std::cout << "v: " << v.transpose() << std::endl;
        // std::cout << "u: " << u.transpose() << std::endl;
        std::cout << "KKT error: " << mpc_->KKTError() << std::endl;
    }

    t_ += dt_;

    return u;
}