#include "mpc_biped_jump.hpp"

#include <stdexcept>
#include <iostream>
#include <cassert>
#include <cmath>
#include <limits>
#include <algorithm>


namespace robotoc {

MPCBipedJump::MPCBipedJump(const Robot& robot, const double T, const int N)
  : foot_step_planner_(),
    contact_sequence_(std::make_shared<robotoc::ContactSequence>(robot, 1)),
    cost_(std::make_shared<CostFunction>()),
    constraints_(std::make_shared<Constraints>(1.0e-03, 0.995)),
    ocp_solver_(OCP(robot, cost_, constraints_, contact_sequence_, T, N), 
                SolverOptions()), 
    solver_options_(SolverOptions()),
    cs_ground_(robot.createContactStatus()),
    cs_flying_(robot.createContactStatus()),
    cs_right_flying_(robot.createContactStatus()), 
    cs_left_flying_(robot.createContactStatus()),
    s_(),
    jump_height_(0), 
    ground_time_(0), 
    flying_up_time_(0), 
    flying_time_(0), 
    landing_time_(0),
    T_(T),
    dt_(T/N),
    dtm_(T/N),
    eps_(std::sqrt(std::numeric_limits<double>::epsilon())),
    N_(N),
    current_step_(0) {
  // create costs
  config_cost_ = std::make_shared<ConfigurationSpaceCost>(robot);
  Eigen::VectorXd q_weight = Eigen::VectorXd::Constant(robot.dimv(), 0.01);
  q_weight.template head<6>() << 0, 0, 0, 100, 100, 100;
  Eigen::VectorXd q_weight_impact = Eigen::VectorXd::Constant(robot.dimv(), 10);
  q_weight_impact.template head<6>() << 0, 0, 0, 1000, 1000, 1000;
  config_cost_->set_q_weight(q_weight);
  config_cost_->set_q_weight_terminal(q_weight);
  config_cost_->set_q_weight_impact(q_weight_impact);
  config_cost_->set_v_weight(Eigen::VectorXd::Constant(robot.dimv(), 1.0));
  config_cost_->set_v_weight_terminal(Eigen::VectorXd::Constant(robot.dimv(), 1.0));
  config_cost_->set_v_weight_impact(Eigen::VectorXd::Constant(robot.dimv(), 10.0));
  config_cost_->set_u_weight(Eigen::VectorXd::Constant(robot.dimu(), 1.0e-02));
  cost_->add("config_cost", config_cost_);
  com_cost_ = std::make_shared<CoMCost>(robot);
  com_cost_->set_weight(Eigen::Vector3d::Constant(1.0e04));
  com_cost_->set_weight_terminal(Eigen::Vector3d::Constant(1.0e04));
  L_foot_cost_ = std::make_shared<TaskSpace3DCost>(robot, robot.contactFrames()[0]);
  L_foot_cost_->set_weight(Eigen::Vector3d::Constant(1.0e03));
  R_foot_cost_ = std::make_shared<TaskSpace3DCost>(robot, robot.contactFrames()[1]);
  R_foot_cost_->set_weight(Eigen::Vector3d::Constant(1.0e03));

  L_foot_rot_cost_ = std::make_shared<TaskSpace6DCost>(robot, robot.contactFrames()[0], SE3::Identity());
  R_foot_rot_cost_ = std::make_shared<TaskSpace6DCost>(robot, robot.contactFrames()[1], SE3::Identity());
  // create constraints 
  auto joint_position_lower = std::make_shared<robotoc::JointPositionLowerLimit>(robot);
  auto joint_position_upper = std::make_shared<robotoc::JointPositionUpperLimit>(robot);
  auto joint_velocity_lower = std::make_shared<robotoc::JointVelocityLowerLimit>(robot);
  auto joint_velocity_upper = std::make_shared<robotoc::JointVelocityUpperLimit>(robot);
  auto joint_torques_lower  = std::make_shared<robotoc::JointTorquesLowerLimit>(robot);
  auto joint_torques_upper  = std::make_shared<robotoc::JointTorquesUpperLimit>(robot);
  constraints_->add("joint_position_lower", joint_position_lower);
  constraints_->add("joint_position_upper", joint_position_upper);
  constraints_->add("joint_velocity_lower", joint_velocity_lower);
  constraints_->add("joint_velocity_upper", joint_velocity_upper);
  constraints_->add("joint_torques_lower", joint_torques_lower);
  constraints_->add("joint_torques_upper", joint_torques_upper);
  // const double X = 0.08;
  // const double Y = 0.04;
  const double X = 0.06;
  const double Y = 0.03;
  contact_wrench_cone_ = std::make_shared<ContactWrenchCone>(robot, X, Y);
  constraints_->add("contact_wrench_cone", contact_wrench_cone_);
  // init contact status
  cs_ground_.activateContact(0);
  cs_ground_.activateContact(1);
  cs_left_flying_.activateContact(1);
  cs_right_flying_.activateContact(0);
  const double friction_coefficient = 0.4;
  cs_ground_.setFrictionCoefficients(std::vector<double>(cs_ground_.maxNumContacts(), friction_coefficient));
  cs_left_flying_.setFrictionCoefficients(std::vector<double>(cs_ground_.maxNumContacts(), friction_coefficient));
  cs_right_flying_.setFrictionCoefficients(std::vector<double>(cs_ground_.maxNumContacts(), friction_coefficient));
}


MPCBipedJump::MPCBipedJump() {
}


MPCBipedJump::~MPCBipedJump() {
}


void MPCBipedJump::setJumpPattern(
    const std::shared_ptr<ContactPlannerBase>& foot_step_planner, 
    const double jump_height, const double ground_time, const double flying_up_time, 
    const double flying_time, const double landing_time) {
  // if (flying_time <= 0) {
  //   throw std::out_of_range("[MPCBipedJump] invalid argument: 'flying_time' must be positive!");
  // }
  // if (min_flying_time <= 0) {
  //   throw std::out_of_range("[MPCBipedJump] invalid argument: 'min_flying_time' must be positive!");
  // }
  // if (ground_time <= 0) {
  //   throw std::out_of_range("[MPCBipedJump] invalid argument: 'ground_time' must be positive!");
  // }
  // if (min_ground_time <= 0) {
  //   throw std::out_of_range("[MPCBipedJump] invalid argument: 'min_ground_time' must be positive!");
  // }
  // if (flying_time+ground_time > T_) {
  //   throw std::out_of_range("[MPCBipedJump] invalid argument: 'flying_time' + 'ground_time' must be less than T!");
  // }
  // if (min_flying_time+min_ground_time > T_) {
  //   throw std::out_of_range("[MPCBipedJump] invalid argument: 'min_flying_time' + 'min_ground_time' must be less than T!");
  // }
  foot_step_planner_ = foot_step_planner;
  jump_height_ = jump_height;
  ground_time_ = ground_time;
  flying_up_time_ = flying_up_time;
  flying_time_ = flying_time;
  landing_time_ = landing_time;
}


void MPCBipedJump::init(const double t, const Eigen::VectorXd& q, 
                        const Eigen::VectorXd& v, 
                        const SolverOptions& solver_options) {
  current_step_ = 0;
  double ts = t;
  contact_sequence_->init(cs_ground_);
  ts += ground_time_;
  contact_sequence_->push_back(cs_left_flying_, ts);
  ts += flying_up_time_;
  contact_sequence_->push_back(cs_flying_, ts);
  ts += flying_time_;
  contact_sequence_->push_back(cs_right_flying_, ts);
  ts += landing_time_;
  contact_sequence_->push_back(cs_ground_, ts);

  foot_step_planner_->init(q);
  Eigen::VectorXd q_ref = q;
  q_ref.template head<3>().noalias() += (foot_step_planner_->CoM(4) 
                                          - foot_step_planner_->CoM(0));
  q_ref.template segment<4>(3) = Eigen::Quaterniond(foot_step_planner_->R(4)).coeffs();
  config_cost_->set_q_ref(q_ref);
  auto com_ref = std::make_shared<MPCJumpCoMRef>(t+ground_time_+flying_up_time_, 
                                                 flying_time_+landing_time_);
  const Eigen::Vector3d jump_length = foot_step_planner_->CoM(4) - foot_step_planner_->CoM(0);
  com_ref->setCoMRef(foot_step_planner_->CoM(0), jump_length, jump_height_);
  com_cost_->set_ref(com_ref);
  // cost_->add("com_cost", com_cost_);

  const double swing_height = 1.0;
  auto L_foot_ref = std::make_shared<PeriodicSwingFootRef>(foot_step_planner_->contactPositions(0)[0],
                                                           jump_length, swing_height,
                                                           t+ground_time_, 
                                                           flying_up_time_+flying_time_, 1.0e03, false);
  auto R_foot_ref = std::make_shared<PeriodicSwingFootRef>(foot_step_planner_->contactPositions(0)[1],
                                                           jump_length, swing_height,
                                                           t+ground_time_+flying_up_time_, 
                                                           flying_time_+landing_time_, 1.0e03, false);
  L_foot_cost_->set_ref(L_foot_ref);
  R_foot_cost_->set_ref(R_foot_ref);
  cost_->add("L_foot_cost", L_foot_cost_);
  cost_->add("R_foot_cost", R_foot_cost_);

  // L_foot_rot_cost_->set_weight(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(1.0e02));
  // R_foot_rot_cost_->set_weight(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(1.0e02));
  L_foot_rot_cost_->set_weight(Eigen::Vector3d::Constant(1.0e01), Eigen::Vector3d::Zero());
  R_foot_rot_cost_->set_weight(Eigen::Vector3d::Constant(1.0e01), Eigen::Vector3d::Zero());
  cost_->add("L_foot_rot_cost", L_foot_rot_cost_);
  cost_->add("R_foot_rot_cost", R_foot_rot_cost_);

  resetContactPlacements(t, q, v);
  ocp_solver_.setSolution("q", q);
  ocp_solver_.setSolution("v", v);
  ocp_solver_.setSolverOptions(solver_options);
  ocp_solver_.solve(t, q, v, true);
  std::cout << ocp_solver_.getSolverStatistics() << std::endl;
  s_ = ocp_solver_.getSolution();
}


void MPCBipedJump::reset() {
  ocp_solver_.setSolution(s_);
}


void MPCBipedJump::reset(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  ocp_solver_.setSolution(s_);
  ocp_solver_.setSolution("q", q);
  ocp_solver_.setSolution("v", v);
}


void MPCBipedJump::setSolverOptions(const SolverOptions& solver_options) {
  ocp_solver_.setSolverOptions(solver_options);
}


void MPCBipedJump::updateSolution(const double t, const double dt,
                                  const Eigen::VectorXd& q, 
                                  const Eigen::VectorXd& v) {
  assert(dt > 0);
  const auto ts = contact_sequence_->eventTimes();
  bool remove_step = false;
  if (!ts.empty()) {
    if (ts.front()+eps_ < t+dt) {
      contact_sequence_->pop_front();
      remove_step = true;
      ++current_step_;
    }
  }
  resetContactPlacements(t, q, v);
  ocp_solver_.solve(t, q, v, true);
}


const Eigen::VectorXd& MPCBipedJump::getInitialControlInput() const {
  return ocp_solver_.getSolution(0).u;
}


const Solution& MPCBipedJump::getSolution() const {
  return ocp_solver_.getSolution();
}


const aligned_vector<LQRPolicy>& MPCBipedJump::getLQRPolicy() const {
  return ocp_solver_.getLQRPolicy();
}


double MPCBipedJump::KKTError(const double t, const Eigen::VectorXd& q, 
                              const Eigen::VectorXd& v) {
  return ocp_solver_.KKTError(t, q, v);
}


double MPCBipedJump::KKTError() const {
  return ocp_solver_.KKTError();
}


std::shared_ptr<CostFunction> MPCBipedJump::getCostHandle() {
  return cost_;
}


std::shared_ptr<ConfigurationSpaceCost> MPCBipedJump::getConfigCostHandle() {
  return config_cost_;
}


std::shared_ptr<Constraints> MPCBipedJump::getConstraintsHandle() {
  return constraints_;
}


std::shared_ptr<ContactWrenchCone> MPCBipedJump::getContactWrenchConeHandle() {
  return contact_wrench_cone_;
}


void MPCBipedJump::setRobotProperties(const RobotProperties& properties) {
  ocp_solver_.setRobotProperties(properties);
}


void MPCBipedJump::resetContactPlacements(const double t, const Eigen::VectorXd& q,
                                          const Eigen::VectorXd& v) {
  const bool success = foot_step_planner_->plan(t, q, v, contact_sequence_->contactStatus(0),
                                                contact_sequence_->numContactPhases());
  for (int i=0; i<contact_sequence_->numContactPhases(); ++i) {
    contact_sequence_->setContactPlacements(i, foot_step_planner_->contactPlacements(i+1));
  }
}

} // namespace robotoc 