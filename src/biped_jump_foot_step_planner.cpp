#include "biped_jump_foot_step_planner.hpp"
#include "robotoc/utils/rotation.hpp"

#include <stdexcept>
#include <iostream>
#include <cassert>


namespace robotoc {

BipedJumpFootStepPlanner::BipedJumpFootStepPlanner(const Robot& biped_robot)
  : ContactPlannerBase(),
    robot_(biped_robot),
    contact_frames_(biped_robot.contactFrames()),
    current_step_(0),
    L_foot_id_(biped_robot.contactFrames()[0]), 
    R_foot_id_(biped_robot.contactFrames()[1]),
    contact_placement_ref_(),
    contact_position_ref_(),
    contact_surface_ref_(),
    com_ref_(),
    com_to_contact_position_local_(),
    R_(),
    jump_length_(Eigen::Vector3d::Zero()),
    R_yaw_(Eigen::Matrix3d::Identity()),
    L_contact_active_(false), 
    R_contact_active_(false) {
  if (biped_robot.maxNumSurfaceContacts() < 2) {
    throw std::out_of_range(
        "[BipedJumpFootStepPlanner] invalid argument: 'robot' is not a bipedal robot!\n robot.maxNumSurfaceContacts() must be larger than 2!");
  }
}


BipedJumpFootStepPlanner::BipedJumpFootStepPlanner() {
}


BipedJumpFootStepPlanner::~BipedJumpFootStepPlanner() {
}


void BipedJumpFootStepPlanner::setJumpPattern(const Eigen::Vector3d& jump_length,
                                              const double step_yaw) {
  jump_length_ = jump_length;
  R_yaw_ << std::cos(step_yaw), -std::sin(step_yaw), 0, 
          std::sin(step_yaw), std::cos(step_yaw),  0,
          0, 0, 1;
}


void BipedJumpFootStepPlanner::init(const Eigen::VectorXd& q) {
  Eigen::Matrix3d R = rotation::RotationMatrixFromQuaternion(q.template segment<4>(3));
  rotation::ProjectRotationMatrix(R, rotation::ProjectionAxis::Z);
  robot_.updateFrameKinematics(q);
  aligned_vector<SE3> contact_placement;
  for (const auto frame : robot_.contactFrames()) {
    contact_placement.push_back(robot_.framePlacement(frame));
  }
  aligned_vector<SE3> contact_placement_local;
  for (const auto frame : robot_.contactFrames()) {
    contact_placement_local.emplace_back(
        R.transpose() * robot_.frameRotation(frame),
        R.transpose() * (robot_.framePosition(frame) - q.template head<3>()));
  }
  const Eigen::Matrix3d R_goal = R_yaw_ * R;
  const Eigen::Quaterniond quat_goal = Eigen::Quaterniond(R_goal);
  Eigen::VectorXd q_goal = q;
  q_goal.template head<3>().noalias() += R_yaw_ * jump_length_;
  q_goal.template segment<4>(3) = quat_goal.coeffs();
  robot_.updateFrameKinematics(q_goal);
  aligned_vector<SE3> contact_placement_goal;
  for (int i=0; i<robot_.contactFrames().size(); ++i) {
    contact_placement_goal.emplace_back(
        R_goal * contact_placement_local[i].rotation(),
        q.template head<3>() + R_goal * contact_placement_local[i].translation()
                             + R_yaw_ * jump_length_);
  }
  contact_placement_ref_.clear();
  contact_placement_ref_.push_back(contact_placement);
  contact_placement_ref_.push_back(contact_placement);
  contact_placement_ref_.push_back(contact_placement);
  contact_placement_ref_.push_back(contact_placement_goal);
  contact_placement_ref_.push_back(contact_placement_goal);
  contact_placement_ref_.push_back(contact_placement_goal);
  contact_position_ref_.clear();
  contact_surface_ref_.clear();
  for (const auto& e : contact_placement_ref_) {
    contact_position_ref_.push_back(
        std::vector<Eigen::Vector3d>({e[0].translation(), e[1].translation()}));
    contact_surface_ref_.push_back(
        std::vector<Eigen::Matrix3d>({e[0].rotation(), e[1].rotation()}));
  }
  robot_.updateFrameKinematics(q);
  com_ref_.push_back(robot_.CoM());
  com_ref_.push_back(robot_.CoM());
  com_ref_.push_back(robot_.CoM());
  robot_.updateFrameKinematics(q_goal);
  com_ref_.push_back(robot_.CoM());
  com_ref_.push_back(robot_.CoM());
  com_ref_.push_back(robot_.CoM());
  R_.clear();
  R_.push_back(R);
  R_.push_back(R);
  R_.push_back(R);
  R_.push_back(R_goal);
  R_.push_back(R_goal);
  R_.push_back(R_goal);
  current_step_ = 0;
  L_contact_active_ = true;
  R_contact_active_ = true;
}


bool BipedJumpFootStepPlanner::plan(const double t, const Eigen::VectorXd& q,
                                    const Eigen::VectorXd& v,
                                    const ContactStatus& contact_status,
                                    const int planning_steps) {
  assert(planning_steps >= 0);
  if (L_contact_active_ != contact_status.isContactActive(0)
      || R_contact_active_ != contact_status.isContactActive(1)) {
    contact_placement_ref_.pop_front();
    com_ref_.pop_front();
    R_.pop_front();
    contact_position_ref_.pop_front();
    contact_surface_ref_.pop_front();
    --planning_size_;
  }
  L_contact_active_ = contact_status.isContactActive(0);
  R_contact_active_ = contact_status.isContactActive(1);

  robot_.updateFrameKinematics(q);
  contact_placement_ref_[1][0] = robot_.framePlacement(L_foot_id_);
  contact_placement_ref_[1][1] = robot_.framePlacement(R_foot_id_);

  planning_size_ = com_ref_.size();

  return true;
}


const aligned_vector<SE3>& BipedJumpFootStepPlanner::contactPlacements(const int step) const {
  return contact_placement_ref_[step];
}


const std::vector<Eigen::Vector3d>& BipedJumpFootStepPlanner::contactPositions(const int step) const {
  return contact_position_ref_[step];
}


const std::vector<Eigen::Matrix3d>& BipedJumpFootStepPlanner::contactSurfaces(const int step) const {
  return contact_surface_ref_[step];
}


const Eigen::Vector3d& BipedJumpFootStepPlanner::CoM(const int step) const {
  return com_ref_[step];
}


const Eigen::Matrix3d& BipedJumpFootStepPlanner::R(const int step) const {
  return R_[step];
}
  

std::ostream& operator<<(std::ostream& os, 
                         const BipedJumpFootStepPlanner& planner) {
  planner.disp(os);
  return os;
}


std::ostream& operator<<(std::ostream& os, 
                         const std::shared_ptr<BipedJumpFootStepPlanner>& planner) {
  planner->disp(os);
  return os;
}

} // namespace robotoc 