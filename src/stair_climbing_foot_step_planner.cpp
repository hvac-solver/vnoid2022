#include "stair_climbing_foot_step_planner.hpp"
#include "robotoc/utils/rotation.hpp"

#include <stdexcept>
#include <iostream>
#include <cassert>


namespace robotoc {

StairClimbingFootStepPlanner::StairClimbingFootStepPlanner(const Robot& biped_robot)
  : ContactPlannerBase(),
    robot_(biped_robot),
    L_foot_id_(biped_robot.surfaceContactFrames()[0]),
    R_foot_id_(biped_robot.surfaceContactFrames()[1]),
    current_step_(0),
    left_to_right_leg_distance_(0),
    foot_height_to_com_height_(0),
    contact_placement_ref_(),
    contact_position_ref_(),
    contact_surface_ref_(),
    com_ref_(),
    R_(),
    vcom_(Eigen::Vector3d::Zero()),
    vcom_cmd_(Eigen::Vector3d::Zero()),
    step_length_(Eigen::Vector3d::Zero()),
    R_current_(Eigen::Matrix3d::Identity()),
    enable_double_support_phase_(false),
    L_contact_active_(false), 
    R_contact_active_(false) {
  if (biped_robot.maxNumSurfaceContacts() < 2) {
    throw std::out_of_range(
        "[StairClimbingFootStepPlanner] invalid argument: 'robot' is not a bipedal robot!\n robot.maxNumSurfaceContacts() must be larger than 2!");
  }
}


StairClimbingFootStepPlanner::StairClimbingFootStepPlanner() {
}


StairClimbingFootStepPlanner::~StairClimbingFootStepPlanner() {
}


void StairClimbingFootStepPlanner::setGaitPattern(const Eigen::Vector3d& step_length, 
                                                  const int num_stair_steps,
                                                  const bool enable_double_support_phase) {
  step_length_ = step_length;
  planning_size_ = 2 * (num_stair_steps + 1);
  enable_double_support_phase_ = enable_double_support_phase;
}


void StairClimbingFootStepPlanner::init(const Eigen::VectorXd& q) {
  Eigen::Matrix3d R = rotation::RotationMatrixFromQuaternion(q.template segment<4>(3));
  rotation::ProjectRotationMatrix(R, rotation::ProjectionAxis::Z);
  robot_.updateFrameKinematics(q);
  std::vector<Eigen::Vector3d> contact_position_local 
      = { R.transpose() * (robot_.framePosition(L_foot_id_)-q.template head<3>()),
          R.transpose() * (robot_.framePosition(R_foot_id_)-q.template head<3>()) };
  left_to_right_leg_distance_ = contact_position_local[0].coeff(1)
                                  - contact_position_local[1].coeff(1);
  const double foot_height = 0.5 * (robot_.framePosition(L_foot_id_).coeff(2)
                                      + robot_.framePosition(R_foot_id_).coeff(2));
  foot_height_to_com_height_ = robot_.CoM().coeff(2) - foot_height;
  contact_position_ref_.clear();
  contact_surface_ref_.clear();
  com_ref_.clear(),
  R_.clear();
  R_.push_back(R);
  current_step_ = 0;

  aligned_vector<SE3> contact_placement, contact_placement_local;
  for (const auto frame : robot_.surfaceContactFrames()) {
    contact_placement.push_back(robot_.framePlacement(frame));
    contact_placement_local.emplace_back(
        R.transpose() * robot_.frameRotation(frame),
        R.transpose() * (robot_.framePosition(frame) - q.template head<3>()));
  }

  // plans all steps here
  if (enable_double_support_phase_) {
    throw std::runtime_error(
        "[StairClimbingFootStepPlanner] : the double support phase is not supported!");
  }
  else {
    contact_placement_ref_.clear();
    contact_placement_ref_.push_back(contact_placement); // step -1
    contact_placement_ref_.push_back(contact_placement); // step 0 

    // first 2 steps (on a blue plate)
    contact_placement[1].translation().noalias() += 0.5 * step_length_;
    contact_placement_ref_.push_back(contact_placement); // step 1
    contact_placement[0].translation().noalias() += step_length_;
    contact_placement_ref_.push_back(contact_placement); // step 2

    // next 3 steps
    const double offset = 0.05;
    contact_placement[1].translation().noalias() += step_length_;
    contact_placement[1].translation().coeffRef(2) -= offset; // adjust the height
    contact_placement_ref_.push_back(contact_placement); // step 1
    contact_placement[0].translation().noalias() += step_length_;
    contact_placement[0].translation().coeffRef(2) -= offset; // adjust the height
    contact_placement_ref_.push_back(contact_placement); // step 2
    contact_placement[1].translation().noalias() += step_length_;
    contact_placement[1].translation().coeffRef(2) -= offset; // adjust the height
    contact_placement_ref_.push_back(contact_placement); // step 1

    for (int i=4; i<planning_size_; ++i) {
      if (i%2 != 0) {
        contact_placement[0].translation().noalias() += step_length_;
      }
      else {
        contact_placement[1].translation().noalias() += step_length_;
      }
      contact_placement_ref_.push_back(contact_placement);
    }
  }
  contact_placement_ref_.push_back(contact_placement);
  L_contact_active_ = true;
  R_contact_active_ = true;
}


bool StairClimbingFootStepPlanner::plan(const double t, const Eigen::VectorXd& q,
                                        const Eigen::VectorXd& v,
                                        const ContactStatus& contact_status,
                                        const int planning_steps) {
  assert(planning_steps >= 0);
  if (L_contact_active_ != contact_status.isContactActive(0)
      || R_contact_active_ != contact_status.isContactActive(1)) {
    contact_placement_ref_.pop_front();
    --planning_size_;
  }
  L_contact_active_ = contact_status.isContactActive(0);
  R_contact_active_ = contact_status.isContactActive(1);

  robot_.updateFrameKinematics(q);
  contact_placement_ref_[1][0] = robot_.framePlacement(L_foot_id_);
  contact_placement_ref_[1][1] = robot_.framePlacement(R_foot_id_);

  return true;
}


const aligned_vector<SE3>& StairClimbingFootStepPlanner::contactPlacements(const int step) const {
  return contact_placement_ref_[step];
}


const std::vector<Eigen::Vector3d>& StairClimbingFootStepPlanner::contactPositions(const int step) const {
  return contact_position_ref_[step];
}


const std::vector<Eigen::Matrix3d>& StairClimbingFootStepPlanner::contactSurfaces(const int step) const {
  return contact_surface_ref_[step];
}


const Eigen::Vector3d& StairClimbingFootStepPlanner::CoM(const int step) const {
  return com_ref_[step];
}


const Eigen::Matrix3d& StairClimbingFootStepPlanner::R(const int step) const {
  return R_[step];
}
  

std::ostream& operator<<(std::ostream& os, 
                         const StairClimbingFootStepPlanner& planner) {
  planner.disp(os);
  return os;
}


std::ostream& operator<<(std::ostream& os, 
                         const std::shared_ptr<StairClimbingFootStepPlanner>& planner) {
  planner->disp(os);
  return os;
}

} // namespace robotoc 