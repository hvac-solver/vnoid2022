#include "biped_running_foot_step_planner.hpp"
#include "robotoc/utils/rotation.hpp"

#include <stdexcept>
#include <iostream>
#include <cassert>


namespace robotoc {

BipedRunningFootStepPlanner::BipedRunningFootStepPlanner(const Robot& biped_robot)
  : ContactPlannerBase(),
    robot_(biped_robot),
    raibert_heuristic_(),
    vcom_moving_window_filter_(),
    enable_raibert_heuristic_(false),
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
    R_yaw_(Eigen::Matrix3d::Identity()),
    R_current_(Eigen::Matrix3d::Identity()),
    yaw_rate_cmd_(0) {
  if (biped_robot.maxNumSurfaceContacts() < 2) {
    throw std::out_of_range(
        "[BipedRunningFootStepPlanner] invalid argument: 'robot' is not a bipedal robot!\n robot.maxNumSurfaceContacts() must be larger than 2!");
  }
}


BipedRunningFootStepPlanner::BipedRunningFootStepPlanner() {
}


BipedRunningFootStepPlanner::~BipedRunningFootStepPlanner() {
}


void BipedRunningFootStepPlanner::setGaitPattern(const Eigen::Vector3d& step_length, 
                                                 const double step_yaw) {
  step_length_ = step_length;
  R_yaw_<< std::cos(step_yaw), -std::sin(step_yaw), 0, 
           std::sin(step_yaw), std::cos(step_yaw),  0,
           0, 0, 1;
  enable_raibert_heuristic_ = false;
}


void BipedRunningFootStepPlanner::setRaibertGaitPattern(
    const Eigen::Vector3d& vcom_cmd, const double yaw_rate_cmd, 
    const double flying_time, const double stance_time, const double gain) {
  if (flying_time <= 0.0) {
    throw std::out_of_range("[BipedRunningFootStepPlanner] invalid argument: 'flying_time' must be positive!");
  }
  if (stance_time <= 0.0) {
    throw std::out_of_range("[BipedRunningFootStepPlanner] invalid argument: 'stance_time' must be positive!");
  }
  if (gain <= 0.0) {
    throw std::out_of_range("[BipedRunningFootStepPlanner] invalid argument: 'gain' must be positive!");
  }
  const double period = 2.0 * (flying_time + stance_time);
  raibert_heuristic_.setParameters(period, gain);
  vcom_moving_window_filter_.setParameters(period, 0.1*period);
  vcom_cmd_ = vcom_cmd;
  const double step_yaw = yaw_rate_cmd * (flying_time + stance_time);
  R_yaw_<< std::cos(step_yaw), -std::sin(step_yaw), 0, 
           std::sin(step_yaw),  std::cos(step_yaw), 0,
           0, 0, 1;
  yaw_rate_cmd_ = yaw_rate_cmd;
  enable_raibert_heuristic_ = true;
}


void BipedRunningFootStepPlanner::init(const Eigen::VectorXd& q) {
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
  vcom_moving_window_filter_.clear();
  current_step_ = 0;
}


bool BipedRunningFootStepPlanner::plan(const double t, const Eigen::VectorXd& q,
                                       const Eigen::VectorXd& v,
                                       const ContactStatus& contact_status,
                                       const int planning_steps) {
  assert(planning_steps >= 0);
  if (enable_raibert_heuristic_) {
    vcom_ = v.template head<3>();
    vcom_moving_window_filter_.push_back(t, vcom_.template head<2>());
    const Eigen::Vector2d& vcom_avg = vcom_moving_window_filter_.average();
    raibert_heuristic_.planStepLength(vcom_avg, vcom_cmd_.template head<2>(), 
                                      yaw_rate_cmd_);
    step_length_ = raibert_heuristic_.stepLength();
  }
  robot_.updateFrameKinematics(q);
  aligned_vector<SE3> contact_placement;
  for (const auto frame : robot_.surfaceContactFrames()) {
    contact_placement.push_back(robot_.framePlacement(frame));
  }
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R = R_.front();
  std::vector<Eigen::Vector3d> contact_position_local;
  for (const auto& e : contact_placement) {
    contact_position_local.push_back(
        R.transpose() * (e.translation()-q.template head<3>()));
  }
  if (contact_status.isContactActive(0) && contact_status.isContactActive(1)) {
    current_step_ = 0;
    com = 0.5 * (contact_placement[0].translation() + contact_placement[1].translation());
    com.coeffRef(2) += foot_height_to_com_height_;
  }
  else if (contact_status.isContactActive(0)) {
    if (current_step_%4 != 1) {
      ++current_step_;
      R = (R_yaw_ * R).eval();
    }
    contact_position_local[1].noalias() 
        = contact_position_local[0] - 0.5 * R_yaw_.transpose() * step_length_;
    contact_position_local[1].coeffRef(1) -= left_to_right_leg_distance_;
    contact_placement[1].translation().noalias()  
        = R * contact_position_local[1] + q.template head<3>();
    contact_placement[1].rotation() = R;
    com = 0.5 * (contact_placement[0].translation() + contact_placement[1].translation());
    com.coeffRef(2) += foot_height_to_com_height_;
  }
  else if (contact_status.isContactActive(1)) {
    if (current_step_%4 != 3) {
      ++current_step_;
      R = (R_yaw_ * R).eval();
    }
    contact_position_local[0].noalias() 
        = contact_position_local[1] - 0.5 * R_yaw_.transpose() * step_length_;
    contact_position_local[0].coeffRef(1) += left_to_right_leg_distance_;
    contact_placement[0].translation().noalias()  
        = R * contact_position_local[0] + q.template head<3>();
    contact_placement[0].rotation() = R;
    com = 0.5 * (contact_placement[0].translation() + contact_placement[1].translation());
    com.coeffRef(2) += foot_height_to_com_height_;
  }
  else {
    if (current_step_%2 != 0) {
      ++current_step_;
    }
    for (int i=0; i<2; ++i) {
      contact_position_local[i] = contact_position_ref_[0][i];
    }
    contact_placement[0].translation().noalias()  
        = R * contact_position_local[0] + q.template head<3>();
    contact_placement[0].rotation() = R;
    contact_placement[1].translation().noalias()  
        = R * contact_position_local[1] + q.template head<3>();
    contact_placement[1].rotation() = R;
    com = 0.5 * (contact_placement[0].translation() + contact_placement[1].translation());
    com.coeffRef(2) += foot_height_to_com_height_;
  } 
  com_ref_.clear();
  com_ref_.push_back(com);
  contact_placement_ref_.clear();
  contact_placement_ref_.push_back(contact_placement);
  R_.clear();
  R_.push_back(R);
  for (int step=current_step_; step<=planning_steps+current_step_; ++step) {
    if (step == 0) {
      // do nothing
    }
    else if (current_step_ == 0 && step == 1) {
      // do nothing
    }
    else if (current_step_ == 0 && step == 2) {
      R = (R_yaw_ * R).eval();
      if (enable_raibert_heuristic_) {
        com.noalias() += 0.5 * R * step_length_;
      }
      else {
        com.noalias() += 0.25 * R * step_length_;
      }
      contact_placement[1].translation().noalias() += 0.25 * R * step_length_;
    }
    else if (step%4 == 1) {
      // do nothing
    }
    else if (step%4 == 2) {
      R = (R_yaw_ * R).eval();
      com.noalias() += 0.5 * R * step_length_;
      contact_placement[1].translation().noalias() += 0.5 * R * step_length_;
      contact_placement[1].rotation() = R;
    }
    else if (step%4 == 3) {
      // do nothing
    }
    else {
      R = (R_yaw_ * R).eval();
      com.noalias() += 0.5 * R * step_length_;
      contact_placement[0].translation().noalias() += R * step_length_;
      contact_placement[0].rotation() = R;
    }
    contact_placement_ref_.push_back(contact_placement);
    com_ref_.push_back(com);
    R_.push_back(R);
  }
  contact_position_ref_.clear();
  contact_surface_ref_.clear();
  for (const auto& e : contact_placement_ref_) {
    contact_position_ref_.push_back(
        std::vector<Eigen::Vector3d>({e[0].translation(), e[1].translation()}));
    contact_surface_ref_.push_back(
        std::vector<Eigen::Matrix3d>({e[0].rotation(), e[1].rotation()}));
  }
  planning_size_ = com_ref_.size();
  return true;
}


const aligned_vector<SE3>& BipedRunningFootStepPlanner::contactPlacements(const int step) const {
  return contact_placement_ref_[step];
}


const std::vector<Eigen::Vector3d>& BipedRunningFootStepPlanner::contactPositions(const int step) const {
  return contact_position_ref_[step];
}


const std::vector<Eigen::Matrix3d>& BipedRunningFootStepPlanner::contactSurfaces(const int step) const {
  return contact_surface_ref_[step];
}


const Eigen::Vector3d& BipedRunningFootStepPlanner::CoM(const int step) const {
  return com_ref_[step];
}


const Eigen::Matrix3d& BipedRunningFootStepPlanner::R(const int step) const {
  return R_[step];
}
  

std::ostream& operator<<(std::ostream& os, 
                         const BipedRunningFootStepPlanner& planner) {
  planner.disp(os);
  return os;
}


std::ostream& operator<<(std::ostream& os, 
                         const std::shared_ptr<BipedRunningFootStepPlanner>& planner) {
  planner->disp(os);
  return os;
}

} // namespace robotoc 