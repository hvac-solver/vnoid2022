#include "mpc_jump_com_ref.hpp"

#include <stdexcept>
#include <iostream>
#include <cassert>

namespace robotoc {

MPCJumpCoMRef::MPCJumpCoMRef(const double jump_start_time, 
                             const double flying_time) 
  : com_init_(Eigen::Vector3d::Zero()), 
    jump_length_(Eigen::Vector3d::Zero()),
    jump_velocity_(Eigen::Vector3d::Zero()),
    jump_start_time_(jump_start_time), 
    flying_time_(flying_time), 
    jump_height_(0) {
  if (flying_time <= 0.0) {
    throw std::invalid_argument(
        "[MPCJumpCoMRef] invalid argument: 'flying_time' must be positive!");
  }
  jump_velocity_ = jump_length_ / flying_time_;
}


MPCJumpCoMRef::~MPCJumpCoMRef() {
}


void MPCJumpCoMRef::setTimings(const double jump_start_time, 
                               const double flying_time) {
  if (flying_time <= 0.0) {
    throw std::invalid_argument(
        "[MPCJumpCoMRef] invalid argument: 'flying_time' must be positive!");
  }
  jump_start_time_ = jump_start_time;
  flying_time_ = flying_time;
  jump_velocity_ = jump_length_ / flying_time_;
}


void MPCJumpCoMRef::setCoMRef(const Eigen::Vector3d& com_init,
                              const Eigen::Vector3d& jump_length,
                              const double jump_height) {
  if (jump_height < 0.0) {
    throw std::invalid_argument(
        "[MPCJumpCoMRef] invalid argument: 'jump_height' must be non-negative!");
  }
  com_init_ = com_init;
  jump_length_ = jump_length;
  jump_velocity_ = jump_length_ / flying_time_;
  jump_height_ = jump_height;
  jump_height_trajectory_ = JumpHeightTrajectory(jump_start_time_, com_init_.coeff(2),
                                                 jump_start_time_+flying_time_, 
                                                 com_init_.coeff(2)+jump_length_.coeff(2),
                                                 com_init_.coeff(2)+jump_height_);
}


void MPCJumpCoMRef::updateRef(const GridInfo& grid_info, 
                              Eigen::VectorXd& com_ref) const {
  assert(com_ref.size() == 3);
  if (grid_info.t <= jump_start_time_) {
    com_ref.template head<3>() = com_init_;
  }
  else if (grid_info.t >= jump_start_time_ + flying_time_) {
    com_ref.template head<3>() = com_init_ + jump_length_;
  }
  else {
    com_ref.template head<3>() = com_init_ + (grid_info.t - jump_start_time_) * jump_velocity_;
    com_ref.coeffRef(2) = jump_height_trajectory_.h(grid_info.t);
  }
}


bool MPCJumpCoMRef::isActive(const GridInfo& grid_info) const {
  return true;
}

} // namespace robotoc