#ifndef ROBOTOC_MPC_JUMP_COM_REF_HPP_
#define ROBOTOC_MPC_JUMP_COM_REF_HPP_

#include <vector>
#include <memory>

#include "Eigen/Core"

#include "robotoc/robot/robot.hpp"
#include "robotoc/cost/com_ref_base.hpp"


namespace robotoc {

class JumpHeightTrajectory {
public:
  JumpHeightTrajectory(const double t0, const double h0, 
                       const double tf, const double hf, const double hM)
    : hM_(hM),
      tM_(0),
      a_(0) {
    tM_ = (h0*tf + hM*t0 - hM*tf - hf*t0 + std::sqrt(std::abs((h0 - hM)*(hM - hf)))*(-t0 + tf)) / (h0 - hf);
    a_ = (hM-h0) / ((t0-tM_)*(t0-tM_));
  }

  JumpHeightTrajectory() = default;

  double h(const double t) const {
    return - a_ * (t-tM_) * (t-tM_) + hM_;
  }

private:
  double hM_, tM_, a_;
};

///
/// @class MPCJumpCoMRef
/// @brief Reference position of the center of mass for jump. 
///
class MPCJumpCoMRef final : public CoMRefBase {
public:
  ///
  /// @brief Constructor. 
  /// @param[in] jump_start_tiem Start time of the jump.
  /// @param[in] flying_time Flying time.
  ///
  MPCJumpCoMRef(const double jump_start_time, const double flying_time);

  ///
  /// @brief Destructor. 
  ///
  ~MPCJumpCoMRef();

  ///
  /// @brief Sets jump timings. 
  /// @param[in] jump_start_tiem Start time of the jump.
  /// @param[in] flying_time Flying time.
  ///
  void setTimings(const double jump_start_time, const double flying_time);

  ///
  /// @brief Set the reference positions of CoM.
  /// @param[in] com_init Initial CoM position.
  /// @param[in] jump_length Length of the jump.
  /// @param[in] jump_height Height of the jump.
  ///
  void setCoMRef(const Eigen::Vector3d& com_init,
                 const Eigen::Vector3d& jump_length,
                 const double jump_height);

  void updateRef(const GridInfo& grid_info, 
                 Eigen::VectorXd& com_ref) const override;

  bool isActive(const GridInfo& grid_info) const override;

private:
  Eigen::Vector3d com_init_, jump_length_, jump_velocity_;
  double jump_start_time_, flying_time_, jump_height_;
  JumpHeightTrajectory jump_height_trajectory_;
};

} // namespace robotoc

#endif // ROBOTOC_MPC_JUMP_COM_REF_HPP_