#ifndef ROBOTOC_BIPED_JUMP_FOOT_STEP_PLANNER_HPP_
#define ROBOTOC_BIPED_JUMP_FOOT_STEP_PLANNER_HPP_

#include <vector>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "robotoc/robot/robot.hpp"
#include "robotoc/robot/contact_status.hpp"
#include "robotoc/robot/se3.hpp"
#include "robotoc/utils/aligned_vector.hpp"
#include "robotoc/utils/aligned_deque.hpp"
#include "robotoc/mpc/contact_planner_base.hpp"


namespace robotoc {

///
/// @class BipedJumpFootStepPlanner
/// @brief Foot step planner for the biped robot jump. 
///
class BipedJumpFootStepPlanner final : public ContactPlannerBase {
public:
  ///
  /// @brief Constructs the planner.
  /// @param[in] biped_robot Biped robot model. 
  ///
  BipedJumpFootStepPlanner(const Robot& biped_robot);

  ///
  /// @brief Default constructor. 
  ///
  BipedJumpFootStepPlanner();

  ///
  /// @brief Destructor. 
  ///
  ~BipedJumpFootStepPlanner();

  ///
  /// @brief Default copy constructor. 
  ///
  BipedJumpFootStepPlanner(const BipedJumpFootStepPlanner&) = default;

  ///
  /// @brief Default copy assign operator. 
  ///
  BipedJumpFootStepPlanner& operator=(const BipedJumpFootStepPlanner&) = default;

  ///
  /// @brief Default move constructor. 
  ///
  BipedJumpFootStepPlanner(BipedJumpFootStepPlanner&&) noexcept = default;

  ///
  /// @brief Default move assign operator. 
  ///
  BipedJumpFootStepPlanner& operator=(BipedJumpFootStepPlanner&&) noexcept = default;

  ///
  /// @brief Sets the gait pattern by step length and yaw step command. 
  /// @param[in] jump_length Length of the jump. 
  /// @param[in] jump_yaw Change in the yaw angle of the jump. 
  ///
  void setJumpPattern(const Eigen::Vector3d& jump_length, const double jump_yaw);

  void init(const Eigen::VectorXd& q) override;

  bool plan(const double t, const Eigen::VectorXd& q, const Eigen::VectorXd& v, 
            const ContactStatus& contact_status, 
            const int planning_steps) override;

  const aligned_vector<SE3>& contactPlacements(const int step) const override;

  ///
  /// @brief This is invalid in BipedJumpFootStepPlanner. 
  ///
  const std::vector<Eigen::Vector3d>& contactPositions(const int step) const override;

  ///
  /// @brief This is invalid in BipedJumpFootStepPlanner. 
  ///
  const std::vector<Eigen::Matrix3d>& contactSurfaces(const int step) const override;

  const Eigen::Vector3d& CoM(const int step) const override;

  const Eigen::Matrix3d& R(const int step) const override;

  int size() const override { return planning_size_; }

  friend std::ostream& operator<<(std::ostream& os, 
                                  const BipedJumpFootStepPlanner& planner);

  friend std::ostream& operator<<(std::ostream& os, 
                                  const std::shared_ptr<BipedJumpFootStepPlanner>& planner);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Robot robot_;
  std::vector<int> contact_frames_;
  int current_step_, planning_size_, L_foot_id_, R_foot_id_;
  aligned_deque<aligned_vector<SE3>> contact_placement_ref_;
  std::deque<std::vector<Eigen::Vector3d>> contact_position_ref_;
  std::deque<std::vector<Eigen::Matrix3d>> contact_surface_ref_;
  std::deque<Eigen::Vector3d> com_ref_, com_to_contact_position_local_;
  std::deque<Eigen::Matrix3d> R_;
  Eigen::Vector3d jump_length_;
  Eigen::Matrix3d R_yaw_;
  bool L_contact_active_, R_contact_active_;

};

} // namespace robotoc 

#endif // ROBOTOC_BIPED_JUMP_FOOT_STEP_PLANNER_HPP_