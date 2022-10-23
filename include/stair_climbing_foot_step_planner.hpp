#ifndef ROBOTOC_STAIR_CLIMBING_FOOT_STEP_PLANNER_HPP_
#define ROBOTOC_STAIR_CLIMBING_FOOT_STEP_PLANNER_HPP_

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
/// @class StairClimbingFootStepPlanner
/// @brief Foot step planner for the biped robot walk. 
///
class StairClimbingFootStepPlanner final : public ContactPlannerBase {
public:
  ///
  /// @brief Constructs the planner.
  /// @param[in] biped_robot Biped robot model. 
  ///
  StairClimbingFootStepPlanner(const Robot& biped_robot);

  ///
  /// @brief Default constructor. 
  ///
  StairClimbingFootStepPlanner();

  ///
  /// @brief Destructor. 
  ///
  ~StairClimbingFootStepPlanner();

  ///
  /// @brief Default copy constructor. 
  ///
  StairClimbingFootStepPlanner(const StairClimbingFootStepPlanner&) = default;

  ///
  /// @brief Default copy assign operator. 
  ///
  StairClimbingFootStepPlanner& operator=(const StairClimbingFootStepPlanner&) = default;

  ///
  /// @brief Default move constructor. 
  ///
  StairClimbingFootStepPlanner(StairClimbingFootStepPlanner&&) noexcept = default;

  ///
  /// @brief Default move assign operator. 
  ///
  StairClimbingFootStepPlanner& operator=(StairClimbingFootStepPlanner&&) noexcept = default;

  ///
  /// @brief Sets the gait pattern by step length and yaw step command. 
  /// @param[in] step_length Step length of the gait. 
  /// @param[in] num_stair_steps Number of the stair steps. 
  /// @param[in] enable_double_support_phase Enables the double-support 
  /// phase or not. Default is false.
  ///
  void setGaitPattern(const Eigen::Vector3d& step_length, 
                      const int num_stair_steps,
                      const bool enable_double_support_phase=false);

  void init(const Eigen::VectorXd& q) override;

  bool plan(const double t, const Eigen::VectorXd& q, const Eigen::VectorXd& v, 
            const ContactStatus& contact_status, 
            const int planning_steps) override;

  const aligned_vector<SE3>& contactPlacements(const int step) const override;

  ///
  /// @brief This is invalid in StairClimbingFootStepPlanner. 
  ///
  const std::vector<Eigen::Vector3d>& contactPositions(const int step) const override;

  ///
  /// @brief This is invalid in StairClimbingFootStepPlanner. 
  ///
  const std::vector<Eigen::Matrix3d>& contactSurfaces(const int step) const override;

  const Eigen::Vector3d& CoM(const int step) const override;

  const Eigen::Matrix3d& R(const int step) const override;

  int size() const override { return planning_size_; }

  friend std::ostream& operator<<(std::ostream& os, 
                                  const StairClimbingFootStepPlanner& planner);

  friend std::ostream& operator<<(std::ostream& os, 
                                  const std::shared_ptr<StairClimbingFootStepPlanner>& planner);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Robot robot_;
  int L_foot_id_, R_foot_id_, current_step_, planning_size_;
  double left_to_right_leg_distance_, foot_height_to_com_height_;
  aligned_deque<aligned_vector<SE3>> contact_placement_ref_;
  std::vector<std::vector<Eigen::Vector3d>> contact_position_ref_;
  std::vector<std::vector<Eigen::Matrix3d>> contact_surface_ref_;
  std::vector<Eigen::Vector3d> com_ref_;
  std::vector<Eigen::Matrix3d> R_;
  Eigen::Vector3d vcom_, vcom_cmd_, step_length_;
  Eigen::Matrix3d R_current_;
  double yaw_rate_cmd_;
  bool enable_double_support_phase_, L_contact_active_, R_contact_active_;

};

} // namespace robotoc 

#endif // ROBOTOC_STAIR_CLIMBING_FOOT_STEP_PLANNER_HPP_