#ifndef ROBOTOC_MPC_BIPED_JUMP_HPP_
#define ROBOTOC_MPC_BIPED_JUMP_HPP_

#include <vector>
#include <memory>
#include <limits>

#include "Eigen/Core"

#include "robotoc/robot/robot.hpp"
#include "robotoc/ocp/ocp.hpp"
#include "robotoc/solver/ocp_solver.hpp"
#include "robotoc/planner/contact_sequence.hpp"
#include "robotoc/cost/cost_function.hpp"
#include "robotoc/constraints/constraints.hpp"
#include "robotoc/solver/solver_options.hpp"
#include "robotoc/utils/aligned_vector.hpp"
#include "robotoc/robot/se3.hpp"
#include "robotoc/mpc/contact_planner_base.hpp"
#include "robotoc/cost/configuration_space_cost.hpp"
#include "robotoc/cost/com_cost.hpp"
#include "robotoc/cost/task_space_3d_cost.hpp"
#include "robotoc/cost/task_space_6d_cost.hpp"
#include "robotoc/cost/periodic_swing_foot_ref.hpp"
#include "robotoc/constraints/joint_position_lower_limit.hpp"
#include "robotoc/constraints/joint_position_upper_limit.hpp"
#include "robotoc/constraints/joint_velocity_lower_limit.hpp"
#include "robotoc/constraints/joint_velocity_upper_limit.hpp"
#include "robotoc/constraints/joint_torques_lower_limit.hpp"
#include "robotoc/constraints/joint_torques_upper_limit.hpp"
#include "robotoc/constraints/contact_wrench_cone.hpp"
#include "robotoc/mpc/control_policy.hpp"

#include "mpc_jump_com_ref.hpp"


namespace robotoc {

///
/// @class MPCBipedJump
/// @brief MPC solver for the jump control. 
///
class MPCBipedJump {
public:
  ///
  /// @brief Construct MPC solver.
  /// @param[in] robot Robot model. 
  /// @param[in] T Length of the horizon. 
  /// @param[in] N Number of the discretization grids of the horizon. 
  ///
  MPCBipedJump(const Robot& robot, const double T, const int N);

  ///
  /// @brief Default constructor. 
  ///
  MPCBipedJump();

  ///
  /// @brief Destructor. 
  ///
  ~MPCBipedJump();

  ///
  /// @brief Default copy constructor. 
  ///
  MPCBipedJump(const MPCBipedJump&) = default;

  ///
  /// @brief Default copy assign operator. 
  ///
  MPCBipedJump& operator=(const MPCBipedJump&) = default;

  ///
  /// @brief Default move constructor. 
  ///
  MPCBipedJump(MPCBipedJump&&) noexcept = default;

  ///
  /// @brief Default move assign operator. 
  ///
  MPCBipedJump& operator=(MPCBipedJump&&) noexcept = default;

  ///
  /// @brief Sets the gait pattern. 
  /// @param[in] foot_step_planner Foot step planner of the jump. 
  /// @param[in] jump_height Jump height.
  /// @param[in] ground_time The ground time before jump.
  /// @param[in] flying_up_time The flying up time.
  /// @param[in] flying_time The flying time.
  /// @param[in] landing_time The landing time.
  ///
  void setJumpPattern(const std::shared_ptr<ContactPlannerBase>& foot_step_planner,
                      const double jump_height,
                      const double ground_time, const double flying_up_time, 
                      const double flying_time, const double landing_time);

  ///
  /// @brief Initializes the optimal control problem solover. 
  /// @param[in] t Initial time of the horizon. 
  /// @param[in] q Initial configuration. Size must be Robot::dimq().
  /// @param[in] v Initial velocity. Size must be Robot::dimv().
  /// @param[in] solver_options Solver options for the initialization. 
  ///
  void init(const double t, const Eigen::VectorXd& q, const Eigen::VectorXd& v, 
            const SolverOptions& solver_options);

  ///
  /// @brief Resets the optimal control problem solover via the solution 
  /// computed by init() or reset(). 
  ///
  void reset();

  ///
  /// @brief Resets the optimal control problem solover via the solution 
  /// computed by init(), q, and v.
  ///
  void reset(const Eigen::VectorXd& q, const Eigen::VectorXd& v);

  ///
  /// @brief Resets the optimal control problem solver using the previous 
  /// results of init() or reset().
  /// @param[in] t Initial time of the horizon. 
  /// @param[in] q Initial configuration. Size must be Robot::dimq().
  /// @param[in] v Initial velocity. Size must be Robot::dimv().
  /// @param[in] solver_options Solver options for the initialization. 
  /// @remark The linear and angular velocities of the floating base are assumed
  /// to be expressed in the body local coordinate.
  ///
  void reset(const double t, const Eigen::VectorXd& q, const Eigen::VectorXd& v, 
             const SolverOptions& solver_options);

  ///
  /// @brief Sets the solver options. 
  /// @param[in] solver_options Solver options.  
  ///
  void setSolverOptions(const SolverOptions& solver_options);

  ///
  /// @brief Updates the solution by iterationg the Newton-type method.
  /// @param[in] t Initial time of the horizon. 
  /// @param[in] dt Sampling time of MPC. Must be positive.
  /// @param[in] q Configuration. Size must be Robot::dimq().
  /// @param[in] v Velocity. Size must be Robot::dimv().
  /// @remark The linear and angular velocities of the floating base are assumed
  /// to be expressed in the body local coordinate.
  ///
  void updateSolution(const double t, const double dt, const Eigen::VectorXd& q, 
                      const Eigen::VectorXd& v);

  ///
  /// @brief Get the initial control input.
  /// @return Const reference to the control input.
  ///
  const Eigen::VectorXd& getInitialControlInput() const;

  ///
  /// @brief Get the solution over the horizon. 
  /// @return const reference to the solution.
  ///
  const Solution& getSolution() const;

  ///
  /// @brief Gets of the local LQR policies over the horizon. 
  /// @return const reference to the local LQR policies.
  ///
  const aligned_vector<LQRPolicy>& getLQRPolicy() const;

  ///
  /// @brief Gets the control policy at the specified time.  
  /// @param[in] t The specified time.  
  /// @return Control poclity at the specified time.
  ///
  ControlPolicy getControlPolicy(const double t) const { 
    return ControlPolicy(ocp_solver_, t); 
  }

  ///
  /// @brief Computes the KKT residual of the optimal control problem. 
  /// @param[in] t Initial time of the horizon. 
  /// @param[in] q Initial configuration. Size must be Robot::dimq().
  /// @param[in] v Initial velocity. Size must be Robot::dimv().
  /// @remark The linear and angular velocities of the floating base are assumed
  /// to be expressed in the body local coordinate.
  ///
  double KKTError(const double t, const Eigen::VectorXd& q, 
                  const Eigen::VectorXd& v);

  ///
  /// @brief Returns the l2-norm of the KKT residuals.
  /// MPCBipedJump::updateSolution() must be computed.  
  /// @return The l2-norm of the KKT residual.
  ///
  double KKTError() const;

  ///
  /// @brief Gets the cost function handle.  
  /// @return Shared ptr to the cost function.
  ///
  std::shared_ptr<CostFunction> getCostHandle();

  ///
  /// @brief Gets the configuration space cost handle.  
  /// @return Shared ptr to the configuration space cost.
  ///
  std::shared_ptr<ConfigurationSpaceCost> getConfigCostHandle();

  ///
  /// @brief Gets the constraints handle.  
  /// @return Shared ptr to the constraints.
  ///
  std::shared_ptr<Constraints> getConstraintsHandle();

  ///
  /// @brief Gets the contact wrench cone constraints handle.  
  /// @return Shared ptr to the wrench cone constraints.
  ///
  std::shared_ptr<ContactWrenchCone> getContactWrenchConeHandle();

  ///
  /// @brief Gets the const handle of the MPC solver.  
  /// @return Const reference to the MPC solver.
  ///
  const OCPSolver& getSolver() const { return ocp_solver_; }

  ///
  /// @brief Gets the const handle of the contact sequence.  
  /// @return Const reference to the shared_ptr of the contact sequence.
  ///
  const std::shared_ptr<ContactSequence>& getContactSequence() const { 
    return contact_sequence_; 
  }

  ///
  ///
  /// @brief Sets a collection of the properties for robot model in this MPC. 
  /// @param[in] properties A collection of the properties for the robot model.
  ///
  void setRobotProperties(const RobotProperties& properties);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  std::shared_ptr<ContactPlannerBase> foot_step_planner_;
  std::shared_ptr<ContactSequence> contact_sequence_;
  std::shared_ptr<CostFunction> cost_;
  std::shared_ptr<Constraints> constraints_;
  OCPSolver ocp_solver_;
  SolverOptions solver_options_;
  ContactStatus cs_ground_, cs_right_flying_, cs_left_flying_, cs_flying_;
  Solution s_;
  double jump_height_, ground_time_, flying_up_time_, flying_time_, landing_time_,
         T_, dt_, dtm_, eps_;
  int N_, current_step_;

  std::shared_ptr<ConfigurationSpaceCost> config_cost_;
  std::shared_ptr<CoMCost> com_cost_;
  std::shared_ptr<TaskSpace3DCost> L_foot_cost_, R_foot_cost_;
  std::shared_ptr<TaskSpace6DCost> L_foot_rot_cost_, R_foot_rot_cost_;
  std::shared_ptr<ContactWrenchCone> contact_wrench_cone_;

  void resetGoalContactPlacements(const Eigen::VectorXd& q);

  void resetContactPlacements(const double t, const Eigen::VectorXd& q, 
                              const Eigen::VectorXd& v);
};

} // namespace robotoc 

#endif // ROBOTOC_MPC_BIPED_JUMP_HPP_