#pragma once

#include "kinex/export.h"
#include "kinex/kinematics.h"
#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace kinex {

class DaQPSolver;

struct SolverConfig {
  size_t max_iterations = 64;
  double tolerance = 1e-4;
  double regularization = 1e-5;
  double max_step_size = 0.5;
  size_t max_line_search_steps = 6;
  double line_search_shrink = 0.5;
  double line_search_min_alpha = 0.05;
  double line_search_improvement = 1e-6;
  double position_weight = 1.0;
  double orientation_weight = 1.0;
  double position_anchor_weight = 1e-2;
  double orientation_anchor_weight = 1e-6;
  double joint_limit_margin = 1e-4;
  double unbounded_joint_limit = 50.0;
  bool enable_warm_start = true;

  // Adaptive damping (DLS) parameters
  bool enable_adaptive_damping = true;
  double damping_threshold =
      0.1; // Manipulability threshold below which damping increases
  double max_damping = 0.1; // Maximum damping factor added at singularity

  // Stagnation escape parameters
  bool enable_stagnation_escape = true;
  double stagnation_perturbation = 0.1; // Magnitude of random perturbation
  size_t stagnation_detect_iters = 5;   // Iterations to detect stagnation
  double stagnation_threshold = 1e-6;   // Error change threshold

  // Manipulability maximization (singularity avoidance)
  bool enable_manipulability_gradient = false; // Disabled by default for backward compatibility
  double manipulability_weight = 0.001;      // Weight for manipulability gradient
  double manipulability_min_threshold = 0.01; // Below this, increase weight
  double manipulability_scale_factor = 5.0;  // Scale factor for weight increase

  // SVD-based damped least squares
  bool enable_svd_damping = false;           // Use SVD-based damping instead of simple diagonal
  double svd_damping_lambda_max = 0.5;       // Maximum damping for near-singular directions
  double svd_singular_threshold = 0.01;      // Singular value threshold for damping

  // Slack variables for feasibility relaxation
  bool enable_slack_variables = false;       // Disabled by default for backward compatibility
  double slack_penalty = 100.0;              // Penalty weight for slack variables
  double slack_max = 0.1;                    // Maximum slack allowed
};

struct SolverStatus {
  bool converged = false;
  size_t iterations = 0;
  double final_error_norm = 0.0;
  double final_step_norm = 0.0;
  int qp_status = 0;
  std::string message;
  std::vector<double> error_history;
};

class KINEX_API IKSolver {
public:
  IKSolver(std::shared_ptr<const RobotModel> robot, std::string end_link,
           std::string base_link = "");
  virtual ~IKSolver() = default;

  virtual SolverStatus solve(const Transform &target_pose,
                             const Eigen::VectorXd &initial_guess,
                             Eigen::VectorXd &solution) = 0;

  void setSolverConfig(const SolverConfig &config);
  const SolverConfig &getSolverConfig() const { return config_; }

  std::shared_ptr<const RobotModel> getRobot() const { return robot_; }
  const std::string &getEndLink() const { return end_link_; }
  const std::string &getBaseLink() const { return base_link_; }

  void setPositionOnly(bool enable);
  void setOrientationOnly(bool enable);

  void setWarmStart(const Eigen::VectorXd &guess);
  std::optional<Eigen::VectorXd> getWarmStart() const { return warm_start_; }

protected:
  std::shared_ptr<const RobotModel> robot_;
  std::string end_link_;
  std::string base_link_;
  SolverConfig config_;
  bool position_only_ = false;
  bool orientation_only_ = false;
  std::optional<Eigen::VectorXd> warm_start_;
};

class KINEX_API SQPIKSolver final : public IKSolver {
public:
  SQPIKSolver(std::shared_ptr<const RobotModel> robot,
              const std::string &end_link, const std::string &base_link = "");

  ~SQPIKSolver() override;

  SolverStatus solve(const Transform &target_pose,
                     const Eigen::VectorXd &initial_guess,
                     Eigen::VectorXd &solution) override;

private:
  ForwardKinematics fk_;
  JacobianCalculator jacobian_;
  std::unique_ptr<class DaQPSolver> qp_solver_;

  // Pre-allocated working arrays for zero-allocation solve loop
  mutable RobotState robot_state_;
  mutable Eigen::VectorXd weighted_error_;
  mutable Eigen::VectorXd delta_;
  mutable Eigen::VectorXd q_current_;
  mutable Eigen::VectorXd lower_bounds_;
  mutable Eigen::VectorXd upper_bounds_;
  mutable Eigen::VectorXd lower_delta_;
  mutable Eigen::VectorXd upper_delta_;
  mutable Eigen::MatrixXd H_;
  mutable Eigen::VectorXd g_;
  mutable Eigen::MatrixXd J_;
  mutable Eigen::MatrixXd weighted_jac_;
  mutable Eigen::VectorXd manip_gradient_;  // Manipulability gradient

  Eigen::VectorXd buildTaskError(const Transform &current_pose,
                                 const Transform &target_pose) const;

  Eigen::VectorXd weightError(const Eigen::VectorXd &full_error) const;
  Eigen::MatrixXd weightJacobian(const Eigen::MatrixXd &full_jacobian) const;

  void computeJointBounds(Eigen::VectorXd &lower, Eigen::VectorXd &upper) const;

  void clampToJointLimits(Eigen::VectorXd &joints) const;

  // Compute manipulability measure and its gradient
  double computeManipulability(const Eigen::MatrixXd &J) const;
  Eigen::VectorXd computeManipulabilityGradient(const Eigen::VectorXd &q,
                                                 double h = 1e-6) const;

  // Apply SVD-based damped least squares
  void applySVDDamping(Eigen::MatrixXd &H, const Eigen::MatrixXd &J,
                       double lambda_max, double threshold) const;
};

} // namespace kinex
