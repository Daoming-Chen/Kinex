#include "kinex/inverse_kinematics.h"
#include "api.h"
#include "constants.h"
#include "kinex/logging.h"
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace kinex {

class DaQPSolver {
public:
  explicit DaQPSolver(size_t dof) : dof_(static_cast<int>(dof)) {
    result_.x = nullptr;
    result_.lam = nullptr;
  }

  bool solve(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
             const Eigen::VectorXd &lower, const Eigen::VectorXd &upper,
             Eigen::VectorXd &delta, int &exitflag) {
    if (dof_ == 0) {
      exitflag = EXIT_INFEASIBLE;
      return false;
    }

    if (lower.size() != dof_ || upper.size() != dof_) {
      throw std::invalid_argument("DaQPSolver bounds size mismatch");
    }

    if (delta.size() != dof_) {
      delta.resize(dof_);
    }

    lower_.assign(lower.data(), lower.data() + dof_);
    upper_.assign(upper.data(), upper.data() + dof_);
    x_.assign(dof_, 0.0);
    lam_.assign(dof_, 0.0);

    DAQPProblem problem{};
    problem.n = dof_;
    problem.m = dof_;
    problem.ms = dof_;
    problem.H = const_cast<double *>(H.data());
    problem.f = const_cast<double *>(g.data());
    problem.A = nullptr;
    problem.bupper = upper_.data();
    problem.blower = lower_.data();
    problem.sense = nullptr;
    problem.break_points = nullptr;
    problem.nh = 0;

    result_.x = x_.data();
    result_.lam = lam_.data();
    result_.fval = 0.0;
    result_.soft_slack = 0.0;
    result_.exitflag = EXIT_ITERLIMIT;
    result_.iter = 0;
    result_.nodes = 0;
    result_.solve_time = 0.0;
    result_.setup_time = 0.0;

    daqp_quadprog(&result_, &problem, nullptr);
    exitflag = result_.exitflag;

    if (exitflag >= EXIT_OPTIMAL) {
      Eigen::Map<const Eigen::VectorXd> mapped(result_.x, dof_);
      delta = mapped;
      return true;
    }

    return false;
  }

private:
  int dof_;
  DAQPResult result_{};
  std::vector<double> lower_;
  std::vector<double> upper_;
  std::vector<double> x_;
  std::vector<double> lam_;
};

namespace {

Eigen::Vector3d orientationError(const Eigen::Matrix3d &current,
                                 const Eigen::Matrix3d &target) {
  Eigen::Matrix3d R_err = current.transpose() * target;
  Eigen::AngleAxisd axis_angle(R_err);
  double angle = axis_angle.angle();
  if (std::abs(angle) < 1e-9) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d axis = axis_angle.axis();
  return current * (axis * angle);
}

} // namespace

IKSolver::IKSolver(std::shared_ptr<const RobotModel> robot,
                   std::string end_link, std::string base_link)
    : robot_(std::move(robot)), end_link_(std::move(end_link)),
      base_link_(std::move(base_link)) {
  if (!robot_) {
    throw std::invalid_argument("IKSolver requires valid robot pointer");
  }

  if (base_link_.empty()) {
    base_link_ = robot_->getRootLink();
  }
  if (end_link_.empty()) {
    end_link_ = robot_->getRootLink();
  }

  if (!robot_->getLink(base_link_)) {
    throw std::invalid_argument("Base link not present in robot model");
  }
  if (!robot_->getLink(end_link_)) {
    throw std::invalid_argument("End link not present in robot model");
  }
}

void IKSolver::setSolverConfig(const SolverConfig &config) { config_ = config; }

void IKSolver::setPositionOnly(bool enable) {
  if (enable && orientation_only_) {
    throw std::invalid_argument(
        "Cannot enable position-only and orientation-only simultaneously");
  }
  position_only_ = enable;
}

void IKSolver::setOrientationOnly(bool enable) {
  if (enable && position_only_) {
    throw std::invalid_argument(
        "Cannot enable position-only and orientation-only simultaneously");
  }
  orientation_only_ = enable;
}

void IKSolver::setWarmStart(const Eigen::VectorXd &guess) {
  warm_start_ = guess;
}

SQPIKSolver::SQPIKSolver(std::shared_ptr<const RobotModel> robot,
                         const std::string &end_link,
                         const std::string &base_link)
    : IKSolver(std::move(robot), end_link, base_link),
      fk_(robot_, end_link_, base_link_),
      jacobian_(robot_, end_link_, base_link_),
      qp_solver_(std::make_unique<DaQPSolver>(fk_.getNumJoints())) {
  // Pre-allocate all working arrays for zero-allocation solve loop
  const size_t dof = fk_.getNumJoints();
  const Eigen::Index dof_idx = static_cast<Eigen::Index>(dof);

  robot_state_.resize(fk_.getChain().getLinkNames().size(), dof);
  weighted_error_.resize(6);
  delta_.resize(dof_idx);
  q_current_.resize(dof_idx);
  lower_bounds_.resize(dof_idx);
  upper_bounds_.resize(dof_idx);
  lower_delta_.resize(dof_idx);
  upper_delta_.resize(dof_idx);
  H_.resize(dof_idx, dof_idx);
  g_.resize(dof_idx);
  J_.resize(6, dof_idx);
  weighted_jac_.resize(6, dof_idx);
  manip_gradient_.resize(dof_idx);
}

SQPIKSolver::~SQPIKSolver() = default;

Eigen::VectorXd
SQPIKSolver::buildTaskError(const Transform &current_pose,
                            const Transform &target_pose) const {
  Eigen::VectorXd error(6);
  error.head<3>() = target_pose.translation() - current_pose.translation();
  error.tail<3>() =
      orientationError(current_pose.rotation(), target_pose.rotation());
  return error;
}

Eigen::VectorXd
SQPIKSolver::weightError(const Eigen::VectorXd &full_error) const {
  Eigen::VectorXd weighted = full_error;
  const double pos_weight = orientation_only_ ? config_.position_anchor_weight
                                              : config_.position_weight;
  const double ori_weight = position_only_ ? config_.orientation_anchor_weight
                                           : config_.orientation_weight;
  weighted.head<3>() *= pos_weight;
  weighted.tail<3>() *= ori_weight;
  return weighted;
}

Eigen::MatrixXd
SQPIKSolver::weightJacobian(const Eigen::MatrixXd &full_jacobian) const {
  Eigen::MatrixXd weighted = full_jacobian;
  const double pos_weight = orientation_only_ ? config_.position_anchor_weight
                                              : config_.position_weight;
  const double ori_weight = position_only_ ? config_.orientation_anchor_weight
                                           : config_.orientation_weight;
  weighted.topRows(3) *= pos_weight;
  weighted.bottomRows(3) *= ori_weight;
  return weighted;
}

void SQPIKSolver::computeJointBounds(Eigen::VectorXd &lower,
                                     Eigen::VectorXd &upper) const {
  const auto &joints = fk_.getChain().getJoints();
  const size_t dof = joints.size();
  lower.resize(static_cast<Eigen::Index>(dof));
  upper.resize(static_cast<Eigen::Index>(dof));
  for (size_t i = 0; i < dof; ++i) {
    const auto &joint = joints[i];
    const auto &limits = joint->getLimits();
    if (limits) {
      lower[static_cast<Eigen::Index>(i)] =
          limits->lower - config_.joint_limit_margin;
      upper[static_cast<Eigen::Index>(i)] =
          limits->upper + config_.joint_limit_margin;
    } else {
      lower[static_cast<Eigen::Index>(i)] = -config_.unbounded_joint_limit;
      upper[static_cast<Eigen::Index>(i)] = config_.unbounded_joint_limit;
    }
  }
}

void SQPIKSolver::clampToJointLimits(Eigen::VectorXd &joints) const {
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  computeJointBounds(lower, upper);
  joints = joints.cwiseMax(lower).cwiseMin(upper);
}

double SQPIKSolver::computeManipulability(const Eigen::MatrixXd &J) const {
  // Yoshikawa manipulability measure: w = sqrt(det(J*J^T))
  // For non-square Jacobians, this measures how far from singularity we are
  Eigen::MatrixXd JJT = J * J.transpose();
  double det = JJT.determinant();
  return std::sqrt(std::max(det, 0.0));
}

Eigen::VectorXd
SQPIKSolver::computeManipulabilityGradient(const Eigen::VectorXd &q,
                                           double h) const {
  // Numerical gradient of manipulability w.r.t. joint angles
  // This gives us the direction to move to increase manipulability
  const size_t dof = fk_.getNumJoints();
  Eigen::VectorXd gradient(static_cast<Eigen::Index>(dof));

  // Compute gradient using finite differences (central difference)
  Eigen::VectorXd q_perturbed = q;
  for (size_t i = 0; i < dof; ++i) {
    Eigen::Index idx = static_cast<Eigen::Index>(i);
    q_perturbed[idx] = q[idx] + h;
    Eigen::MatrixXd J_plus = jacobian_.compute(q_perturbed);
    double w_plus = computeManipulability(J_plus);

    q_perturbed[idx] = q[idx] - h;
    Eigen::MatrixXd J_minus = jacobian_.compute(q_perturbed);
    double w_minus = computeManipulability(J_minus);

    gradient[idx] = (w_plus - w_minus) / (2.0 * h);
    q_perturbed[idx] = q[idx]; // Reset
  }

  return gradient;
}

void SQPIKSolver::applySVDDamping(Eigen::MatrixXd &H, const Eigen::MatrixXd &J,
                                  double lambda_max, double threshold) const {
  // Compute SVD of Jacobian to identify near-singular directions
  // Then apply stronger damping along those directions
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU |
                                               Eigen::ComputeThinV);
  const Eigen::VectorXd &sigma = svd.singularValues();
  const Eigen::MatrixXd &V = svd.matrixV();

  // Get the maximum singular value for normalization
  double sigma_max = sigma(0);
  if (sigma_max < 1e-10) {
    // Jacobian is essentially zero, add large damping everywhere
    H.diagonal().array() += lambda_max;
    return;
  }

  // Build damping matrix: add more damping for small singular values
  // Using Nakamura's method: lambda_i = lambda_max * (1 - (sigma_i/sigma_max)^2)
  // for sigma_i < threshold * sigma_max
  const size_t dof = static_cast<size_t>(H.rows());
  Eigen::MatrixXd damping_contribution =
      Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(dof),
                            static_cast<Eigen::Index>(dof));

  for (Eigen::Index i = 0; i < sigma.size(); ++i) {
    double sigma_ratio = sigma(i) / sigma_max;
    if (sigma_ratio < threshold) {
      // This direction is near-singular, add damping
      double factor = 1.0 - (sigma_ratio / threshold);
      double lambda_i = lambda_max * factor * factor;

      // Add damping along this direction in joint space
      // The direction in joint space is V.col(i)
      Eigen::VectorXd v_i = V.col(i);
      damping_contribution += lambda_i * (v_i * v_i.transpose());
    }
  }

  H += damping_contribution;
}

SolverStatus SQPIKSolver::solve(const Transform &target_pose,
                                const Eigen::VectorXd &initial_guess,
                                Eigen::VectorXd &solution) {
  SolverStatus status;
  const size_t dof = fk_.getNumJoints();
  if (dof == 0) {
    status.message = "No actuated joints in chain";
    return status;
  }

  // Use pre-allocated q_current_ instead of local variable
  if (initial_guess.size() == static_cast<Eigen::Index>(dof)) {
    q_current_ = initial_guess;
  } else if (initial_guess.size() == 0 && warm_start_ &&
             warm_start_->size() == static_cast<Eigen::Index>(dof)) {
    q_current_ = *warm_start_;
  } else if (initial_guess.size() == 0) {
    q_current_.setZero();
  } else {
    throw std::invalid_argument("Initial guess dimension mismatch");
  }

  clampToJointLimits(q_current_);

  // Compute joint bounds once (reuse pre-allocated arrays)
  computeJointBounds(lower_bounds_, upper_bounds_);

  status.error_history.reserve(config_.max_iterations);

  size_t stagnation_counter = 0;
  double prev_error_norm = std::numeric_limits<double>::max();

  // Random number generator for stagnation escape
  std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  KINEX_LOG_DEBUG("[IK] Starting solve: DOF={}, max_iter={}, tolerance={}", dof,
                  config_.max_iterations, config_.tolerance);

  for (size_t iter = 0; iter < config_.max_iterations; ++iter) {
    // FK/Jacobian fusion: compute both in one pass to reuse transforms
    Transform current_pose =
        fk_.computeWithStateAndJacobian(q_current_, robot_state_, J_);

    // Build and weight error (reuse pre-allocated arrays)
    weighted_error_.noalias() =
        weightError(buildTaskError(current_pose, target_pose));
    double error_norm = weighted_error_.norm();
    status.iterations = iter + 1;
    status.error_history.push_back(error_norm);

    if (error_norm < config_.tolerance) {
      status.converged = true;
      status.final_error_norm = error_norm;
      KINEX_LOG_DEBUG("[IK] Converged at iteration {}: error={:.6e}", iter,
                      error_norm);
      break;
    }

    // Stagnation detection and escape
    if (std::abs(prev_error_norm - error_norm) < config_.stagnation_threshold) {
      stagnation_counter++;

      if (stagnation_counter >= config_.stagnation_detect_iters) {
        KINEX_LOG_WARN("[IK] Stagnation detected at iter {}, error={:.6e}",
                       iter, error_norm);

        if (config_.enable_stagnation_escape) {
          // Apply random perturbation
          for (size_t i = 0; i < dof; ++i) {
            q_current_[i] += dist(rng) * config_.stagnation_perturbation;
          }
          clampToJointLimits(q_current_);
          stagnation_counter = 0;
          prev_error_norm =
              std::numeric_limits<double>::max(); // Reset error history
          KINEX_LOG_INFO(
              "[IK] Applied random perturbation to escape stagnation");
          continue; // Skip optimization step and re-evaluate FK
        }
      }
    } else {
      stagnation_counter = 0;
    }
    prev_error_norm = error_norm;

    // Weight Jacobian (use noalias for efficiency)
    weighted_jac_.noalias() = weightJacobian(J_);

    // Compute Hessian approximation (reuse pre-allocated H_)
    H_.noalias() = weighted_jac_.transpose() * weighted_jac_;

    // Compute manipulability for adaptive strategies
    double w = computeManipulability(J_);

    // Apply damping based on selected strategy
    double damping = config_.regularization;

    if (config_.enable_svd_damping) {
      // Use SVD-based damping for direction-dependent regularization
      applySVDDamping(H_, J_, config_.svd_damping_lambda_max,
                      config_.svd_singular_threshold);
    }

    if (config_.enable_adaptive_damping) {
      // Add scalar damping based on manipulability
      if (w < config_.damping_threshold) {
        double factor = (1.0 - w / config_.damping_threshold);
        damping += config_.max_damping * factor * factor;
      }
    }

    H_.diagonal().array() += damping;

    // Compute gradient (reuse pre-allocated g_)
    // Primary term: minimize task error
    g_.noalias() = -weighted_jac_.transpose() * weighted_error_;

    // Add manipulability gradient term (singularity avoidance)
    // This pushes the solution away from singular configurations
    if (config_.enable_manipulability_gradient) {
      manip_gradient_ = computeManipulabilityGradient(q_current_);

      // Dynamic weight: decreases as we approach convergence
      // This ensures manipulability doesn't interfere with final precision
      double convergence_ratio = error_norm / config_.tolerance;
      double convergence_decay = std::min(1.0, convergence_ratio);

      // Scale the manipulability weight based on:
      // 1. How close to singularity we are (increase near singularity)
      // 2. How close to convergence we are (decrease near convergence)
      double manip_weight = config_.manipulability_weight * convergence_decay;

      if (w < config_.manipulability_min_threshold && w > 1e-10) {
        // Increase weight as we approach singularity
        double ratio = config_.manipulability_min_threshold / w;
        manip_weight *= config_.manipulability_scale_factor * ratio;
      }

      // The gradient of -w (negative because we want to maximize w)
      // gets added to the cost gradient
      g_.noalias() -= manip_weight * manip_gradient_;

      KINEX_LOG_DEBUG("[IK] iter {}: manipulability={:.6e}, manip_weight={:.6e}, "
                      "convergence_decay={:.4f}",
                      iter, w, manip_weight, convergence_decay);
    }

    // Compute delta bounds (reuse pre-allocated arrays)
    lower_delta_.noalias() = lower_bounds_ - q_current_;
    upper_delta_.noalias() = upper_bounds_ - q_current_;

    bool qp_ok =
        qp_solver_ && qp_solver_->solve(H_, g_, lower_delta_, upper_delta_,
                                        delta_, status.qp_status);

    if (!qp_ok) {
      status.qp_status = EXIT_NONCONVEX;

      if (config_.enable_slack_variables) {
        // Use SVD-based damped pseudoinverse as fallback
        // This approach is more robust near singularities
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            weighted_jac_, Eigen::ComputeThinU | Eigen::ComputeThinV);

        const Eigen::VectorXd &sigma = svd.singularValues();
        const Eigen::MatrixXd &U = svd.matrixU();
        const Eigen::MatrixXd &V = svd.matrixV();

        // Compute damped pseudoinverse: J^+ = V * Σ^+ * U^T
        // where Σ^+ uses Tikhonov regularization: σ_i / (σ_i² + λ²)
        double lambda_sq = config_.slack_penalty * config_.slack_penalty;
        if (w < config_.damping_threshold) {
          // Increase damping near singularity
          lambda_sq *= (config_.damping_threshold / (w + 1e-10));
        }

        Eigen::MatrixXd sigma_inv = Eigen::MatrixXd::Zero(V.cols(), U.cols());
        for (Eigen::Index i = 0; i < sigma.size(); ++i) {
          double si = sigma(i);
          sigma_inv(i, i) = si / (si * si + lambda_sq);
        }

        Eigen::MatrixXd J_pinv_damped = V * sigma_inv * U.transpose();
        delta_.noalias() = J_pinv_damped * weighted_error_;

        // Add manipulability gradient if enabled
        if (config_.enable_manipulability_gradient) {
          // Project manipulability gradient into null space
          // N = I - J^+ * J
          Eigen::Index dof_idx = static_cast<Eigen::Index>(dof);
          Eigen::MatrixXd J_pinv_J = J_pinv_damped * weighted_jac_;
          Eigen::MatrixXd N =
              Eigen::MatrixXd::Identity(dof_idx, dof_idx) - J_pinv_J;
          delta_.noalias() +=
              config_.manipulability_weight * (N * manip_gradient_);
        }

        status.message = "Using damped pseudoinverse fallback";
        KINEX_LOG_DEBUG("[IK] QP failed, using damped pseudoinverse");
      } else {
        // Original LDLT fallback
        delta_.noalias() = H_.ldlt().solve(-g_);
        status.message = "Fallback damping step";
      }
    }

    double step_norm = delta_.norm();
    status.final_step_norm = step_norm;
    if (step_norm > config_.max_step_size && step_norm > 1e-12) {
      delta_ *= config_.max_step_size / step_norm;
    }

    double alpha = 1.0;
    bool accepted = false;
    double current_norm = error_norm;
    for (size_t ls = 0; ls < config_.max_line_search_steps; ++ls) {
      if (alpha < config_.line_search_min_alpha) {
        break;
      }
      Eigen::VectorXd candidate = q_current_ + alpha * delta_;
      candidate = candidate.cwiseMax(lower_bounds_).cwiseMin(upper_bounds_);
      Transform pose_candidate = fk_.compute(candidate);
      Eigen::VectorXd candidate_error =
          weightError(buildTaskError(pose_candidate, target_pose));
      double candidate_norm = candidate_error.norm();
      if (candidate_norm + config_.line_search_improvement < current_norm) {
        q_current_ = candidate;
        error_norm = candidate_norm;
        accepted = true;
        break;
      }
      alpha *= config_.line_search_shrink;
    }

    if (!accepted) {
      // Line search failed - try a more conservative approach
      if (config_.enable_slack_variables && w < config_.damping_threshold) {
        // Near singularity with failed line search: use very small step
        // in the direction that improves manipulability
        double small_alpha = config_.line_search_min_alpha * 0.1;
        Eigen::VectorXd conservative_step = small_alpha * delta_;

        // Also move slightly in the manipulability gradient direction
        if (config_.enable_manipulability_gradient && manip_gradient_.norm() > 1e-10) {
          conservative_step += small_alpha * config_.manipulability_weight *
                               manip_gradient_.normalized();
        }

        q_current_ += conservative_step;
        q_current_ = q_current_.cwiseMax(lower_bounds_).cwiseMin(upper_bounds_);

        KINEX_LOG_DEBUG("[IK] Line search failed near singularity, using "
                        "conservative step with manip gradient");
      } else {
        // Standard fallback: apply the computed step
        q_current_ += delta_;
        q_current_ = q_current_.cwiseMax(lower_bounds_).cwiseMin(upper_bounds_);
      }
    }

    status.final_error_norm = error_norm;
  }

  solution = q_current_;
  if (config_.enable_warm_start) {
    warm_start_ = q_current_;
  }

  if (!status.converged) {
    Transform final_pose = fk_.compute(q_current_);
    Eigen::VectorXd final_task_error = buildTaskError(final_pose, target_pose);
    status.final_error_norm = weightError(final_task_error).norm();
    status.message = "Maximum iterations reached";

    // Log detailed failure information
    // Compute Jacobian for failure analysis (we can reuse the last computed
    // one)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_, Eigen::ComputeThinU |
                                                  Eigen::ComputeThinV);
    const auto &singular_values = svd.singularValues();
    double cond_number =
        singular_values(0) / singular_values(singular_values.size() - 1);

    std::ostringstream oss;
    oss << "[IK] FAILED after " << status.iterations << " iterations:\n";
    oss << "  Final error norm: " << std::scientific << std::setprecision(4)
        << status.final_error_norm << "\n";
    oss << "  Position error: [" << final_task_error[0] << ", "
        << final_task_error[1] << ", " << final_task_error[2] << "]\n";
    oss << "  Orientation error: [" << final_task_error[3] << ", "
        << final_task_error[4] << ", " << final_task_error[5] << "]\n";
    oss << "  Position error norm: " << final_task_error.head<3>().norm()
        << "\n";
    oss << "  Orientation error norm: " << final_task_error.tail<3>().norm()
        << "\n";
    oss << "  Jacobian condition number: " << std::fixed << std::setprecision(2)
        << cond_number << "\n";
    oss << "  Singular values: [";
    for (int i = 0; i < singular_values.size(); ++i) {
      if (i > 0)
        oss << ", ";
      oss << std::scientific << std::setprecision(2) << singular_values(i);
    }
    oss << "]\n";
    oss << "  Final joint config: [";
    for (size_t i = 0; i < dof; ++i) {
      if (i > 0)
        oss << ", ";
      oss << std::fixed << std::setprecision(3) << q_current_[i];
    }
    oss << "]\n";
    oss << "  Error history (last 10): [";
    size_t start_idx =
        status.error_history.size() > 10 ? status.error_history.size() - 10 : 0;
    for (size_t i = start_idx; i < status.error_history.size(); ++i) {
      if (i > start_idx)
        oss << ", ";
      oss << std::scientific << std::setprecision(2) << status.error_history[i];
    }
    oss << "]";
    KINEX_LOG_WARN("{}", oss.str());
  } else {
    status.message = "IK converged";
    KINEX_LOG_DEBUG("[IK] Success: iterations={}, final_error={:.6e}",
                    status.iterations, status.final_error_norm);
  }

  return status;
}

} // namespace kinex
