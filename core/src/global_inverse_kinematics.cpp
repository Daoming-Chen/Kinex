#include "kinex/global_inverse_kinematics.h"
#include "kinex/robot_model.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace kinex {

GlobalIKSolver::GlobalIKSolver(std::shared_ptr<const RobotModel> robot,
                               const std::string &end_link,
                               const std::string &base_link)
    : robot_(robot), end_link_(end_link), base_link_(base_link) {
  // Initialize random number generator
  std::random_device rd;
  rng_.seed(rd());
}

void GlobalIKSolver::setConfig(const GlobalSolverConfig &config) {
  config_ = config;
}

Eigen::VectorXd GlobalIKSolver::sampleRandomConfiguration() const {
  auto joints = robot_->getActuatedJoints();
  Eigen::VectorXd q(joints.size());

  std::uniform_real_distribution<double> dist(0.0, 1.0);

  for (size_t i = 0; i < joints.size(); ++i) {
    const auto &joint = joints[i];
    double lower = -M_PI;
    double upper = M_PI;

    if (joint->getLimits()) {
      lower = joint->getLimits()->lower;
      upper = joint->getLimits()->upper;
    } else if (joint->getType() == JointType::Continuous) {
      lower = -M_PI;
      upper = M_PI;
    } else {
      // For unbounded joints that are not continuous, use a reasonable range
      // or the unbounded limit from config if we had access to it here.
      // Using -PI to PI is a safe default for initial sampling.
      lower = -M_PI;
      upper = M_PI;
    }

    double t = dist(rng_);
    q[i] = lower + t * (upper - lower);
  }
  return q;
}

GlobalSolverResult GlobalIKSolver::solve(const Transform &target_pose,
                                         const Eigen::VectorXd &initial_guess) {
  GlobalSolverResult result;
  
  // Generate start configurations (seeds)
  std::vector<Eigen::VectorXd> start_configs;
  start_configs.reserve(config_.num_seeds);
  
  // First attempt uses the provided initial guess
  start_configs.push_back(initial_guess);
  
  // Remaining attempts use random seeds
  for (size_t i = 1; i < config_.num_seeds; ++i) {
    start_configs.push_back(sampleRandomConfiguration());
  }

  // Launch async tasks
  std::vector<std::future<std::pair<SolverStatus, Eigen::VectorXd>>> futures;
  futures.reserve(start_configs.size());

  for (const auto &start_config : start_configs) {
    futures.push_back(std::async(std::launch::async, [this, target_pose, start_config]() {
      SQPIKSolver solver(robot_, end_link_, base_link_);
      solver.setSolverConfig(config_.sqp_config);
      Eigen::VectorXd solution;
      SolverStatus status = solver.solve(target_pose, start_config, solution);
      return std::make_pair(status, solution);
    }));
  }

  // Wait for results with timeout
  auto start_time = std::chrono::steady_clock::now();
  
  for (auto &f : futures) {
    // Calculate remaining time
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    long remaining = (long)config_.max_time_ms - elapsed;
    if (remaining < 0) remaining = 0;

    if (f.wait_for(std::chrono::milliseconds(remaining)) == std::future_status::ready) {
      auto [status, solution] = f.get();
      if (status.converged) {
        // Check for duplicates
        bool is_duplicate = false;
        for (const auto &existing : result.solutions) {
          if ((existing - solution).norm() < config_.unique_threshold) {
            is_duplicate = true;
            break;
          }
        }
        
        if (!is_duplicate) {
          result.solutions.push_back(solution);
          result.statuses.push_back(status);
        }
      }
    }
  }

  if (result.solutions.empty()) {
    result.success = false;
    return result;
  }

  result.success = true;

  // Sort solutions to find the best one
  std::vector<size_t> indices(result.solutions.size());
  for (size_t i = 0; i < indices.size(); ++i) indices[i] = i;

  std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
    const auto &sa = result.statuses[a];
    const auto &sb = result.statuses[b];
    
    // Primary criteria: Error norm
    if (std::abs(sa.final_error_norm - sb.final_error_norm) > 1e-6) {
      return sa.final_error_norm < sb.final_error_norm;
    }
    // Secondary criteria: Iterations
    return sa.iterations < sb.iterations;
  });

  // Reorder result based on sorted indices
  std::vector<Eigen::VectorXd> sorted_solutions;
  std::vector<SolverStatus> sorted_statuses;
  sorted_solutions.reserve(result.solutions.size());
  sorted_statuses.reserve(result.statuses.size());
  
  for (size_t idx : indices) {
    sorted_solutions.push_back(result.solutions[idx]);
    sorted_statuses.push_back(result.statuses[idx]);
  }
  
  result.solutions = sorted_solutions;
  result.statuses = sorted_statuses;
  result.best_status = result.statuses[0];

  if (!config_.return_all_solutions) {
    // Keep only the best one
    result.solutions.resize(1);
    result.statuses.resize(1);
  }

  return result;
}

} // namespace kinex
