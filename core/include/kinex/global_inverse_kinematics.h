#pragma once

#include "kinex/export.h"
#include "kinex/inverse_kinematics.h"
#include <vector>
#include <future>
#include <random>

namespace kinex {

struct GlobalSolverConfig {
  size_t num_threads = 0; // 0 means hardware concurrency
  size_t num_seeds = 16;
  size_t max_time_ms = 1000;
  bool return_all_solutions = false; // false = robust (single best), true = global (all)
  double unique_threshold = 1e-3; // L2 norm threshold for deduplication
  
  // Underlying SQP solver config
  SolverConfig sqp_config;
};

struct GlobalSolverResult {
  bool success = false;
  std::vector<Eigen::VectorXd> solutions;
  std::vector<SolverStatus> statuses;
  SolverStatus best_status; // Status of the best solution (if any)
};

class KINEX_API GlobalIKSolver {
public:
  GlobalIKSolver(std::shared_ptr<const RobotModel> robot,
                 const std::string &end_link, const std::string &base_link = "");
  
  ~GlobalIKSolver() = default;

  GlobalSolverResult solve(const Transform &target_pose,
                           const Eigen::VectorXd &initial_guess);

  void setConfig(const GlobalSolverConfig &config);
  const GlobalSolverConfig &getConfig() const { return config_; }

private:
  std::shared_ptr<const RobotModel> robot_;
  std::string end_link_;
  std::string base_link_;
  GlobalSolverConfig config_;
  
  // Random number generation
  mutable std::mt19937 rng_;
  
  Eigen::VectorXd sampleRandomConfiguration() const;
};

} // namespace kinex
