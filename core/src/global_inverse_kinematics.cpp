#include "kinex/global_inverse_kinematics.h"

#include "kinex/robot_model.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace kinex {

// ============================================================================
// ThreadPool Implementation
// ============================================================================

ThreadPool::ThreadPool(size_t num_threads) {
  workers_.reserve(num_threads);
  for (size_t i = 0; i < num_threads; ++i) {
    workers_.emplace_back([this] {
      while (true) {
        std::function<void()> task;
        {
          std::unique_lock<std::mutex> lock(queue_mutex_);
          condition_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
          if (stop_ && tasks_.empty()) {
            return;
          }
          task = std::move(tasks_.front());
          tasks_.pop();
        }
        task();
      }
    });
  }
}

ThreadPool::~ThreadPool() {
  shutdown();
}

void ThreadPool::shutdown() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_ = true;
  }
  condition_.notify_all();
  for (auto& worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

// ============================================================================
// GlobalIKSolver Implementation
// ============================================================================

GlobalIKSolver::GlobalIKSolver(std::shared_ptr<const RobotModel> robot,
                               const std::string& end_link, const std::string& base_link)
    : robot_(robot), end_link_(end_link), base_link_(base_link) {
  // Initialize random number generator
  std::random_device rd;
  rng_.seed(rd());
}

GlobalIKSolver::~GlobalIKSolver() {
  if (thread_pool_) {
    thread_pool_->shutdown();
  }
}

void GlobalIKSolver::setConfig(const GlobalSolverConfig& config) {
  config_ = config;
}

Eigen::VectorXd GlobalIKSolver::sampleRandomConfiguration() const {
  auto joints = robot_->getActuatedJoints();
  Eigen::VectorXd q(joints.size());

  std::uniform_real_distribution<double> dist(0.0, 1.0);

  for (size_t i = 0; i < joints.size(); ++i) {
    const auto& joint = joints[i];
    double lower = -M_PI;
    double upper = M_PI;

    if (joint->getLimits()) {
      lower = joint->getLimits()->lower;
      upper = joint->getLimits()->upper;
    } else if (joint->getType() == JointType::Continuous) {
      lower = -M_PI;
      upper = M_PI;
    } else {
      lower = -M_PI;
      upper = M_PI;
    }

    double t = dist(rng_);
    q[i] = lower + t * (upper - lower);
  }
  return q;
}

void GlobalIKSolver::ensureThreadPool(size_t num_threads) {
  if (!thread_pool_ || current_pool_size_ != num_threads) {
    if (thread_pool_) {
      thread_pool_->shutdown();
    }
    thread_pool_ = std::make_unique<ThreadPool>(num_threads);
    current_pool_size_ = num_threads;
  }
}

void GlobalIKSolver::ensureSolverPool(size_t num_solvers) {
  std::lock_guard<std::mutex> lock(solver_pool_mutex_);
  while (solver_pool_.size() < num_solvers) {
    auto solver = std::make_unique<SQPIKSolver>(robot_, end_link_, base_link_);
    solver->setSolverConfig(config_.sqp_config);
    solver_pool_.push_back(std::move(solver));
  }
  // Update config for existing solvers
  for (auto& solver : solver_pool_) {
    solver->setSolverConfig(config_.sqp_config);
  }
}

std::pair<SolverStatus, Eigen::VectorXd> GlobalIKSolver::solveSingleSeed(
    const Transform& target_pose, const Eigen::VectorXd& start_config, size_t solver_index) {
  // Check early termination
  if (config_.early_termination && !config_.return_all_solutions && found_solution_.load()) {
    SolverStatus status;
    status.converged = false;
    status.message = "Early termination";
    return {status, Eigen::VectorXd()};
  }

  SQPIKSolver* solver = nullptr;
  {
    std::lock_guard<std::mutex> lock(solver_pool_mutex_);
    if (solver_index < solver_pool_.size()) {
      solver = solver_pool_[solver_index].get();
    }
  }

  if (!solver) {
    // Fallback: create temporary solver (should not happen normally)
    SQPIKSolver temp_solver(robot_, end_link_, base_link_);
    temp_solver.setSolverConfig(config_.sqp_config);
    Eigen::VectorXd solution;
    SolverStatus status = temp_solver.solve(target_pose, start_config, solution);
    if (status.converged) {
      found_solution_.store(true);
    }
    return {status, solution};
  }

  Eigen::VectorXd solution;
  SolverStatus status = solver->solve(target_pose, start_config, solution);
  if (status.converged) {
    found_solution_.store(true);
  }
  return {status, solution};
}

GlobalSolverResult GlobalIKSolver::solveSerial(const Transform& target_pose,
                                               const std::vector<Eigen::VectorXd>& start_configs) {
  GlobalSolverResult result;

  // Ensure we have at least one solver
  ensureSolverPool(1);

  for (size_t i = 0; i < start_configs.size(); ++i) {
    // Check early termination for robust mode
    if (config_.early_termination && !config_.return_all_solutions && !result.solutions.empty()) {
      break;
    }

    auto [status, solution] = solveSingleSeed(target_pose, start_configs[i], 0);

    if (status.converged) {
      // Check for duplicates
      bool is_duplicate = false;
      for (const auto& existing : result.solutions) {
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

  return result;
}

GlobalSolverResult GlobalIKSolver::solveParallel(
    const Transform& target_pose, const std::vector<Eigen::VectorXd>& start_configs) {
  GlobalSolverResult result;

  size_t num_threads = config_.num_threads;
  if (num_threads == 0) {
    num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0) num_threads = 4;
  }

  // Limit threads to number of seeds
  num_threads = std::min(num_threads, start_configs.size());

  // Ensure thread pool and solver pool
  ensureThreadPool(num_threads);
  ensureSolverPool(num_threads);

  // Reset early termination flag
  found_solution_.store(false);

  // Submit tasks
  std::vector<std::future<std::pair<SolverStatus, Eigen::VectorXd>>> futures;
  futures.reserve(start_configs.size());

  for (size_t i = 0; i < start_configs.size(); ++i) {
    size_t solver_idx = i % num_threads;
    futures.push_back(thread_pool_->submit(
        [this, &target_pose, config = start_configs[i], solver_idx]() {
          return solveSingleSeed(target_pose, config, solver_idx);
        }));
  }

  // Collect results with timeout
  auto start_time = std::chrono::steady_clock::now();

  for (auto& f : futures) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    long remaining = static_cast<long>(config_.max_time_ms) - elapsed;
    if (remaining < 0) remaining = 0;

    if (f.wait_for(std::chrono::milliseconds(remaining)) == std::future_status::ready) {
      auto [status, solution] = f.get();
      if (status.converged) {
        // Check for duplicates
        bool is_duplicate = false;
        for (const auto& existing : result.solutions) {
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

  return result;
}

GlobalSolverResult GlobalIKSolver::solve(const Transform& target_pose,
                                         const Eigen::VectorXd& initial_guess) {
  // Generate start configurations (seeds)
  std::vector<Eigen::VectorXd> start_configs;
  start_configs.reserve(config_.num_seeds);

  // First attempt uses the provided initial guess
  start_configs.push_back(initial_guess);

  // Remaining attempts use random seeds
  for (size_t i = 1; i < config_.num_seeds; ++i) {
    start_configs.push_back(sampleRandomConfiguration());
  }

  // Reset early termination flag
  found_solution_.store(false);

  // Choose execution mode
  GlobalSolverResult result;
  if (config_.use_serial || config_.num_seeds <= 2) {
    result = solveSerial(target_pose, start_configs);
  } else {
    result = solveParallel(target_pose, start_configs);
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
    const auto& sa = result.statuses[a];
    const auto& sb = result.statuses[b];

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

  result.solutions = std::move(sorted_solutions);
  result.statuses = std::move(sorted_statuses);
  result.best_status = result.statuses[0];

  if (!config_.return_all_solutions) {
    // Keep only the best one
    result.solutions.resize(1);
    result.statuses.resize(1);
  }

  return result;
}

}  // namespace kinex
