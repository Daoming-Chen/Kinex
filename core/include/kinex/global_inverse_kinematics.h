#pragma once

#include "kinex/export.h"
#include "kinex/inverse_kinematics.h"
#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <random>
#include <thread>
#include <vector>

namespace kinex {

struct GlobalSolverConfig {
  size_t num_threads = 0;  // 0 means hardware concurrency
  size_t num_seeds = 4;    // Reduced from 16 for better performance
  size_t max_time_ms = 1000;
  bool return_all_solutions =
      false;                       // false = robust (single best), true = global (all)
  double unique_threshold = 1e-3;  // L2 norm threshold for deduplication

  // Early termination: stop when first valid solution is found (for robust mode)
  bool early_termination = true;

  // Use serial execution instead of parallel (lower latency for small num_seeds)
  bool use_serial = false;

  // Underlying SQP solver config
  SolverConfig sqp_config;
};

struct GlobalSolverResult {
  bool success = false;
  std::vector<Eigen::VectorXd> solutions;
  std::vector<SolverStatus> statuses;
  SolverStatus best_status;  // Status of the best solution (if any)
};

// Simple thread pool for reusing threads
class ThreadPool {
 public:
  explicit ThreadPool(size_t num_threads);
  ~ThreadPool();

  // Submit a task and get a future for the result
  template <typename F, typename... Args>
  auto submit(F&& f, Args&&... args)
      -> std::future<typename std::invoke_result<F, Args...>::type>;

  void shutdown();
  size_t size() const { return workers_.size(); }

 private:
  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> tasks_;
  std::mutex queue_mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stop_{false};
};

// Template implementation must be in header
template <typename F, typename... Args>
auto ThreadPool::submit(F&& f, Args&&... args)
    -> std::future<typename std::invoke_result<F, Args...>::type> {
  using return_type = typename std::invoke_result<F, Args...>::type;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));

  std::future<return_type> result = task->get_future();
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stop_) {
      throw std::runtime_error("Submit on stopped ThreadPool");
    }
    tasks_.emplace([task]() { (*task)(); });
  }
  condition_.notify_one();
  return result;
}

class KINEX_API GlobalIKSolver {
 public:
  GlobalIKSolver(std::shared_ptr<const RobotModel> robot, const std::string& end_link,
                 const std::string& base_link = "");

  ~GlobalIKSolver();

  GlobalSolverResult solve(const Transform& target_pose, const Eigen::VectorXd& initial_guess);

  void setConfig(const GlobalSolverConfig& config);
  const GlobalSolverConfig& getConfig() const { return config_; }

 private:
  std::shared_ptr<const RobotModel> robot_;
  std::string end_link_;
  std::string base_link_;
  GlobalSolverConfig config_;

  // Thread pool for parallel execution
  std::unique_ptr<ThreadPool> thread_pool_;
  size_t current_pool_size_ = 0;

  // Pre-allocated solver pool (one per thread)
  std::vector<std::unique_ptr<SQPIKSolver>> solver_pool_;
  std::mutex solver_pool_mutex_;

  // Random number generation
  mutable std::mt19937 rng_;

  // Early termination flag
  mutable std::atomic<bool> found_solution_{false};

  Eigen::VectorXd sampleRandomConfiguration() const;
  void ensureThreadPool(size_t num_threads);
  void ensureSolverPool(size_t num_solvers);

  // Serial solve implementation
  GlobalSolverResult solveSerial(const Transform& target_pose,
                                 const std::vector<Eigen::VectorXd>& start_configs);

  // Parallel solve implementation
  GlobalSolverResult solveParallel(const Transform& target_pose,
                                   const std::vector<Eigen::VectorXd>& start_configs);

  // Single seed solve (used by both serial and parallel)
  std::pair<SolverStatus, Eigen::VectorXd> solveSingleSeed(const Transform& target_pose,
                                                           const Eigen::VectorXd& start_config,
                                                           size_t solver_index);
};

}  // namespace kinex
