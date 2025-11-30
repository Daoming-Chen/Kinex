#include "kinex/robot.h"
#include "kinex/urdf_parser.h"
#include <iostream>
#include <filesystem>
#include <vector>
#include <chrono>
#include <cmath>

using namespace kinex;

int main(int argc, char **argv) {
  // Load robot
  std::filesystem::path urdf_path = std::filesystem::path(__FILE__)
                                        .parent_path()
                                        .parent_path()
                                        .parent_path() /
                                    "examples" / "models" / "ur5" / "ur5e.urdf";
  
  if (!std::filesystem::exists(urdf_path)) {
    std::cerr << "URDF file not found: " << urdf_path << std::endl;
    return 1;
  }

  std::cout << "Loading robot from: " << urdf_path << std::endl;
  auto robot = Robot::fromURDF(urdf_path.string(), "wrist_3_link");
  
  // Generate random targets
  size_t num_targets = 100;
  std::vector<Eigen::VectorXd> targets;
  std::vector<Transform> target_poses;
  
  std::cout << "Generating " << num_targets << " random targets..." << std::endl;
  
  auto joints = robot.getRobotModel()->getActuatedJoints();
  
  for (size_t i = 0; i < num_targets; ++i) {
    Eigen::VectorXd q(joints.size());
    for (size_t j = 0; j < joints.size(); ++j) {
      double lower = -M_PI;
      double upper = M_PI;
      if (joints[j]->getLimits()) {
        lower = joints[j]->getLimits()->lower;
        upper = joints[j]->getLimits()->upper;
      }
      q[j] = lower + (upper - lower) * ((double)rand() / RAND_MAX);
    }
    targets.push_back(q);
    target_poses.push_back(robot.forwardKinematics(q));
  }
  
  // Configure Robot
  GlobalSolverConfig global_config;
  global_config.num_seeds = 32;
  robot.setGlobalSolverConfig(global_config);

  // Benchmark Standard IK (Single Start)
  std::cout << "\nBenchmarking Standard IK (Single Start)..." << std::endl;
  size_t success_standard = 0;
  auto start_standard = std::chrono::high_resolution_clock::now();
  
  for (const auto &pose : target_poses) {
    Eigen::VectorXd q_init = Eigen::VectorXd::Zero(joints.size());
    auto [sol, status] = robot.solveIK(pose, q_init);
    if (status.converged) success_standard++;
  }
  
  auto end_standard = std::chrono::high_resolution_clock::now();
  double time_standard = std::chrono::duration<double, std::milli>(end_standard - start_standard).count();
  
  std::cout << "Standard IK Success Rate: " << (double)success_standard / num_targets * 100.0 << "%" << std::endl;
  std::cout << "Average Time: " << time_standard / num_targets << " ms" << std::endl;
  
  // Benchmark Robust IK
  std::cout << "\nBenchmarking Robust IK..." << std::endl;
  size_t success_robust = 0;
  auto start_robust = std::chrono::high_resolution_clock::now();
  
  for (size_t i = 0; i < target_poses.size(); ++i) {
    const auto &pose = target_poses[i];
    Eigen::VectorXd q_init = Eigen::VectorXd::Zero(joints.size());
    
    auto [sol, status] = robot.solveRobustIK(pose, q_init);
    if (status.converged) {
        success_robust++;
    }
  }
  
  auto end_robust = std::chrono::high_resolution_clock::now();
  double time_robust = std::chrono::duration<double, std::milli>(end_robust - start_robust).count();
  
  std::cout << "Robust IK Success Rate: " << (double)success_robust / num_targets * 100.0 << "%" << std::endl;
  std::cout << "Average Time: " << time_robust / num_targets << " ms" << std::endl;
  std::cout << "Improvement: " << (double)(success_robust - success_standard) / num_targets * 100.0 << "%" << std::endl;

  
  // Demonstrate Global IK
  std::cout << "\nDemonstrating Global IK..." << std::endl;
  Transform pose = target_poses[0];
  Eigen::VectorXd q_init = Eigen::VectorXd::Zero(joints.size());
  
  GlobalSolverResult result = robot.solveGlobalIK(pose, q_init);
  
  std::cout << "Found " << result.solutions.size() << " solutions for target 0." << std::endl;
  for (size_t i = 0; i < result.solutions.size(); ++i) {
    std::cout << "Solution " << i << ": " << result.solutions[i].transpose() << std::endl;
  }
  
  return 0;
}
