#include "kinex/global_inverse_kinematics.h"
#include "kinex/kinematics.h"
#include "kinex/logging.h"
#include "kinex/urdf_parser.h"
#include <gtest/gtest.h>
#include <filesystem>

using namespace kinex;

class GlobalInverseKinematicsTest : public ::testing::Test {
protected:
  void SetUp() override {
    setLogLevel(spdlog::level::err);
    auto root = std::filesystem::path(__FILE__)
                    .parent_path()
                    .parent_path()
                    .parent_path();
    urdf_path_ = root / "examples" / "models" / "ur5" / "ur5e.urdf";
    if (!std::filesystem::exists(urdf_path_)) {
      GTEST_SKIP() << "UR5e URDF not available";
    }

    URDFParser parser;
    robot_ = parser.parseFile(urdf_path_.string());
    ASSERT_NE(robot_, nullptr);

    // Find end link
    std::vector<std::string> candidates = {"tool0", "ee_link", "wrist_3_link"};
    for (const auto &name : candidates) {
      if (robot_->getLink(name)) {
        end_link_ = name;
        break;
      }
    }
    if (end_link_.empty()) end_link_ = "wrist_3_link"; // Fallback

    base_link_ = robot_->getRootLink();
    
    fk_ = std::make_unique<ForwardKinematics>(robot_, end_link_, base_link_);
  }

  std::filesystem::path urdf_path_;
  std::shared_ptr<RobotModel> robot_;
  std::string end_link_;
  std::string base_link_;
  std::unique_ptr<ForwardKinematics> fk_;
};

TEST_F(GlobalInverseKinematicsTest, Instantiation) {
  GlobalIKSolver solver(robot_, end_link_, base_link_);
  auto config = solver.getConfig();
  EXPECT_EQ(config.num_seeds, 16);
  EXPECT_FALSE(config.return_all_solutions);
}

TEST_F(GlobalInverseKinematicsTest, SolveRobust) {
  GlobalIKSolver solver(robot_, end_link_, base_link_);
  
  // Create a reachable target
  Eigen::VectorXd q_target(6);
  q_target << 0.5, -1.5, 1.5, -1.5, -1.5, 0.0;
  Transform target_pose = fk_->compute(q_target);
  
  Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(6);
  
  GlobalSolverResult result = solver.solve(target_pose, initial_guess);
  
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.solutions.size(), 1);
  EXPECT_EQ(result.statuses.size(), 1);
  
  // Check error
  Transform result_pose = fk_->compute(result.solutions[0]);
  double pos_error = (result_pose.translation() - target_pose.translation()).norm();
  EXPECT_LT(pos_error, 1e-3);
}

TEST_F(GlobalInverseKinematicsTest, SolveGlobal) {
  GlobalIKSolver solver(robot_, end_link_, base_link_);
  GlobalSolverConfig config;
  config.return_all_solutions = true;
  config.num_seeds = 32; // Increase seeds to find multiple solutions
  solver.setConfig(config);
  
  // Create a reachable target
  Eigen::VectorXd q_target(6);
  q_target << 0.5, -1.5, 1.5, -1.5, -1.5, 0.0;
  Transform target_pose = fk_->compute(q_target);
  
  Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(6);
  
  GlobalSolverResult result = solver.solve(target_pose, initial_guess);
  
  EXPECT_TRUE(result.success);
  // Should find at least one solution
  EXPECT_GE(result.solutions.size(), 1);
  
  // Check all solutions are valid
  for (const auto &sol : result.solutions) {
    Transform result_pose = fk_->compute(sol);
    double pos_error = (result_pose.translation() - target_pose.translation()).norm();
    EXPECT_LT(pos_error, 1e-3);
  }
}
