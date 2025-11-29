#include "kinex/logging.h"
#include "kinex/robot.h"
#include <filesystem>
#include <future>
#include <gtest/gtest.h>
#include <thread>

// Define M_PI for MSVC
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace kinex;

class RobotTest : public ::testing::Test {
protected:
  void SetUp() override {
    setLogLevel(spdlog::level::warn);

    // Simple 2-link robot URDF
    simple_urdf_ = R"(
<?xml version="1.0"?>
<robot name="simple_robot">
    <link name="base_link"/>
    <link name="link1"/>
    <link name="link2"/>
    
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    </joint>
    
    <joint name="joint2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="1 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    </joint>
</robot>
)";

    ur5e_urdf_path_ = std::filesystem::path(__FILE__)
                          .parent_path()
                          .parent_path()
                          .parent_path() /
                      "examples" / "models" / "ur5" / "ur5e.urdf";
  }

  std::string simple_urdf_;
  std::filesystem::path ur5e_urdf_path_;
};

TEST_F(RobotTest, CreationFromURDFString) {
  auto robot = Robot::fromURDFString(simple_urdf_, "link2", "base_link");
  EXPECT_EQ(robot.getName(), "simple_robot");
  EXPECT_EQ(robot.getEndLink(), "link2");
  EXPECT_EQ(robot.getBaseLink(), "base_link");
  EXPECT_EQ(robot.getDOF(), 2);
  EXPECT_NE(robot.getRobotModel(), nullptr);
}

TEST_F(RobotTest, CreationFromURDFFile) {
  if (!std::filesystem::exists(ur5e_urdf_path_)) {
    GTEST_SKIP() << "UR5e URDF not found";
  }

  auto robot = Robot::fromURDF(ur5e_urdf_path_.string(), "wrist_3_link");
  EXPECT_EQ(robot.getName(), "converted_robot"); // Name from URDF
  EXPECT_EQ(robot.getEndLink(), "wrist_3_link");
  EXPECT_EQ(robot.getDOF(), 6);
}

TEST_F(RobotTest, ForwardKinematics) {
  auto robot = Robot::fromURDFString(simple_urdf_, "link2", "base_link");

  // Zero config -> (1, 0, 0)
  Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
  Transform pose = robot.forwardKinematics(q);

  EXPECT_NEAR(pose.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(pose.translation().y(), 0.0, 1e-6);

  // 90 degree rotations
  q << M_PI / 2, 0.0;
  pose = robot.forwardKinematics(q);
  EXPECT_NEAR(pose.translation().x(), 0.0, 1e-6);
  EXPECT_NEAR(pose.translation().y(), 1.0, 1e-6);

  // Test alias
  Transform pose2 = robot.computePose(q);
  EXPECT_TRUE(pose.asMatrix().isApprox(pose2.asMatrix()));
}

TEST_F(RobotTest, InverseKinematics) {
  auto robot = Robot::fromURDFString(simple_urdf_, "link2", "base_link");

  Eigen::VectorXd target_q(2);
  target_q << 0.5, -0.5;
  Transform target_pose = robot.forwardKinematics(target_q);

  Eigen::VectorXd q_init = Eigen::VectorXd::Zero(2);
  auto [sol, status] = robot.inverseKinematics(target_pose, q_init);

  EXPECT_TRUE(status.converged);

  Transform result_pose = robot.forwardKinematics(sol);
  EXPECT_LT((result_pose.translation() - target_pose.translation()).norm(),
            1e-4);

  // Test alias
  auto [sol2, status2] = robot.solveIK(target_pose, q_init);
  EXPECT_TRUE(status2.converged);
}

TEST_F(RobotTest, Jacobian) {
  auto robot = Robot::fromURDFString(simple_urdf_, "link2", "base_link");
  Eigen::VectorXd q = Eigen::VectorXd::Zero(2);

  Eigen::MatrixXd J = robot.computeJacobian(q);
  EXPECT_EQ(J.rows(), 6);
  EXPECT_EQ(J.cols(), 2);

  // Test manipulability
  double m = robot.getManipulability(q);
  EXPECT_GE(m, 0.0);

  // Test singularity
  // For 6x2 Jacobian, columns are independent (rank 2) even at q=0
  // Col1: (0,2,0,0,0,1), Col2: (0,1,0,0,0,1) are independent
  double cond = robot.getConditionNumber(q);
  EXPECT_LT(cond, 100.0);
  EXPECT_FALSE(robot.isSingular(q));
}

TEST_F(RobotTest, Clone) {
  auto robot = Robot::fromURDFString(simple_urdf_, "link2", "base_link");

  // Configure original
  robot.setIKTolerance(1e-5);
  robot.setPositionOnlyIK(true);

  // Clone
  auto cloned = robot.clone();

  // Check properties copied
  EXPECT_EQ(cloned.getName(), robot.getName());
  EXPECT_EQ(cloned.getDOF(), robot.getDOF());

  // Check config copied
  auto config = cloned.getSolverConfig();
  EXPECT_EQ(config.tolerance, 1e-5);

  // Verify independence
  // Modifying clone config shouldn't affect original
  cloned.setIKTolerance(1e-2);
  EXPECT_EQ(robot.getSolverConfig().tolerance, 1e-5);
  EXPECT_EQ(cloned.getSolverConfig().tolerance, 1e-2);
}

TEST_F(RobotTest, ThreadSafety) {
  auto robot = Robot::fromURDFString(simple_urdf_, "link2", "base_link");

  // Use clones in threads
  auto task = [&robot]() {
    auto thread_robot = robot.clone();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
    for (int i = 0; i < 100; ++i) {
      thread_robot.forwardKinematics(q);
    }
    return true;
  };

  auto f1 = std::async(std::launch::async, task);
  auto f2 = std::async(std::launch::async, task);

  EXPECT_TRUE(f1.get());
  EXPECT_TRUE(f2.get());
}

TEST_F(RobotTest, LinkOverride) {
  auto robot = Robot::fromURDFString(simple_urdf_, "link2", "base_link");

  Eigen::VectorXd q = Eigen::VectorXd::Zero(2);

  // FK to link1 - requires joint vector matching the sub-chain (1 joint)
  // Note: In a future update, we might want Robot class to automatically slice
  // q
  Eigen::VectorXd q_sub = q.head(1);
  Transform pose1 = robot.forwardKinematics(q_sub, "link1");
  EXPECT_NEAR(pose1.translation().x(), 0.0, 1e-6); // link1 is at origin

  // FK to default (link2)
  Transform pose2 = robot.forwardKinematics(q);
  EXPECT_NEAR(pose2.translation().x(), 1.0, 1e-6);
}

TEST_F(RobotTest, InvalidLink) {
  auto robot = Robot::fromURDFString(simple_urdf_, "link2", "base_link");
  Eigen::VectorXd q = Eigen::VectorXd::Zero(2);

  EXPECT_THROW(robot.forwardKinematics(q, "bad_link"), std::invalid_argument);
}
