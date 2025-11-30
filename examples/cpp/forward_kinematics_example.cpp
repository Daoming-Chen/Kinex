#include "kinex/logging.h"
#include "kinex/robot.h"
#include "kinex/urdf_parser.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace kinex;

void printTransform(const Transform &transform) {
  auto [position, quaternion] = transform.asPositionQuaternion();

  std::cout << "Position: [" << std::fixed << std::setprecision(4)
            << position.x() << ", " << position.y() << ", " << position.z()
            << "]" << std::endl;

  std::cout << "Quaternion (w,x,y,z): [" << quaternion.w() << ", "
            << quaternion.x() << ", " << quaternion.y() << ", "
            << quaternion.z() << "]" << std::endl;

  auto [pos_rpy, rpy] = transform.asPositionRPY();
  std::cout << "RPY: [" << rpy.x() << ", " << rpy.y() << ", " << rpy.z()
            << "] rad" << std::endl;
}

std::string resolveEndLink(std::shared_ptr<const RobotModel> model) {
  std::vector<std::string> candidates = {"tool0", "ee_link", "wrist_3_link"};
  for (const auto &name : candidates) {
    if (model->getLink(name)) {
      return name;
    }
  }
  // Fallback
  auto joints = model->getActuatedJoints();
  if (!joints.empty()) {
    return joints.back()->getChildLink();
  }
  return "";
}

int main(int argc, char **argv) {
  // Set logging level
  setLogLevel(spdlog::level::info);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <urdf_file>" << std::endl;
    return 1;
  }

  try {
    std::string urdf_path = argv[1];

    // Need to inspect model to find end link first if we want to be generic
    // Or we can just assume one
    // To be safe, let's parse model first
    URDFParser parser;
    auto model = parser.parseFile(urdf_path);
    std::string end_link = resolveEndLink(model);

    if (end_link.empty()) {
      KINEX_LOG_ERROR("Could not determine end-effector link");
      return 1;
    }

    KINEX_LOG_INFO("Loading robot '{}' with end-effector '{}'",
                   model->getName(), end_link);

    // Create unified Robot
    Robot robot(model, end_link);

    KINEX_LOG_INFO("Robot loaded successfully");
    KINEX_LOG_INFO("DOF: {}", robot.getDOF());

    // Test at zero configuration
    std::cout << "\n=== Zero Configuration ===" << std::endl;
    Eigen::VectorXd zero_config = Eigen::VectorXd::Zero(robot.getDOF());
    Transform zero_pose = robot.forwardKinematics(zero_config);
    printTransform(zero_pose);

    // Test at random configuration
    std::cout << "\n=== Random Configuration ===" << std::endl;
    Eigen::VectorXd random_config(robot.getDOF());
    std::cout << "Joint angles: [";
    for (size_t i = 0; i < robot.getDOF(); ++i) {
      random_config[i] = (i + 1) * 0.1;
      std::cout << random_config[i];
      if (i < robot.getDOF() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    Transform random_pose = robot.forwardKinematics(random_config);
    printTransform(random_pose);

    // Performance test
    std::cout << "\n=== Performance Test ===" << std::endl;
    const int num_iterations = 100000;

    // Force initialization of FK solver before loop
    robot.forwardKinematics(random_config);

    auto loop_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_iterations; ++i) {
      robot.forwardKinematics(random_config);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - loop_start);
    double avg_time_us = static_cast<double>(duration.count()) / num_iterations;

    std::cout << "Average FK computation time: " << avg_time_us << " μs"
              << std::endl;
    std::cout << "Throughput: " << (1000000.0 / avg_time_us) << " FK/second"
              << std::endl;

    return 0;

  } catch (const std::exception &e) {
    KINEX_LOG_ERROR("Error: {}", e.what());
    return 1;
  }
}