# C++ Tutorial

## Introduction

This tutorial covers the basic usage of kinex for C++ developers. Kinex provides two levels of API:
1. **Unified Robot API (`kinex::Robot`)**: High-level, easy-to-use interface (Recommended).
2. **Core API (`kinex::RobotModel`, `kinex::ForwardKinematics`, etc.)**: Low-level components for advanced usage.

## Prerequisites

- C++20 compatible compiler (GCC 10+, Clang 10+, MSVC 19.29+)
- CMake 3.20+
- Eigen3 library

## 1. Unified Robot API (Recommended)

The `kinex::Robot` class provides a unified interface for all common robotics operations.

### Loading a Robot

```cpp
#include <kinex/robot.h>
#include <iostream>

int main() {
    // Load robot from URDF file
    // Specify the end-effector link name
    auto robot = kinex::Robot::fromURDF("path/to/robot.urdf", "tool0");
    
    std::cout << "Robot name: " << robot.getName() << std::endl;
    std::cout << "DOF: " << robot.getDOF() << std::endl;
    
    return 0;
}
```

### Forward Kinematics

```cpp
#include <kinex/robot.h>

// Create joint configuration (Eigen::VectorXd)
Eigen::VectorXd q(6);
q << 0, M_PI/4, -M_PI/4, 0, M_PI/2, 0;

// Compute pose of the default end-effector
kinex::Transform pose = robot.forwardKinematics(q);

std::cout << "Position: " << pose.translation().transpose() << std::endl;
std::cout << "Rotation:\n" << pose.rotation() << std::endl;

// Compute pose of a specific link
kinex::Transform link_pose = robot.forwardKinematics(q, "forearm_link");
```

### Inverse Kinematics

```cpp
#include <kinex/robot.h>

// Define target pose
kinex::Transform target;
target.setTranslation(Eigen::Vector3d(0.5, 0.2, 0.4));
// ... set rotation ...

// Initial guess
Eigen::VectorXd q_init = Eigen::VectorXd::Zero(robot.getDOF());

// Solve IK
auto [solution, status] = robot.inverseKinematics(target, q_init);

if (status.converged) {
    std::cout << "IK Solution: " << solution.transpose() << std::endl;
} else {
    std::cout << "IK Failed: " << status.message << std::endl;
}
```

### Jacobian and Dynamics

```cpp
// Compute Analytic Jacobian
Eigen::MatrixXd J = robot.computeJacobian(q);

// Check manipulability
double m = robot.getManipulability(q);
bool singular = robot.isSingular(q);
```

## 2. Core API (Advanced)

For performance-critical applications or custom solver integration, you can use the core classes directly.

### RobotModel

The `RobotModel` class represents the structure of the robot (links, joints) but contains no solver state.

```cpp
#include <kinex/robot_model.h>
#include <kinex/urdf_parser.h>

kinex::URDFParser parser;
std::shared_ptr<kinex::RobotModel> model = parser.parseFile("robot.urdf");
```

### Solvers

Solvers are instantiated separately and hold their own state/cache.

```cpp
#include <kinex/kinematics.h>
#include <kinex/inverse_kinematics.h>

// Forward Kinematics
kinex::ForwardKinematics fk(model, "tool0");
kinex::Transform pose = fk.compute(q);

// Inverse Kinematics
kinex::SQPIKSolver ik_solver(model, "tool0");
Eigen::VectorXd solution;
kinex::SolverStatus status = ik_solver.solve(target, q_init, solution);
```

## More Examples

See `examples/cpp/` for complete working examples.
