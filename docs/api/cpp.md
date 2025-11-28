# C++ API Documentation

This document provides detailed API documentation for the C++ bindings of Kinex.

## Table of Contents

- [Robot Class](#robot-class)
- [RobotModel Class](#robotmodel-class)
- [Exception Handling](#exception-handling)
- [Examples](#examples)

## Robot Class

The `Robot` class provides a unified high-level interface for all common robotics operations. This is the recommended way to use Kinex.

### Constructors

```cpp
// Static factory methods (recommended)
static Robot fromURDF(const std::string& filepath,
                      const std::string& end_link,
                      const std::string& base_link = "");

static Robot fromURDFString(const std::string& urdf_string,
                            const std::string& end_link,
                            const std::string& base_link = "");

// Copy operations
Robot clone() const;
```

### Forward Kinematics

```cpp
// Compute pose of default end-effector or specified link
Transform forwardKinematics(const Eigen::VectorXd& q,
                           const std::string& link = "") const;

// Alias for forwardKinematics
Transform computePose(const Eigen::VectorXd& q,
                     const std::string& link = "") const;
```

### Inverse Kinematics

```cpp
// Solve inverse kinematics for target pose
std::pair<Eigen::VectorXd, SolverStatus> inverseKinematics(
    const Transform& target,
    const Eigen::VectorXd& q_init,
    const std::string& link = "");

// Alias for inverseKinematics
std::pair<Eigen::VectorXd, SolverStatus> solveIK(
    const Transform& target,
    const Eigen::VectorXd& q_init,
    const std::string& link = "");
```

### Jacobian Analysis

```cpp
// Compute Jacobian matrix
Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& q,
                               const std::string& link = "",
                               JacobianType type = JacobianType::Analytic) const;

// Manipulability measures
double getManipulability(const Eigen::VectorXd& q,
                         const std::string& link = "") const;
bool isSingular(const Eigen::VectorXd& q,
                double threshold = 1e-6,
                const std::string& link = "") const;
double getConditionNumber(const Eigen::VectorXd& q,
                         const std::string& link = "") const;
```

### Configuration

```cpp
// IK solver configuration
void setIKTolerance(double tolerance);
void setPositionOnlyIK(bool enable);
void setOrientationOnlyIK(bool enable);
void setSolverConfig(const SolverConfig& config);
SolverConfig getSolverConfig() const;
```

### Properties

```cpp
// Robot information
std::shared_ptr<const RobotModel> getRobotModel() const;
const std::string& getName() const;
const std::string& getEndLink() const;
const std::string& getBaseLink() const;
size_t getDOF() const;
```

### Usage Example

```cpp
#include <kinex/robot.h>
#include <iostream>

int main() {
    // Load robot from URDF
    auto robot = kinex::Robot::fromURDF("ur5e.urdf", "tool0");

    // Set joint configuration
    Eigen::VectorXd q(6);
    q << 0, M_PI/4, -M_PI/4, 0, M_PI/2, 0;

    // Forward kinematics
    auto pose = robot.forwardKinematics(q);
    std::cout << "End-effector position: " << pose.translation().transpose() << std::endl;

    // Jacobian analysis
    auto J = robot.computeJacobian(q);
    double manipulability = robot.getManipulability(q);
    bool is_singular = robot.isSingular(q);

    // Inverse kinematics
    kinex::Transform target;
    target.setTranslation(Eigen::Vector3d(0.5, 0.2, 0.4));
    target.rotation() = Eigen::Matrix3d::Identity();

    Eigen::VectorXd q_init = Eigen::VectorXd::Zero(6);
    auto [solution, status] = robot.inverseKinematics(target, q_init);

    if (status.converged) {
        std::cout << "IK solution: " << solution.transpose() << std::endl;
    }

    return 0;
}
```

## RobotModel Class

The `RobotModel` class represents the structural model of a robot (links, joints, geometry). This is a low-level class for advanced users who need fine-grained control.

### Construction

```cpp
// Typically obtained from URDFParser
kinex::URDFParser parser;
auto model = parser.parseFile("robot.urdf");
```

### Usage Example

```cpp
#include <kinex/robot_model.h>
#include <kinex/kinematics.h>
#include <kinex/inverse_kinematics.h>

// Use with individual solvers
kinex::URDFParser parser;
auto model = parser.parseFile("robot.urdf");

// Forward kinematics
kinex::ForwardKinematics fk(model, "tool0");
auto pose = fk.compute(q);

// Inverse kinematics
kinex::SQPIKSolver ik_solver(model, "tool0");
kinex::SolverStatus status = ik_solver.solve(target, q_init, solution);

// Jacobian computation
kinex::JacobianCalculator jacobian(model, "tool0");
auto J = jacobian.compute(q);
```

## Exception Handling

Kinex throws various exceptions for error conditions:

```cpp
#include <kinex/exceptions.h>

try {
    auto robot = kinex::Robot::fromURDF("invalid.urdf", "tool0");
} catch (const kinex::URDFParseException& e) {
    std::cerr << "URDF parsing error: " << e.what() << std::endl;
} catch (const kinex::LinkNotFoundException& e) {
    std::cerr << "Link not found: " << e.what() << std::endl;
}
```

## Type Definitions

```cpp
// Common types used throughout the API
using Transform = Eigen::Transform<double, 3, Eigen::Isometry>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// Solver status
struct SolverStatus {
    bool converged;
    int iterations;
    double residual_error;
    std::string message;
};

// Jacobian computation types
enum class JacobianType {
    Analytic,       // Analytic Jacobian
    Geometric,      // Geometric Jacobian
    Angular         // Angular velocity Jacobian
};

// IK solver configuration
struct SolverConfig {
    double tolerance = 1e-6;
    int max_iterations = 100;
    bool position_only = false;
    bool orientation_only = false;
    // ... other configuration options
};
```

## Examples

See the [C++ examples](../../examples/cpp/) directory for complete working examples:

- **[forward_kinematics_example.cpp](../../examples/cpp/forward_kinematics_example.cpp)** - Basic FK usage
- **[inverse_kinematics_example.cpp](../../examples/cpp/inverse_kinematics_example.cpp)** - IK solving example
- **[jacobian_analysis.cpp](../../examples/cpp/jacobian_analysis.cpp)** - Manipulability analysis

## Thread Safety

- `Robot` objects are thread-safe for read operations
- Each `Robot` instance maintains its own solver state
- Use `clone()` to create independent copies for multi-threaded applications:

```cpp
auto robot1 = kinex::Robot::fromURDF("robot.urdf", "tool0");
auto robot2 = robot1.clone();  // Independent copy for different thread
```