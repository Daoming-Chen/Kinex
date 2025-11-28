# jacobian-computation Specification Delta

## MODIFIED Requirements

### Requirement: JacobianCalculator SHALL compute Jacobian matrices using RobotModel
The JacobianCalculator SHALL compute analytical and geometric Jacobians using a RobotModel (previously called Robot) and Eigen linear algebra.

#### Scenario: Create JacobianCalculator with RobotModel
**GIVEN** a RobotModel parsed from UR5e URDF
**WHEN** the user creates `JacobianCalculator calc(model, "tool0")`
**THEN** the calculator is initialized for the kinematic chain to "tool0"
**AND** the calculator can compute Jacobians for any joint configuration

## ADDED Requirements

### Requirement: Robot class SHALL provide convenient Jacobian computation methods
The Robot class SHALL provide simplified methods for computing Jacobians and manipulability metrics without requiring users to instantiate JacobianCalculator objects.

#### Scenario: Compute Jacobian using default end-effector
**GIVEN** a Robot instance created with `Robot::fromURDF("ur5e.urdf", "tool0")`
**AND** joint angles q = [0, -π/2, 0, -π/2, 0, 0]
**WHEN** the user calls `robot.computeJacobian(q)`
**THEN** the system returns a 6×N Jacobian matrix (N = number of joints)
**AND** the result matches calling JacobianCalculator::compute() directly
**AND** the Jacobian type is Analytic by default

#### Scenario: Compute Jacobian for specific link
**GIVEN** a Robot instance with default end-effector "tool0"
**AND** joint angles q
**WHEN** the user calls `robot.computeJacobian(q, "wrist_2_link")`
**THEN** the system returns the Jacobian for "wrist_2_link"
**AND** the default end-effector is not changed
**AND** the Robot lazily creates a JacobianCalculator for "wrist_2_link"

#### Scenario: Compute geometric Jacobian
**GIVEN** a Robot instance
**AND** joint angles q
**WHEN** the user calls `robot.computeJacobian(q, "", JacobianType::Geometric)`
**THEN** the system returns the geometric (body) Jacobian
**AND** the Jacobian is expressed in the end-effector frame

#### Scenario: Repeated Jacobian calls use cached calculator
**GIVEN** a Robot instance
**WHEN** the user calls `robot.computeJacobian(q1)` then `robot.computeJacobian(q2)`
**THEN** the internal JacobianCalculator is created only once
**AND** both calls reuse the cached instance
**AND** no performance penalty compared to direct usage

### Requirement: Robot class SHALL provide manipulability analysis methods
The Robot class SHALL expose manipulability metrics and singularity detection for configuration analysis.

#### Scenario: Compute Yoshikawa manipulability measure
**GIVEN** a Robot instance
**AND** joint angles q
**WHEN** the user calls `robot.getManipulability(q)`
**THEN** the system returns the Yoshikawa manipulability index
**AND** the value is non-negative
**AND** the value approaches zero near singularities
**AND** high values indicate better manipulability

#### Scenario: Check for singular configuration
**GIVEN** a Robot instance
**AND** joint angles q placing the robot near a singularity
**WHEN** the user calls `robot.isSingular(q)`
**THEN** the system returns true if the Jacobian is singular (default threshold 1e-6)
**AND** the user can override threshold: `robot.isSingular(q, 1e-4)`

#### Scenario: Get Jacobian condition number
**GIVEN** a Robot instance
**AND** joint angles q
**WHEN** the user calls `robot.getConditionNumber(q)`
**THEN** the system returns the condition number of the Jacobian
**AND** large values (>100) indicate ill-conditioned configurations
**AND** values near 1.0 indicate well-conditioned configurations

#### Scenario: Manipulability methods use default end-effector
**GIVEN** a Robot with default end-effector "tool0"
**WHEN** the user calls manipulability methods without link parameter
**THEN** all methods analyze manipulability for "tool0"
**AND** results are consistent across all methods

#### Scenario: Manipulability methods accept custom link
**GIVEN** a Robot instance
**WHEN** the user calls `robot.getManipulability(q, "wrist_3_link")`
**THEN** the system analyzes manipulability for "wrist_3_link"
**AND** the JacobianCalculator for "wrist_3_link" is cached for reuse
