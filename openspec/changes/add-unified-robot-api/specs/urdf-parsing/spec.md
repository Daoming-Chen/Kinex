# urdf-parsing Specification Delta

## MODIFIED Requirements

### Requirement: URDFParser SHALL parse URDF files into robot structural models
The URDFParser SHALL parse URDF (Unified Robot Description Format) files and construct a RobotModel instance containing links, joints, and their properties. (Previously returned `Robot`, now returns `RobotModel` to clarify it represents only the structural model, not an operational robot interface.)

#### Scenario: Parse URDF file and return RobotModel
**GIVEN** a valid URDF file at path "robots/ur5e.urdf"
**WHEN** the user calls `parser.parseFile("robots/ur5e.urdf")`
**THEN** the system returns a `std::shared_ptr<RobotModel>`
**AND** the RobotModel contains all links and joints from the URDF
**AND** the RobotModel name matches the robot name in the URDF

#### Scenario: Parse URDF string and return RobotModel
**GIVEN** a valid URDF XML string
**WHEN** the user calls `parser.parseString(urdf_content)`
**THEN** the system returns a `std::shared_ptr<RobotModel>`
**AND** the RobotModel structure matches the XML specification

## ADDED Requirements

### Requirement: Robot class SHALL provide static factory methods for URDF loading
The unified Robot class SHALL provide convenient static factory methods that combine URDF parsing and robot initialization, creating a fully operational robot instance ready for kinematics operations.

#### Scenario: Create Robot from URDF file with default end-effector
**GIVEN** a valid URDF file at "robots/ur5e.urdf"
**WHEN** the user calls `Robot::fromURDF("robots/ur5e.urdf", "tool0")`
**THEN** the system returns a Robot instance
**AND** the Robot has "tool0" configured as the default end-effector
**AND** the Robot is immediately ready for FK/IK/Jacobian operations
**AND** the Robot internally holds an immutable RobotModel

#### Scenario: Create Robot from URDF string
**GIVEN** a valid URDF XML string
**WHEN** the user calls `Robot::fromURDFString(urdf_content, "end_effector")`
**THEN** the system returns a Robot instance
**AND** the instance is functionally equivalent to loading from file
**AND** no filesystem access is required

#### Scenario: Create Robot with custom base and end-effector links
**GIVEN** a URDF file "robot.urdf" with multiple potential end-effectors
**WHEN** the user calls `Robot::fromURDF("robot.urdf", "gripper_link", "base_link")`
**THEN** the system creates a Robot with custom base and end-effector
**AND** all kinematics operations use the specified kinematic chain

#### Scenario: URDF parsing error is propagated
**GIVEN** an invalid URDF file with syntax errors
**WHEN** the user calls `Robot::fromURDF("invalid.urdf", "tool0")`
**THEN** the system throws a URDFParseException
**AND** the exception message indicates the parsing error location
**AND** no Robot instance is created
