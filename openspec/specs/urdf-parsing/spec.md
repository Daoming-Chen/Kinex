# urdf-parsing Specification

## Purpose
TBD - created by archiving change add-robotics-kinematics-library. Update Purpose after archive.
## Requirements
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

### Requirement: System SHALL extract robot structure
The system SHALL extract the complete robot structure including links, joints, and their hierarchical relationships.

#### Scenario: Build kinematic tree from URDF
**Given** a parsed Robot object from UR5e URDF  
**When** the user queries the kinematic structure  
**Then** the system provides the complete tree from "world" link to "tool0" link  
**And** each joint connects exactly one parent link to one child link  
**And** the tree has no cycles

#### Scenario: Access joint properties
**Given** a parsed Robot object  
**When** the user queries joint "shoulder_pan_joint"  
**Then** the system returns joint type as "revolute"  
**And** returns joint axis as [0, 0, 1]  
**And** returns joint limits (e.g., [-2π, 2π])  
**And** returns joint origin transformation

### Requirement: System SHALL parse visual geometry
The system SHALL parse visual geometry elements including mesh references.

#### Scenario: Load mesh file paths
**Given** a URDF with visual meshes  
**When** the system parses the file  
**Then** each visual element contains the mesh file path  
**And** the mesh origin transformation is stored  
**And** relative paths are resolved relative to URDF directory

### Requirement: System SHALL parse joint limits
The system SHALL extract and validate joint limit specifications.

#### Scenario: Extract revolute joint limits
**Given** a revolute joint with limits [-π, π]  
**When** the system parses the joint  
**Then** the joint lower limit is stored as -π  
**And** the joint upper limit is stored as π  
**And** velocity and effort limits are also extracted if present

#### Scenario: Handle unlimited joints
**Given** a continuous joint without limits  
**When** the system parses the joint  
**Then** the joint is marked as unlimited  
**And** the joint type is stored as "continuous"

### Requirement: System SHALL support URDF from string
The system SHALL support parsing URDF content from in-memory strings.

#### Scenario: Parse URDF from string buffer
**Given** a string containing valid URDF XML content  
**When** the user calls `parser.parseString(urdf_content)`  
**Then** the system returns a `RobotModel` shared pointer  
**And** the parsing behavior is identical to file-based parsing

### Requirement: System SHALL validate robot model
The system SHALL validate the parsed robot model for common errors.

#### Scenario: Detect disconnected links
**Given** a URDF with links not connected to the root  
**When** the system validates the robot model  
**Then** a warning is logged using spdlog about disconnected links  
**And** only the connected subtree is used for kinematics

#### Scenario: Detect joint limit violations
**Given** a joint with lower limit > upper limit  
**When** the system parses the URDF  
**Then** an exception is thrown with a descriptive error  
**And** the error message identifies the problematic joint

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

