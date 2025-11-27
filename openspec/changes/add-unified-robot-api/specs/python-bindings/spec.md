# python-bindings Specification Delta

## MODIFIED Requirements

### Requirement: Python bindings SHALL expose RobotModel class
The Python bindings SHALL provide a RobotModel class (previously named Robot) that wraps the C++ RobotModel with pythonic interface.

#### Scenario: Create RobotModel from URDF in Python
**GIVEN** a URDF file at "robots/ur5e.urdf"
**WHEN** the user executes `model = kinex.RobotModel.from_urdf_file("robots/ur5e.urdf")`
**THEN** a RobotModel instance is created
**AND** the model contains all links and joints
**AND** low-level APIs accept RobotModel instances

#### Scenario: Existing low-level API uses RobotModel
**GIVEN** a Python RobotModel instance model
**WHEN** the user creates `fk = kinex.ForwardKinematics(model, "tool0")`
**THEN** the ForwardKinematics object is created successfully
**AND** all existing low-level APIs continue to work with RobotModel

## ADDED Requirements

### Requirement: Python bindings SHALL expose unified Robot class
The Python bindings SHALL provide a pythonic Robot class that wraps the C++ unified Robot API with snake_case method names.

#### Scenario: Create Robot from URDF in Python
**GIVEN** a URDF file at "robots/ur5e.urdf"
**WHEN** the user executes `robot = kinex.Robot.from_urdf("robots/ur5e.urdf", "tool0")`
**THEN** a Robot instance is created
**AND** the robot is ready for kinematics operations
**AND** the default end-effector is "tool0"

#### Scenario: Create Robot from URDF string in Python
**GIVEN** a URDF XML string loaded in Python
**WHEN** the user calls `robot = kinex.Robot.from_urdf_string(urdf_content, "end_link")`
**THEN** a Robot instance is created from the string
**AND** no filesystem access is required

#### Scenario: Compute FK from Python
**GIVEN** a Python Robot instance
**AND** joint angles as NumPy array q = np.array([0, -π/2, 0, -π/2, 0, 0])
**WHEN** the user calls `pose = robot.forward_kinematics(q)`
**THEN** the system returns a Transform object
**AND** pose.as_matrix() returns a NumPy array of shape (4, 4)
**AND** pose.translation() returns a NumPy array of shape (3,)

#### Scenario: Compute FK with alternative method name
**GIVEN** a Python Robot instance
**WHEN** the user calls `pose = robot.compute_pose(q)`
**THEN** the result is identical to `robot.forward_kinematics(q)`
**AND** both snake_case method names are available

#### Scenario: Solve IK from Python
**GIVEN** a Python Robot instance
**AND** a target pose as Transform or 4×4 NumPy array
**AND** an initial guess q_init as NumPy array
**WHEN** the user calls `q_solution, status = robot.inverse_kinematics(target_pose, q_init)`
**THEN** q_solution is a NumPy array of joint angles
**AND** status is a dict containing {'converged': bool, 'iterations': int, 'error': float, ...}
**AND** the binding releases GIL during solve for parallelism

#### Scenario: Solve IK with alternative method name
**GIVEN** a Python Robot instance
**WHEN** the user calls `result = robot.solve_ik(target, q_init)`
**THEN** the result is identical to `robot.inverse_kinematics(target, q_init)`

#### Scenario: Compute Jacobian from Python
**GIVEN** a Python Robot instance
**AND** joint angles q as NumPy array
**WHEN** the user calls `J = robot.compute_jacobian(q)`
**THEN** the system returns a NumPy array of shape (6, n_joints)
**AND** the array dtype is np.float64
**AND** the Jacobian can be used for velocity/force mapping

#### Scenario: Get manipulability metrics from Python
**GIVEN** a Python Robot instance
**AND** joint angles q
**WHEN** the user calls `m = robot.get_manipulability(q)`
**THEN** m is a Python float representing manipulability
**WHEN** the user calls `is_sing = robot.is_singular(q)`
**THEN** is_sing is a Python bool
**WHEN** the user calls `cond = robot.get_condition_number(q)`
**THEN** cond is a Python float

#### Scenario: Configure IK solver from Python
**GIVEN** a Python Robot instance
**WHEN** the user calls `robot.set_ik_tolerance(1e-6)`
**THEN** subsequent IK calls use the new tolerance
**WHEN** the user calls `robot.set_position_only_ik(True)`
**THEN** IK ignores orientation constraints
**WHEN** the user calls `config = robot.get_solver_config()`
**THEN** config is a SolverConfig object accessible from Python

#### Scenario: Clone Robot in Python
**GIVEN** a Python Robot instance robot1
**WHEN** the user calls `robot2 = robot1.clone()`
**THEN** robot2 is an independent copy
**AND** modifications to robot1 do not affect robot2
**AND** both can be used from different Python threads
**AND** GIL is released during kinematics operations

#### Scenario: Robot properties are accessible from Python
**GIVEN** a Python Robot instance
**THEN** `robot.name` returns the robot name string
**AND** `robot.dof` returns the degrees of freedom (int)
**AND** `robot.end_link` returns the default end-effector link name
**AND** `robot.base_link` returns the base link name
**AND** `robot.joint_names` returns a list of actuated joint names

### Requirement: Python bindings SHALL maintain backward compatibility with low-level API
The Python bindings SHALL keep all existing low-level classes (ForwardKinematics, SQPIKSolver, JacobianCalculator) accessible alongside the new Robot class.

#### Scenario: Low-level API still works with RobotModel
**GIVEN** a RobotModel instance
**WHEN** the user creates low-level solvers with RobotModel
**THEN** all existing code continues to work
**AND** ForwardKinematics, SQPIKSolver, JacobianCalculator accept RobotModel
**AND** advanced users can still use low-level API for fine control
