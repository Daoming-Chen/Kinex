# python-bindings Specification

## Purpose
TBD - created by archiving change add-python-bindings. Update Purpose after archive.
## Requirements
### Requirement: URDF Parser Binding

The Python bindings SHALL expose URDF parsing functionality through a `Robot` class with factory methods.

#### Scenario: Parse URDF from file

- **WHEN** user calls `Robot.from_urdf("path/to/robot.urdf")`
- **THEN** a Robot instance is returned with parsed robot model
- **AND** all links, joints, and properties are accessible

#### Scenario: Parse URDF from string

- **WHEN** user calls `Robot.from_urdf_string(urdf_xml_string)`
- **THEN** a Robot instance is returned with parsed robot model

#### Scenario: Invalid URDF file

- **WHEN** user provides non-existent file path
- **THEN** a `FileNotFoundError` is raised

#### Scenario: Malformed URDF XML

- **WHEN** user provides invalid URDF XML
- **THEN** a `RuntimeError` is raised with descriptive error message

### Requirement: Robot Model Access

The Python bindings SHALL provide access to robot model properties including links, joints, and metadata.

#### Scenario: Query robot name

- **WHEN** user accesses `robot.name`
- **THEN** the robot name from URDF is returned

#### Scenario: Get joint names

- **WHEN** user calls `robot.get_joint_names()`
- **THEN** a list of all actuated joint names is returned in order

#### Scenario: Get degrees of freedom

- **WHEN** user accesses `robot.dof`
- **THEN** the number of actuated joints is returned

#### Scenario: Get joint limits

- **WHEN** user calls `robot.get_joint_limits()`
- **THEN** a tuple of (lower_limits, upper_limits) NumPy arrays is returned

#### Scenario: Access link by name

- **WHEN** user calls `robot.get_link("link_name")`
- **THEN** a Link object is returned if exists
- **AND** raises `KeyError` if link does not exist

### Requirement: Transform Representation

The Python bindings SHALL provide a `Transform` class for representing 3D transformations.

#### Scenario: Create transform from position and quaternion

- **WHEN** user calls `Transform.from_position_quaternion(pos, quat)`
- **THEN** a Transform instance is created
- **AND** `pos` is a 3-element array-like (x, y, z)
- **AND** `quat` is a 4-element array-like (w, x, y, z)

#### Scenario: Extract transformation matrix

- **WHEN** user accesses `transform.matrix()`
- **THEN** a 4x4 NumPy array representing the homogeneous transformation is returned

#### Scenario: Extract position and quaternion

- **WHEN** user accesses `transform.position` and `transform.quaternion`
- **THEN** position is returned as NumPy array [x, y, z]
- **AND** quaternion is returned as NumPy array [w, x, y, z]

### Requirement: Forward Kinematics Computation

The Python bindings SHALL provide forward kinematics computation through a `ForwardKinematics` class.

#### Scenario: Create FK solver

- **WHEN** user instantiates `ForwardKinematics(robot, end_link)`
- **THEN** a FK solver is created for the kinematic chain to `end_link`

#### Scenario: Compute end-effector pose

- **WHEN** user calls `fk.compute(joint_angles)`
- **THEN** a Transform representing end-effector pose is returned
- **AND** `joint_angles` is a NumPy array matching DOF

#### Scenario: Invalid joint angles size

- **WHEN** user provides joint_angles with incorrect size
- **THEN** a `ValueError` is raised with descriptive message

#### Scenario: Joint bounds checking

- **WHEN** user calls `fk.compute(joint_angles, check_bounds=True)`
- **AND** any joint is out of limits
- **THEN** a `RuntimeError` is raised indicating which joint is out of bounds

### Requirement: Jacobian Computation

The Python bindings SHALL provide Jacobian matrix computation through a `JacobianCalculator` class.

#### Scenario: Create Jacobian calculator

- **WHEN** user instantiates `JacobianCalculator(robot, end_link)`
- **THEN** a Jacobian calculator is created

#### Scenario: Compute Jacobian matrix

- **WHEN** user calls `jacobian.compute(joint_angles)`
- **THEN** a 6×n NumPy array is returned (n = number of joints)
- **AND** first 3 rows represent linear velocity Jacobian
- **AND** last 3 rows represent angular velocity Jacobian

#### Scenario: Check singularity

- **WHEN** user calls `jacobian.is_singular(joint_angles)`
- **THEN** a boolean is returned indicating if configuration is singular

#### Scenario: Compute manipulability

- **WHEN** user calls `jacobian.get_manipulability(joint_angles)`
- **THEN** a scalar manipulability measure is returned

### Requirement: Inverse Kinematics Solving

The Python bindings SHALL provide IK solving through a `SQPIKSolver` class.

#### Scenario: Create IK solver

- **WHEN** user instantiates `SQPIKSolver(robot, end_link)`
- **THEN** an IK solver is created

#### Scenario: Solve IK problem

- **WHEN** user calls `result = ik_solver.solve(target_pose, initial_guess)`
- **THEN** an `IKResult` object is returned
- **AND** `result.converged` indicates success
- **AND** `result.solution` contains joint angles as NumPy array
- **AND** `result.iterations` contains iteration count

#### Scenario: Configure solver parameters

- **WHEN** user sets `ik_solver.tolerance = 1e-5`
- **THEN** solver uses updated tolerance in subsequent solves

#### Scenario: Position-only IK

- **WHEN** user calls `ik_solver.set_position_only(True)`
- **AND** calls `ik_solver.solve(target_pose, initial_guess)`
- **THEN** solver only matches position, ignoring orientation

### Requirement: NumPy Integration

The Python bindings SHALL seamlessly integrate with NumPy for array inputs and outputs.

#### Scenario: Accept NumPy arrays

- **WHEN** user passes NumPy arrays as joint angles
- **THEN** they are accepted without requiring conversion

#### Scenario: Accept Python lists

- **WHEN** user passes Python lists as joint angles
- **THEN** they are automatically converted to appropriate format

#### Scenario: Return NumPy arrays

- **WHEN** any function returns array data
- **THEN** it returns NumPy arrays with appropriate dtype

### Requirement: Type Hints

The Python bindings SHALL provide complete type hints for all public APIs.

#### Scenario: IDE autocompletion

- **WHEN** user types code in IDE with type checking enabled
- **THEN** IDE provides accurate autocompletion and type checking

#### Scenario: mypy validation

- **WHEN** user runs mypy on code using kinex
- **THEN** mypy validates types correctly without errors

### Requirement: Documentation Strings

The Python bindings SHALL provide comprehensive docstrings for all classes and methods.

#### Scenario: Access help

- **WHEN** user calls `help(kinex.ForwardKinematics)`
- **THEN** detailed documentation is displayed

#### Scenario: Parameter documentation

- **WHEN** user calls `help(fk.compute)`
- **THEN** all parameters and return values are documented

### Requirement: Error Messages

The Python bindings SHALL provide clear, actionable error messages.

#### Scenario: Descriptive exceptions

- **WHEN** an error occurs
- **THEN** the exception message clearly describes the problem
- **AND** includes relevant context (e.g., which joint, expected vs actual)

### Requirement: Build System Integration

The Python bindings SHALL integrate with standard Python build tools.

#### Scenario: Install from source

- **WHEN** user runs `pip install .` in bindings/python directory
- **THEN** the kinex package is built and installed

#### Scenario: Development mode installation

- **WHEN** user runs `pip install -e .`
- **THEN** the package is installed in editable mode

#### Scenario: CMake build integration

- **WHEN** CMake is configured with Python bindings enabled
- **THEN** the Python module is built as part of the main build

### Requirement: Test Coverage

The Python bindings SHALL have comprehensive test coverage using pytest.

#### Scenario: Unit tests for all classes

- **WHEN** tests are run with `pytest bindings/python/tests/`
- **THEN** all core classes have unit tests
- **AND** test coverage is > 90%

#### Scenario: Integration tests

- **WHEN** integration tests are run
- **THEN** full FK→IK roundtrip tests pass
- **AND** results match C++ implementation within numerical tolerance

#### Scenario: Numerical accuracy tests

- **WHEN** Python results are compared to C++ results
- **THEN** differences are < 1e-10 for same inputs

### Requirement: Example Code

The Python bindings SHALL include example code demonstrating common use cases.

#### Scenario: Basic FK example

- **WHEN** user reads examples/python/forward_kinematics.py
- **THEN** a complete FK example is provided

#### Scenario: IK example

- **WHEN** user reads examples/python/inverse_kinematics.py
- **THEN** a complete IK example with visualization hints is provided

#### Scenario: Trajectory generation example

- **WHEN** user reads examples/python/trajectory.py
- **THEN** an example showing multiple IK solves with warm-starting is provided

### Requirement: Binding Overhead Benchmarking

The Python bindings SHALL include benchmarks that measure the performance overhead introduced by the binding layer.

#### Scenario: Overhead test suite location

- **WHEN** developer looks for Python binding overhead tests
- **THEN** tests are located in `bindings/python/benchmarks/`
- **AND** suite includes: `test_binding_overhead.py`
- **AND** suite is separate from core performance benchmarks in `benchmarks/`

#### Scenario: FK computation overhead measurement

- **WHEN** overhead benchmark runs FK performance test
- **THEN** it executes identical FK computation in both C++ and Python
- **AND** measures wall-clock time for each (averaged over multiple runs)
- **AND** computes overhead percentage: `(t_python - t_cpp) / t_cpp * 100`
- **AND** verifies overhead is within acceptable threshold (<10%)

#### Scenario: IK solver overhead measurement

- **WHEN** overhead benchmark runs IK performance test
- **THEN** it executes identical IK solve (same target pose, same initial guess, same solver config)
- **AND** measures total solve time in C++ and Python
- **AND** reports overhead breakdown:
  - Setup overhead (Robot/Solver instantiation)
  - Computation overhead (per iteration)
  - Result conversion overhead (Transform �?NumPy)

#### Scenario: Jacobian computation overhead measurement

- **WHEN** overhead benchmark runs Jacobian performance test
- **THEN** it computes Jacobian for same joint configuration in C++ and Python
- **AND** measures matrix computation and conversion time
- **AND** reports overhead for matrix-to-NumPy conversion separately

#### Scenario: Data conversion overhead measurement

- **WHEN** overhead benchmark tests data conversion
- **THEN** it measures time to convert:
  - NumPy array �?Eigen::VectorXd (joint angles)
  - Transform object �?NumPy position/quaternion
  - Eigen::MatrixXd �?NumPy array (Jacobian)
- **AND** reports per-conversion overhead in microseconds

#### Scenario: Overhead report generation

- **WHEN** binding overhead benchmarks complete
- **THEN** a summary report is generated showing:
  - Operation: FK, IK, Jacobian
  - C++ time: baseline timing
  - Python time: binding + baseline
  - Overhead: percentage and absolute
  - Status: PASS/FAIL based on threshold
- **AND** report is saved as `bindings/python/benchmarks/results/overhead_report.txt`

### Requirement: System SHALL expose C++ API to Python via nanobind
The system SHALL provide Python bindings using nanobind for all core functionality.

#### Scenario: Import kinex module in Python
**Given** the kinex library is installed  
**When** the user runs `import kinex` in Python  
**Then** the module loads without errors  
**And** all core classes are accessible (Robot, ForwardKinematics, JacobianCalculator, IKSolver)

#### Scenario: Load robot from URDF in Python
**Given** a URDF file path "ur5_urdf/ur5e.urdf"  
**When** the user calls `robot = kinex.Robot.from_urdf("ur5_urdf/ur5e.urdf")`  
**Then** a Robot object is returned  
**And** the robot can be queried for links and joints

### Requirement: System SHALL support NumPy array interoperability
The system SHALL accept and return NumPy arrays for joint angles, poses, and matrices.

#### Scenario: Compute FK with NumPy array input
**Given** a Robot and ForwardKinematics object  
**And** a NumPy array q = np.array([0, -1.57, 0, 0, 0, 0])  
**When** the user calls `pose = fk.compute(q)`  
**Then** the function accepts the NumPy array  
**And** returns a 4×4 NumPy array representing the pose matrix

#### Scenario: Compute Jacobian returns NumPy array
**Given** a JacobianCalculator and joint angles as NumPy array  
**When** the user calls `J = jacobian_calc.compute(q)`  
**Then** the system returns a 6×6 NumPy array  
**And** the array is writable and can be modified

#### Scenario: Zero-copy NumPy integration
**Given** a large NumPy array of joint angles  
**When** passed to C++ functions via nanobind  
**Then** no data copying occurs (zero-copy)  
**And** the C++ code operates directly on NumPy buffer

### Requirement: System SHALL provide Pythonic API with snake_case naming
The system SHALL follow Python naming conventions for better integration.

#### Scenario: Python API uses snake_case
**Given** the C++ API has CamelCase methods  
**When** accessed from Python  
**Then** methods are exposed with snake_case names  
**And** `forwardKinematics.compute()` becomes `forward_kinematics.compute()`

### Requirement: System SHALL provide type stubs for IDE support
The system SHALL include .pyi type stub files for static type checking.

#### Scenario: Type checking with mypy
**Given** Python code using kinex with type hints  
**When** the user runs `mypy script.py`  
**Then** mypy recognizes kinex types  
**And** type errors are caught at check time  
**And** IDE autocomplete works correctly

### Requirement: System SHALL handle exceptions across language boundary
The system SHALL translate C++ exceptions to Python exceptions.

#### Scenario: URDF parse error raises Python exception
**Given** an invalid URDF file  
**When** Python code calls `Robot.from_urdf("invalid.urdf")`  
**Then** a Python `URDFParseError` exception is raised  
**And** the exception message contains the C++ error details  
**And** the Python traceback is preserved

### Requirement: System SHALL support Python context managers
The system SHALL support Python's context manager protocol for resource management.

#### Scenario: Use Robot as context manager
**Given** a Robot object  
**When** the user uses `with kinex.Robot.from_urdf(...) as robot:`  
**Then** the robot is properly initialized  
**And** resources are cleaned up on context exit

### Requirement: System SHALL provide installable package via pip
The system SHALL be installable as a Python package.

#### Scenario: Install via pip
**Given** the kinex project with setup.py  
**When** the user runs `pip install .`  
**Then** the package is installed to site-packages  
**And** `import kinex` works from any directory

#### Scenario: Install wheel distribution
**Given** a built wheel file kinex-1.0.0-cp38-cp38-linux_x86_64.whl  
**When** the user runs `pip install kinex-1.0.0-*.whl`  
**Then** the binary package installs without compilation  
**And** all dependencies are satisfied

### Requirement: System SHALL support multiple Python versions
The system SHALL support Python 3.8 through 3.12.

#### Scenario: Test on Python 3.8
**Given** Python 3.8 environment  
**When** kinex is installed and tested  
**Then** all tests pass  
**And** NumPy 1.20+ compatibility is verified

#### Scenario: Test on Python 3.12
**Given** Python 3.12 environment  
**When** kinex is installed and tested  
**Then** all tests pass  
**And** nanobind is compatible with Python 3.12

### Requirement: System SHALL efficient memory management
The system SHALL minimize memory overhead in Python bindings.

#### Scenario: Robot object memory footprint
**Given** a Robot loaded in Python  
**When** memory usage is measured  
**Then** the Python object overhead is < 1KB  
**And** the C++ object is referenced (not copied)

#### Scenario: No memory leaks across calls
**Given** 10,000 FK computations in Python  
**When** monitored with memory profiler  
**Then** memory usage remains constant  
**And** no memory leaks are detected

### Requirement: System SHALL provide comprehensive documentation
The system SHALL provide documentation for Python API.

#### Scenario: Access docstrings
**Given** any kinex Python class or function  
**When** the user calls `help(kinex.Robot)`  
**Then** detailed docstrings are displayed  
**And** parameter types and return types are documented  
**And** usage examples are included

