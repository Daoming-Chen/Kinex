# benchmark-infrastructure Specification Delta

## MODIFIED Requirements

### Requirement: Benchmarks SHALL use RobotModel for robot structural models
All benchmark scripts SHALL use RobotModel (previously Robot) when loading robot models from URDF files.

#### Scenario: Tier A benchmarks use RobotModel
**GIVEN** the run_tier_a_benchmarks.py script
**WHEN** robots are loaded from URDF files
**THEN** the script uses `kinex.RobotModel` or the low-level API with RobotModel
**AND** benchmarks continue to measure performance accurately
**AND** results are comparable to previous benchmark runs

#### Scenario: Tier B benchmarks generate RobotModel instances
**GIVEN** the urdf_generator.py script
**WHEN** synthetic robots are generated
**THEN** the generated URDF strings are parsed into RobotModel instances
**AND** benchmark solvers accept RobotModel parameters
**AND** performance measurements remain valid

#### Scenario: Oracle comparisons use RobotModel
**GIVEN** the oracle.py script for IK validation
**WHEN** comparing kinex results against other libraries
**THEN** the oracle uses RobotModel for loading robots
**AND** FK/IK/Jacobian computations use the updated API
**AND** validation logic remains correct

## ADDED Requirements

### Requirement: Benchmarks SHALL support unified Robot API usage
Benchmark scripts SHALL be compatible with the new unified Robot class, allowing cleaner, more concise code in example benchmarks while maintaining the option to use low-level API for direct performance measurement.

#### Scenario: Example benchmark uses unified Robot API
**GIVEN** a benchmark script demonstrating library usage
**WHEN** the script is updated to use Robot class
**THEN** the code is more concise and readable
**AND** performance characteristics are identical to low-level API
**AND** the benchmark serves as usage example for end users

**Note**: Updating benchmarks to use Robot class is OPTIONAL. Benchmarks can continue using low-level API (ForwardKinematics, SQPIKSolver, etc.) with RobotModel for direct performance measurement.
