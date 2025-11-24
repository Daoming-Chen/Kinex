# benchmark-infrastructure Spec Delta

## MODIFIED Requirements

### Requirement: Cold Start vs Warm Start Comparison

The benchmark infrastructure SHALL evaluate the impact of initial guess quality on solver performance **with two distinct cold start strategies**.

#### Scenario: Cold start from zero initialization

- **WHEN** user runs `BM_IK_ColdStart_Zero` (C++) or cold_start_zero scenario (Python)
- **THEN** all IK solves use $q_{init} = 0$ (all joints at zero)
- **AND** warm start is disabled in solver configuration
- **AND** benchmark tests solver from worst-case initialization

#### Scenario: Cold start from random initialization

- **WHEN** user runs `BM_IK_ColdStart_Random` (C++) or cold_start_random scenario (Python)
- **THEN** all IK solves use random initial guesses sampled uniformly within joint limits
- **AND** random guesses are independent of target configurations
- **AND** warm start is disabled in solver configuration
- **AND** benchmark tests solver robustness to arbitrary initialization

#### Scenario: Warm start benchmark

- **WHEN** user runs `BM_IK_WarmStart` (C++) or warm_start scenario (Python)
- **THEN** IK solves reuse previous solution as initial guess
- **AND** first solve is primed outside measured region
- **AND** subsequent solves use previous solution without explicit initial guess

#### Scenario: Initial guess quality impact

- **WHEN** benchmarks compare cold vs warm start
- **THEN** report shows:
  - Iteration count reduction (%)
  - Execution time speedup (×)
  - Success rate difference (%)

### Requirement: Trajectory Tracking Benchmark

The benchmark infrastructure SHALL evaluate IK solver performance on continuous trajectories **with consistent implementation across languages**.

#### Scenario: Generate trajectory dataset

- **WHEN** a trajectory dataset is created
- **THEN** 25 waypoints are sampled along a smooth path in joint space
- **AND** waypoints are separated by small increments (max 0.08 rad per joint)
- **AND** waypoints stay within joint limits (clamped if necessary)
- **AND** same trajectory generation logic is used in C++ and Python

#### Scenario: Trajectory following benchmark

- **WHEN** user runs `BM_IK_Trajectory` (C++) or trajectory scenario (Python)
- **THEN** IK is solved sequentially for each waypoint
- **AND** first waypoint uses zero initialization (cold start)
- **AND** subsequent waypoints use previous solution as initial guess (warm start)
- **AND** metrics track cumulative error and convergence failure rate

### Requirement: Performance Metrics Collection

The benchmark infrastructure SHALL collect comprehensive metrics for IK solver evaluation **with consistent naming and units across implementations**.

#### Scenario: Success rate measurement

- **WHEN** IK is solved for N targets
- **THEN** success rate is computed as: $\frac{\text{converged solutions}}{\text{total attempts}} \times 100\%$
- **AND** convergence is defined as: $\|FK(q_{sol}) - T_{target}\|_{pos} < 5 \times 10^{-4}$ m AND $\|FK(q_{sol}) - T_{target}\|_{rot} < 1 \times 10^{-3}$ rad
- **AND** metric is named `success_rate` with unit `%` in both C++ and Python

#### Scenario: Iteration count statistics

- **WHEN** IK solver is benchmarked
- **THEN** average iterations per solve is recorded
- **AND** iterations are counted only for converged solutions
- **AND** metric is named `iterations_per_solve` (C++) or `avg_iterations` (Python)

#### Scenario: Execution time measurement

- **WHEN** IK solve is executed
- **THEN** wall-clock time is measured in microseconds
- **AND** time excludes dataset loading and result validation
- **AND** metric is named `real_time` (C++) or `avg_time_us` (Python) with unit `us`

#### Scenario: Position and rotation error tracking

- **WHEN** IK solution is obtained
- **THEN** position error (mm) is computed as: $1000 \times \|p_{achieved} - p_{target}\|_2$
- **AND** rotation error (deg) is computed as: $\frac{180}{\pi} \times \text{angularDistance}(q_{achieved}, q_{target})$
- **AND** position metric is named `avg_position_error_mm` in both C++ and Python
- **AND** rotation metric is named `avg_rotation_error_deg` in both C++ and Python

## ADDED Requirements

### Requirement: Multi-Robot Benchmark Support

The benchmark infrastructure SHALL support benchmarking multiple robot configurations in a single test run.

#### Scenario: Test UR5e robot variants

- **WHEN** benchmarks are executed
- **THEN** tests run on all UR5e variants: ur5e (6 DOF), ur5e+x (7 DOF), ur5e+xy (8 DOF), ur5e+xyz (9 DOF)
- **AND** each robot/scenario combination produces separate results
- **AND** results are aggregated in unified output format

#### Scenario: C++ parameterized benchmarks

- **WHEN** C++ benchmarks are built
- **THEN** Google Benchmark parameterized benchmarks are used
- **AND** each robot/scenario combination is a separate benchmark instance
- **AND** benchmark names follow pattern: `BM_IK_{Scenario}/{RobotName}`

#### Scenario: Python robot iteration

- **WHEN** Python benchmarks are run
- **THEN** Tier A runner iterates over robot configurations
- **AND** each robot produces a separate JSON output file
- **AND** JSON files follow naming: `{robot_name}_results.json`

### Requirement: Cross-Language Benchmark Consistency

The benchmark infrastructure SHALL ensure C++ and Python implementations produce comparable results for validation.

#### Scenario: Identical test parameters

- **WHEN** benchmarks are run in both C++ and Python
- **THEN** both use random seed 42 for reproducibility
- **AND** both use 1000 test samples per robot/scenario
- **AND** both use same solver configuration (tolerance, max_iterations, etc.)

#### Scenario: Scenario naming alignment

- **WHEN** benchmark results are compared
- **THEN** C++ scenario names are: `ColdStart_Zero`, `ColdStart_Random`, `Trajectory`
- **AND** Python scenario keys are: `cold_start_zero`, `cold_start_random`, `trajectory`
- **AND** mapping is documented for cross-language comparison

#### Scenario: Metric units consistency

- **WHEN** metrics are reported
- **THEN** time is always in microseconds (µs)
- **AND** position error is always in millimeters (mm)
- **AND** rotation error is always in degrees (deg)
- **AND** success rate is always in percentage (%)
- **AND** iteration count is dimensionless

#### Scenario: Expected differences documentation

- **WHEN** comparing C++ and Python results
- **THEN** documentation explains expected binding overhead (~5-15% time increase)
- **AND** success rates should match within ±5%
- **AND** iteration counts should be identical for converged cases
- **AND** errors should match within floating point precision (±1e-6)

### Requirement: Benchmark Scenario Consistency

The benchmark infrastructure SHALL implement identical test scenarios in both C++ and Python.

#### Scenario: Three standard scenarios

- **WHEN** benchmarks are executed
- **THEN** both C++ and Python implement exactly three scenarios:
  1. Cold start from zero
  2. Cold start from random
  3. Trajectory tracking
- **AND** scenario logic is equivalent across languages

#### Scenario: Scenario execution order

- **WHEN** benchmarks run multiple scenarios
- **THEN** execution order is: ColdStart_Zero, ColdStart_Random, Trajectory
- **AND** each scenario uses independent test samples
- **AND** scenarios do not share state

## REMOVED Requirements

#### Scenario: Joint type sensitivity analysis

**Rationale**: This requirement is specific to mixed-chain robots (Tier B) and not applicable to the standard UR5e benchmarks being aligned. Tier B benchmarks in Python can retain this analysis, but it's not part of the core alignment.

The removed scenario was:
> #### Scenario: Joint type sensitivity analysis
> - **WHEN** benchmarking mixed-joint robots
> - **THEN** errors are grouped by joint type (revolute vs prismatic)
> - **AND** average error per joint type is reported
