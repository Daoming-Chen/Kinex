# benchmark-infrastructure Specification

## Purpose
Consolidate and standardize benchmark scenarios and metrics across C++ and Python implementations so results are comparable and reproducible. This document defines the canonical benchmark infrastructure, dataset generation, scenario semantics, metric naming/units, multi-robot support, and expected behavior for both native and binding-based benchmarks.
## Requirements
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

### Requirement: Benchmark Dataset Generation

The benchmark infrastructure SHALL provide tools to generate ground-truth datasets for IK evaluation.

#### Scenario: Generate reachable target dataset

- **WHEN** user runs dataset generation script with a robot URDF
- **THEN** N random joint configurations are sampled within joint limits
- **AND** forward kinematics is computed for each configuration (ground truth pose)
- **AND** dataset is saved as `{robot_name}_reachable_{N}samples.npz`

#### Scenario: FK oracle validation

- **WHEN** a dataset is generated using kinex Python bindings as FK oracle
- **THEN** FK results match C++ implementation (position error < 1e-6 m, rotation error < 1e-6 rad)

#### Scenario: Generate initial guess variations

- **WHEN** a target pose is included in the dataset
- **THEN** multiple initial guesses are generated:
  - Cold start: $q_{init} = 0$ or random far from $q_{gt}$
  - Warm start: $q_{init} = q_{gt} + \mathcal{N}(0, \sigma^2)$ with $\sigma = 0.1$
  - Trajectory start: $q_{init}$ from previous waypoint

#### Scenario: Generate unreachable target dataset (optional)

- **WHEN** user requests unreachable targets
- **THEN** poses are sampled outside the workspace (e.g., beyond max reach)
- **AND** dataset is labeled as `unreachable` for testing "closest solution" behavior

### Requirement: Variable-DOF Benchmark Support

The C++ benchmark suite SHALL support robots with arbitrary degrees of freedom (6 to 100+).

#### Scenario: Load robot model dynamically

- **WHEN** a benchmark is run with a generated URDF file path
- **THEN** the robot is parsed and FK/IK solvers are instantiated
- **AND** DOF is detected automatically from the robot model

#### Scenario: Parametric benchmarks for DOF sweep

- **WHEN** user runs `BM_IK_MixedChain`
- **THEN** benchmarks are executed for DOF values {10, 20, 50, 100}
- **AND** results are aggregated by DOF

#### Scenario: Dataset loading from file

- **WHEN** a benchmark fixture loads a `.npz` dataset
- **THEN** target poses, initial guesses, and ground truth are extracted
- **AND** C++ Eigen matrices are populated from NumPy arrays (via intermediate JSON or binary format)

### Requirement: Performance Metrics Collection

The benchmark infrastructure SHALL collect comprehensive metrics for IK solver evaluation and SHALL ensure consistent naming and units across implementations (C++ and Python).

#### Scenario: Success rate measurement

- **WHEN** IK is solved for N targets
- **THEN** success rate is computed as: $\frac{\text{converged solutions}}{\text{total attempts}} \times 100\%$
- **AND** convergence is defined as: $\|FK(q_{sol}) - T_{target}\|_{pos} < 5 \times 10^{-4}$ m AND $\|FK(q_{sol}) - T_{target}\|_{rot} < 1 \times 10^{-3}$ rad

#### Scenario: Iteration count statistics

- **WHEN** IK solver is benchmarked
- **THEN** average, median, min, max iterations per solve are recorded
- **AND** iteration count is tracked separately for converged and failed attempts


#### Scenario: Execution time measurement

- **WHEN** IK solve is executed
- **THEN** wall-clock time is measured in microseconds
- **AND** time excludes dataset loading and result validation
- **AND** metric naming is unified: use `real_time` for native C++ Google Benchmark output and `avg_time_us` for Python JSON output; both are expressed in microseconds (us) and must be treated equivalently by post-processing/visualization tools

#### Scenario: Position and rotation error tracking

- **WHEN** IK solution is obtained
- **THEN** position error (mm) is computed as: $1000 \times \|p_{achieved} - p_{target}\|_2$
- **AND** rotation error (deg) is computed as: $\frac{180}{\pi} \times \text{angularDistance}(q_{achieved}, q_{target})$

#### Scenario: Joint type sensitivity analysis (REMOVED)

Joint type sensitivity analysis has been removed from the canonical scope of this benchmark spec. Mixed-chain / Tier-B Python benchmarks may continue to include joint-type analysis as a supplementary capability, but it is not part of the core alignment effort.

### Requirement: Cold Start vs Warm Start Comparison

The benchmark infrastructure SHALL evaluate the impact of initial guess quality on solver performance and SHALL provide three explicitly-named scenarios with consistent semantics across C++ and Python:

1. ColdStart_Zero (C++) / cold_start_zero (Python)
2. ColdStart_Random (C++) / cold_start_random (Python)
3. Trajectory (C++) / trajectory (Python)

#### Scenario: Cold start from zero

- **WHEN** user runs `BM_IK_ColdStart_Zero` (C++) or `cold_start_zero` (Python)
- **THEN** all IK solves use $q_{init} = 0$ (all joints at zero)
- **AND** warm start is disabled in solver configuration

#### Scenario: Cold start from random

- **WHEN** user runs `BM_IK_ColdStart_Random` (C++) or `cold_start_random` (Python)
- **THEN** all IK solves use random initial guesses sampled uniformly within joint limits
- **AND** random guesses are independent of target configurations
- **AND** warm start is disabled in solver configuration

#### Scenario: Warm start benchmark

- **WHEN** user runs `BM_IK_WarmStart` (C++) or `warm_start` scenario (Python)
- **THEN** IK solves reuse previous solution as initial guess
- **AND** first solve is primed outside measured region
@@
### Requirement: Trajectory Tracking Benchmark

The benchmark infrastructure SHALL evaluate IK solver performance on continuous trajectories and SHALL ensure the trajectory semantics match across languages (C++ and Python).

#### Scenario: Generate trajectory dataset

- **WHEN** a trajectory dataset is created
- **THEN** 25 waypoints are sampled along a smooth path in joint space
- **AND** waypoints are separated by small increments (max 0.08 rad per joint)
- **AND** waypoints stay within joint limits (clamped if necessary)
- **AND** same trajectory generation logic is used in C++ and Python

#### Scenario: Trajectory following benchmark

- **WHEN** user runs `BM_IK_Trajectory` (C++) or `trajectory` scenario (Python)
- **THEN** IK is solved sequentially for each waypoint
- **AND** first waypoint uses zero initialization (cold start)
- **AND** subsequent waypoints use previous solution as initial guess (warm start)
- **AND** metrics track cumulative error and convergence failure rate

### Requirement: Multi-Robot Benchmark Support

The benchmark infrastructure SHALL support running benchmarks across multiple robot configurations in a single test run and SHALL standardize naming conventions for results.

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

The benchmark infrastructure SHALL ensure C++ and Python implementations produce comparable results for validation and downstream visualization.

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

#### Scenario: Initial guess quality impact

- **WHEN** benchmarks compare cold vs warm start
- **THEN** report shows:
  - Iteration count reduction (%)
  - Execution time speedup (×)
  - Success rate difference (%)

### Requirement: Trajectory Tracking Benchmark

The benchmark infrastructure SHALL evaluate IK solver performance on continuous trajectories.

#### Scenario: Generate trajectory dataset

- **WHEN** a trajectory dataset is created
- **THEN** N waypoints are sampled along a smooth path in joint space
- **AND** waypoints are separated by small increments (e.g., 0.08 rad per joint)

#### Scenario: Trajectory following benchmark

- **WHEN** user runs `BM_IK_Trajectory`
- **THEN** IK is solved sequentially for each waypoint
- **AND** previous solution is used as initial guess for next waypoint (warm start)
- **AND** metrics track cumulative error and convergence failure rate

### Requirement: Report Generation and Visualization

The benchmark infrastructure SHALL provide automated report generation with visualizations **from a centralized location**.

#### Scenario: Visualization script location

- **WHEN** user wants to generate benchmark visualizations
- **THEN** visualization scripts are located in `benchmarks/tools/`
- **AND** scripts can process results from both C++ and Python benchmarks
- **AND** single command generates all charts: `python -m benchmarks.tools.visualize`

#### Scenario: Markdown report generation

- **WHEN** benchmarks are completed
- **THEN** a Markdown report is generated **in `benchmarks/results/`** with:
  - Summary table (DOF vs success rate vs time)
  - Detailed metrics per benchmark
  - Timestamp and system information

#### Scenario: Interactive HTML visualization

- **WHEN** user runs report generation tool **from `benchmarks/tools/`**
- **THEN** an HTML file with Plotly/Matplotlib charts is created in `benchmarks/results/`
- **AND** charts include:
  - Success rate vs DOF (line plot)
  - Iteration count distribution (histogram)
  - Execution time scaling (log-log plot)

### Requirement: Extensibility for Custom Robots

The benchmark infrastructure SHALL allow users to benchmark custom URDF files **using shared tools**.

#### Scenario: Benchmark custom URDF

- **WHEN** user provides a custom URDF file path
- **THEN** the system generates dataset using **`benchmarks.tools.MixedChainGenerator`**
- **AND** all benchmarks are executed without code modification

#### Scenario: Command-line interface for benchmark execution

- **WHEN** user runs `python -m benchmarks.python.run_benchmarks --urdf my_robot.urdf --samples 500`
- **THEN** dataset is generated **using tools from `benchmarks/tools/`**
- **AND** benchmarks are executed
- **AND** report is saved to `benchmarks/results/`
- **AND** no manual C++ compilation is required for standard benchmarks

### Requirement: Documentation and Examples

The benchmark infrastructure SHALL provide comprehensive documentation and example workflows.

#### Scenario: Quick start guide

- **WHEN** user reads `docs/benchmarks/mixkinbench.md`
- **THEN** they can generate a dataset and run benchmarks within 5 minutes
- **AND** examples include both standard (UR5) and synthetic robots

#### Scenario: Python API reference

- **WHEN** user views Python module docstrings
- **THEN** all classes (`MixedChainGenerator`, `BenchmarkDataset`) have:
  - Class-level documentation
  - Parameter descriptions for all public methods
  - Usage examples

#### Scenario: Example benchmark workflow

- **WHEN** user runs provided example script `examples/python/run_mixkinbench.py`
- **THEN** a complete workflow executes:
  1. Generate a 20-DOF mixed robot
  2. Create dataset with 100 targets
  3. Run C++ benchmarks
  4. Generate HTML report

### Requirement: Benchmark Directory Organization

The benchmark infrastructure SHALL be organized into distinct subdirectories with clear responsibilities.

#### Scenario: C++ benchmarks location

- **WHEN** developer looks for native C++ performance benchmarks
- **THEN** all C++ benchmark executables are located in `benchmarks/cpp/`
- **AND** includes `ik_benchmarks.cpp`, `jacobian_benchmarks.cpp`, `mixed_ik_benchmarks.cpp`

#### Scenario: Python benchmarks location

- **WHEN** developer looks for Python binding performance benchmarks
- **THEN** all Python benchmark runners are located in `benchmarks/python/`
- **AND** includes tier A/B runners and master orchestration scripts

#### Scenario: Shared tools location

- **WHEN** developer needs benchmark utilities (generators, oracles, visualizers)
- **THEN** shared tools are located in `benchmarks/tools/`
- **AND** includes `urdf_generator.py`, `oracle.py`, visualization scripts
- **AND** tools are importable as Python package: `from benchmarks.tools import MixedChainGenerator`

#### Scenario: Results consolidation

- **WHEN** benchmarks generate output files
- **THEN** all results are stored in `benchmarks/results/`
- **AND** includes JSON outputs, visualizations, and summary reports

### Requirement: Binding Overhead Measurement

The Python bindings benchmark suite SHALL focus on measuring binding layer overhead rather than core algorithm performance.

#### Scenario: FK computation overhead

- **WHEN** binding overhead benchmark runs FK tests
- **THEN** it measures time for same FK computation in Python vs C++
- **AND** reports overhead percentage: `(t_python - t_cpp) / t_cpp * 100`
- **AND** overhead is documented and compared against <10% target

#### Scenario: Data conversion overhead

- **WHEN** binding overhead benchmark tests data conversion
- **THEN** it measures NumPy array to Eigen::VectorXd conversion time
- **AND** it measures Transform object creation from Python types
- **AND** reports conversion overhead in microseconds per operation

#### Scenario: IK solver binding overhead

- **WHEN** binding overhead benchmark runs IK tests
- **THEN** it compares identical IK solve (same target, initial guess) in Python vs C++
- **AND** it isolates binding overhead from algorithm performance
- **AND** reports overhead breakdown (setup, computation, result conversion)

### Requirement: Benchmark Architecture Documentation

The benchmark infrastructure SHALL provide clear documentation of organization and decision criteria.

#### Scenario: Architecture overview

- **WHEN** developer reads `docs/benchmarks/architecture.md`
- **THEN** document explains the three-tier organization (cpp/python/tools)
- **AND** provides directory structure diagram
- **AND** explains purpose of each subdirectory

#### Scenario: Benchmark placement decision tree

- **WHEN** developer wants to add a new benchmark
- **THEN** documentation provides decision criteria:
  - "Measuring core algorithm performance?" �?`benchmarks/cpp/` or `benchmarks/python/`
  - "Measuring binding overhead?" �?`bindings/python/benchmarks/`
  - "Creating test infrastructure?" �?`benchmarks/tools/`
- **AND** includes examples for each category

#### Scenario: Import pattern documentation

- **WHEN** developer needs to use benchmark tools in scripts
- **THEN** documentation shows correct import patterns:
  - `from benchmarks.tools import MixedChainGenerator, FKOracle`
  - How to add `benchmarks/` to Python path if needed
- **AND** provides troubleshooting for common import errors

### Requirement: Shared Tool Package

The benchmark tools SHALL be organized as a Python package accessible to all benchmark implementations.

#### Scenario: URDF generator import

- **WHEN** benchmark script needs to generate synthetic robots
- **THEN** it imports: `from benchmarks.tools import MixedChainGenerator`
- **AND** generator creates URDF files in `benchmarks/tools/` module location

#### Scenario: Oracle utilities import

- **WHEN** benchmark script needs FK oracle or joint sampler
- **THEN** it imports: `from benchmarks.tools import FKOracle, JointSampler`
- **AND** utilities work with kinex Robot objects

#### Scenario: Visualization import

- **WHEN** benchmark results need visualization
- **THEN** script imports visualization functions from `benchmarks.tools.visualize`
- **AND** functions accept standard result JSON formats

