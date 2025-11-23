# benchmark-infrastructure Specification

## Purpose
TBD - created by archiving change add-mixed-chain-benchmark-suite. Update Purpose after archive.
## Requirements
### Requirement: Benchmark Dataset Generation

The benchmark infrastructure SHALL provide tools to generate ground-truth datasets for IK evaluation.

#### Scenario: Generate reachable target dataset

- **WHEN** user runs dataset generation script with a robot URDF
- **THEN** N random joint configurations are sampled within joint limits
- **AND** forward kinematics is computed for each configuration (ground truth pose)
- **AND** dataset is saved as `{robot_name}_reachable_{N}samples.npz`

#### Scenario: FK oracle validation

- **WHEN** a dataset is generated using urdfx Python bindings as FK oracle
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

The benchmark infrastructure SHALL collect comprehensive metrics for IK solver evaluation.

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

#### Scenario: Position and rotation error tracking

- **WHEN** IK solution is obtained
- **THEN** position error (mm) is computed as: $1000 \times \|p_{achieved} - p_{target}\|_2$
- **AND** rotation error (deg) is computed as: $\frac{180}{\pi} \times \text{angularDistance}(q_{achieved}, q_{target})$

#### Scenario: Joint type sensitivity analysis

- **WHEN** benchmarking mixed-joint robots
- **THEN** errors are grouped by joint type (revolute vs prismatic)
- **AND** average error per joint type is reported

### Requirement: Cold Start vs Warm Start Comparison

The benchmark infrastructure SHALL evaluate the impact of initial guess quality on solver performance.

#### Scenario: Cold start benchmark

- **WHEN** user runs `BM_IK_ColdStart`
- **THEN** all IK solves use $q_{init} = 0$ or random initial guess
- **AND** warm start is disabled in solver configuration

#### Scenario: Warm start benchmark

- **WHEN** user runs `BM_IK_WarmStart`
- **THEN** IK solves reuse previous solution as initial guess
- **AND** first solve is primed outside measured region

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
  - "Measuring core algorithm performance?" → `benchmarks/cpp/` or `benchmarks/python/`
  - "Measuring binding overhead?" → `bindings/python/benchmarks/`
  - "Creating test infrastructure?" → `benchmarks/tools/`
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
- **AND** utilities work with urdfx Robot objects

#### Scenario: Visualization import

- **WHEN** benchmark results need visualization
- **THEN** script imports visualization functions from `benchmarks.tools.visualize`
- **AND** functions accept standard result JSON formats

