## ADDED Requirements

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

## MODIFIED Requirements

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
