## ADDED Requirements

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
  - Result conversion overhead (Transform → NumPy)

#### Scenario: Jacobian computation overhead measurement

- **WHEN** overhead benchmark runs Jacobian performance test
- **THEN** it computes Jacobian for same joint configuration in C++ and Python
- **AND** measures matrix computation and conversion time
- **AND** reports overhead for matrix-to-NumPy conversion separately

#### Scenario: Data conversion overhead measurement

- **WHEN** overhead benchmark tests data conversion
- **THEN** it measures time to convert:
  - NumPy array → Eigen::VectorXd (joint angles)
  - Transform object → NumPy position/quaternion
  - Eigen::MatrixXd → NumPy array (Jacobian)
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

## REMOVED Requirements

### Requirement: Comprehensive Performance Benchmarking

**Reason**: Comprehensive performance benchmarking (Tier A/B, DOF sweeps, trajectory tracking) should be in `benchmarks/`, not in the bindings directory. Binding benchmarks should focus solely on measuring binding layer overhead.

**Migration**: All comprehensive benchmark functionality has been moved to `benchmarks/python/` and `benchmarks/tools/`. The Python bindings directory now only contains minimal overhead measurement tests.

**Old requirement summary**: The old requirement specified that `bindings/python/benchmarks/` should contain:
- Tier A/B benchmark runners
- URDF generation tools
- FK oracles and dataset management
- Visualization and reporting infrastructure

**New location**: These capabilities are now specified under the `benchmark-infrastructure` spec and implemented in:
- `benchmarks/python/` (runners)
- `benchmarks/tools/` (generators, oracles, visualizers)
