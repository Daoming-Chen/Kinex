# Design: Align Benchmark Implementations

## Overview

This design document explains the technical approach to aligning C++ and Python benchmark implementations while maintaining framework-specific idioms.

## Current State Analysis

### C++ Benchmarks (benchmarks/cpp/ik_benchmarks.cpp)

**Test Scenarios:**
- `BM_IK_ColdStart`: Uses zero initialization (`fx.zero()`)
- `BM_IK_WarmStart`: Reuses previous solution (no initial guess passed)
- `BM_IK_Trajectory`: Sequential poses with warm start

**Metrics Reported:**
- `iterations_per_solve`: Average iterations
- `success_rate`: Percentage of converged solutions
- `avg_position_error_mm`: Position error in millimeters
- `avg_rotation_error_deg`: Rotation error in degrees

**Test Data:**
- 1000 random poses generated in fixture
- Seed: 42 (hardcoded)
- Single robot: UR5e only

### Python Benchmarks (benchmarks/python/run_tier_a_benchmarks.py)

**Test Scenarios:**
- `cold_start_zero`: Zero initialization
- `cold_start_random`: Random initialization far from solution
- `warm_start`: Small noise around ground truth (~0.1 rad)

**Metrics Reported:**
- `avg_time_us`: Average solve time
- `success_rate`: Percentage of successful solves
- `avg_iterations`: Average iterations per solve
- `avg_pos_error_mm`: Position error in millimeters
- `avg_rot_error`: Rotation error (radians, not degrees)

**Test Data:**
- 1000 samples (configurable via --samples)
- Seed: 42 (configurable via --seed)
- Multiple robots: ur5e, ur5e+x, ur5e+xy, ur5e+xyz

**Missing:**
- Trajectory tracking scenario

## Design Decisions

### 1. Three Standard Scenarios

Both implementations will support:

#### Cold Start Zero
- **Initial guess**: All joints at zero (`q_init = 0`)
- **Purpose**: Tests solver from worst-case initialization
- **Implementation**: 
  - C++: `solver->solve(target_pose, zero_config, result)`
  - Python: `q_init = np.zeros(dof)`

#### Cold Start Random
- **Initial guess**: Random joints within limits, far from solution
- **Purpose**: Tests solver robustness to poor initialization
- **Implementation**:
  - C++: Generate random configs with same sampler as targets
  - Python: Pre-generate random configs, reuse across samples

#### Trajectory
- **Initial guess**: Previous solution (warm start enabled)
- **Purpose**: Tests solver on sequential waypoints (realistic scenario)
- **Implementation**:
  - C++: Already implemented, sequential poses with warm start
  - Python: Add new scenario following C++ pattern

### 2. Metric Alignment

Standardize metric names and units:

| Metric | Name | Unit | Formula |
|--------|------|------|---------|
| Solve time | `real_time` / `avg_time_us` | µs | Wall-clock time per solve |
| Success rate | `success_rate` | % | `(converged / total) * 100` |
| Iterations | `iterations_per_solve` / `avg_iterations` | count | Total iterations / total solves |
| Position error | `avg_position_error_mm` | mm | `‖p_achieved - p_target‖₂ * 1000` |
| Rotation error | `avg_rotation_error_deg` | deg | `angularDistance * 180/π` |

**Change required**: Python must convert rotation error from radians to degrees.

### 3. Test Dataset Generation

Both implementations will use:
- **Sample count**: 1000 (configurable)
- **Random seed**: 42 (for reproducibility)
- **Sampling method**: Uniform random within joint limits
- **Target generation**: FK on random joint configs

**C++ approach**: Pre-generate all samples in fixture
**Python approach**: Generate on-the-fly (memory efficient)

Both are acceptable as long as seed and count match.

### 4. Multi-Robot Support in C++

#### Option A: Multiple Benchmark Functions
```cpp
static void BM_IK_UR5E_ColdStart_Zero(benchmark::State& state) { ... }
static void BM_IK_UR5E_X_ColdStart_Zero(benchmark::State& state) { ... }
// ... 12 total functions (4 robots × 3 scenarios)
```

**Pros**: Simple, explicit
**Cons**: Code duplication, maintenance burden

#### Option B: Parameterized Benchmarks
```cpp
static void BM_IK_Generic(benchmark::State& state, const std::string& urdf_path, 
                          const std::string& scenario) { ... }

BENCHMARK_CAPTURE(BM_IK_Generic, UR5E_ColdZero, "ur5e.urdf", "cold_zero");
BENCHMARK_CAPTURE(BM_IK_Generic, UR5E_ColdRand, "ur5e.urdf", "cold_random");
// ... etc
```

**Pros**: Shared logic, maintainable
**Cons**: More complex fixture management

#### Option C: Robot Loop Inside Benchmark (Current Pattern)
```cpp
static void BM_IK_ColdStart_Zero(benchmark::State& state) {
    for (const auto& robot_config : all_robot_configs) {
        // Run benchmark for this robot
    }
}
```

**Pros**: Minimal code changes
**Cons**: Less granular reporting, harder to filter specific robots

**Decision**: Use **Option B (Parameterized Benchmarks)** for best granularity and maintainability.

### 5. Trajectory Implementation

#### C++ (Current)
- Pre-generates trajectory poses in fixture
- Solves sequentially with warm start
- Metrics aggregated across all waypoints

#### Python (To Add)
- Generate trajectory by adding small deltas to joint configs
- Solve each waypoint using previous solution as init
- Match C++ trajectory generation logic

**Implementation**:
```python
# Generate trajectory (25 waypoints)
trajectory_q = []
q_current = sampler.sample(1)[0]  # Start from random config
for i in range(25):
    q_current = q_current + np.random.uniform(-0.08, 0.08, size=dof)
    q_current = np.clip(q_current, lower_limits, upper_limits)
    trajectory_q.append(q_current)

# Solve trajectory
q_prev = None
for q_target in trajectory_q:
    target_pose = oracle.compute_pose(q_target)
    if q_prev is None:
        q_init = np.zeros(dof)  # First waypoint: cold start
    else:
        q_init = q_prev  # Subsequent: warm start
    
    result = solver.solve(target_pose, q_init)
    q_prev = result.solution
```

### 6. Output Format Consistency

Both implementations output Google Benchmark JSON format:

```json
{
  "context": {
    "date": "...",
    "host_name": "...",
    "num_cpus": 16,
    "library_build_type": "release"
  },
  "benchmarks": [
    {
      "name": "BM_IK_ColdStart_Zero/UR5E",
      "real_time": 152.23,
      "cpu_time": 152.23,
      "time_unit": "us",
      "iterations_per_solve": 20.7,
      "success_rate": 85.4,
      "avg_position_error_mm": 0.0012,
      "avg_rotation_error_deg": 0.0034
    }
  ]
}
```

## Implementation Plan

### C++ Changes

1. **Add scenario parameter to fixture**:
   - Extend `IKBenchmarkFixture` to support multiple scenarios
   - Add method: `generateRandomInitialGuesses()`

2. **Implement three parameterized benchmarks**:
   ```cpp
   BENCHMARK_CAPTURE(BM_IK_Parameterized, UR5E_ColdZero, "ur5e.urdf", Scenario::ColdStartZero);
   BENCHMARK_CAPTURE(BM_IK_Parameterized, UR5E_ColdRand, "ur5e.urdf", Scenario::ColdStartRandom);
   BENCHMARK_CAPTURE(BM_IK_Parameterized, UR5E_Trajectory, "ur5e.urdf", Scenario::Trajectory);
   // Repeat for ur5e+x, ur5e+xy, ur5e+xyz
   ```

3. **Update metric names**:
   - Keep existing names, they already align well

4. **Configure URDF paths**:
   - Extend CMake to pass all variant paths
   - Use macro: `kinex_UR5_VARIANT_PATHS`

### Python Changes

1. **Add trajectory scenario to run_tier_a_benchmarks.py**:
   - Implement trajectory generation (match C++ logic)
   - Add `"trajectory": {}` to results dict
   - Generate trajectory test cases

2. **Convert rotation error to degrees**:
   ```python
   avg_rot_error_deg = np.mean(rot_errors) * 180.0 / np.pi
   ```

3. **Update JSON output**:
   - Add trajectory entry to benchmarks list
   - Ensure metric names match C++ exactly

## Validation Strategy

1. **Identical seeds**: Both use seed=42
2. **Identical sample counts**: Both use 1000 samples
3. **FK validation**: Verify Python FK oracle matches C++ FK (error < 1e-6)
4. **Cross-language comparison**: Compare metrics for same robot/scenario
   - Success rates should be similar (±5%)
   - Iteration counts should be identical for converged cases
   - Errors should match (±1e-6 accounting for float precision)
   - Python times will be higher due to binding overhead (~5-15%)

## Testing

1. **Unit tests**: Not applicable (benchmarks are integration tests)
2. **Validation test**: Create script that runs both and compares outputs
3. **CI integration**: Run benchmarks on PR, fail if success rate drops >10%

## Documentation Updates

Update `benchmarks/README.md`:
- Document three scenarios with exact definitions
- Show example results comparing C++ vs Python
- Explain expected differences (binding overhead)
- Document how to run benchmarks for specific robots/scenarios

## Rollout

1. Implement C++ changes first (more complex due to Google Benchmark API)
2. Test C++ benchmarks in isolation
3. Implement Python changes
4. Run validation comparison
5. Update docs
6. Merge and archive proposal
