# urdfx Benchmarks

Comprehensive performance evaluation suite for urdfx.

## Architecture

The benchmark suite is organized into three main directories:

```
benchmarks/
├── cpp/          # C++ benchmark implementations (Google Benchmark)
├── python/       # Python benchmark implementations
├── tools/        # Shared utilities (generators, oracles, visualizers)
└── results/      # Benchmark output files and visualizations
```

### Directory Purposes

#### `cpp/`
Native C++ performance benchmarks measuring core library performance:
- IK solver performance (various DOF configurations)
- Jacobian computation performance
- Forward kinematics performance

Uses Google Benchmark framework for precise measurements.

#### `python/`
Python implementations of core library benchmarks:
- Tier A: Real-world robots (UR5e variants, 6-10 DOF)
- Tier B: Synthetic mixed-chain robots (8-20 DOF)

These measure urdfx performance through the Python binding layer.

#### `tools/`
Shared infrastructure used by both C++ and Python benchmarks:
- `MixedChainGenerator`: Generates synthetic URDF models
- `FKOracle`: Forward kinematics validation
- `JointSampler`: Joint configuration sampling
- Visualization scripts for benchmark results

## Quick Start

### Run All Benchmarks

```bash
# From benchmarks/python/
python run_all_benchmarks.py
```

This runs:
1. Python Tier A benchmarks (real-world robots)
2. Python Tier B benchmarks (synthetic robots)
3. C++ IK benchmarks
4. C++ Jacobian benchmarks
5. Generates visualizations

### Run Python Benchmarks Only

```bash
# Run both tiers
python run_benchmarks.py --all

# Tier A only
python run_tier_a_benchmarks.py --samples 2000

# Tier B only
python run_tier_b_benchmarks.py --dof-min 10 --dof-max 15
```

### Run C++ Benchmarks

```bash
# Build first
cmake --build build --target ik_benchmarks

# Run
./build/benchmarks/cpp/ik_benchmarks
./build/benchmarks/cpp/jacobian_benchmarks
```

Or use CMake targets:
```bash
cmake --build build --target run_benchmarks
```

## When to Add Benchmarks Where

### Add to `cpp/` when:
- Measuring raw C++ performance
- Micro-benchmarking specific algorithms
- Performance-critical code optimization
- Need sub-microsecond precision

### Add to `python/` when:
- Benchmarking complete workflows
- Real-world performance scenarios
- Python user-facing performance
- Comparing different algorithms/configurations

### Add to `tools/` when:
- Creating reusable test infrastructure
- Building dataset generators
- Adding visualization tools
- Writing validation utilities

### Add to `bindings/python/benchmarks/` when:
- Measuring Python binding overhead specifically
- Comparing Python vs C++ performance
- Testing data conversion performance

## Using Shared Tools

### Generate Synthetic Robot

```python
from tools.urdf_generator import MixedChainGenerator

gen = MixedChainGenerator(dof=12, prismatic_prob=0.2, seed=42)
urdf_string = gen.to_urdf_string()
gen.save_urdf("robot_12dof.urdf")
```

### Validate with FK Oracle

```python
from tools.oracle import FKOracle, JointSampler
import urdfx

robot = urdfx.Robot.from_urdf_file("robot.urdf")
oracle = FKOracle(robot)
sampler = JointSampler(robot)

# Generate test data
q_samples = sampler.sample(1000)
pos, rot = oracle.compute_pose(q_samples[0])
```

### Visualize Results

```bash
# Python benchmarks
python tools/visualize_benchmarks.py --results-dir results/

# C++ benchmarks
python tools/visualize_cpp_benchmarks.py --results-dir results/
```

## Results

Benchmark results are saved to `results/`:
- `*_results.json`: Raw benchmark data
- `benchmark_summary.md`: Python benchmark summary
- `cpp_benchmark_summary.md`: C++ benchmark summary
- `*.png`, `*.pdf`: Visualization charts

## Requirements

### Python
- urdfx Python bindings (built from project root)
- NumPy
- SciPy
- Matplotlib (for visualizations)

### C++
- Google Benchmark (automatically fetched by CMake)
- urdfx core library

## Performance Targets

### IK Solver (SQP)
- 6-DOF: <100 µs per solve (cold start)
- 10-DOF: <500 µs per solve (cold start)
- 20-DOF: <2000 µs per solve (cold start)
- Success rate: >95% for reachable poses

### Jacobian
- 6-DOF: <10 µs
- 12-DOF: <30 µs
- 20-DOF: <70 µs

### Python Binding Overhead
- <10% overhead for computational operations
- See `bindings/python/benchmarks/` for binding-specific tests

## Contributing

When adding benchmarks:
1. Follow the architectural guidelines above
2. Use consistent naming conventions
3. Document expected performance targets
4. Include visualization of results
5. Test on multiple DOF configurations

For detailed architectural decisions, see `openspec/changes/consolidate-benchmark-architecture/`.
