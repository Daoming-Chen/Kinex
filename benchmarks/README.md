# urdfx Benchmarks

Comprehensive benchmarking suite for evaluating the performance of urdfx inverse kinematics solvers across diverse robot configurations.

## Overview

This directory contains Python-based benchmarking tools for urdfx's IK solver performance. The benchmarks are organized into two tiers:

- **Tier A**: Real-world robots (UR5e configurations, 6-10 DOF)
- **Tier B**: Synthetic mixed-chain robots (8-20 DOF)

![Python IK Benchmarks](results/python_ik_benchmarks.png)

## Quick Start

### Run All Benchmarks

```bash
# Run both Tier A and Tier B benchmarks with visualization
python run_benchmarks.py --all --visualize
```

### Run Individual Tiers

```bash
# Tier A only (real-world robots)
python run_benchmarks.py --tier-a

# Tier B only (synthetic robots)
python run_benchmarks.py --tier-b
```

### Custom Configurations

```bash
# Custom sample count for Tier A
python run_benchmarks.py --tier-a --samples 2000

# Custom DOF range for Tier B
python run_benchmarks.py --tier-b --dof-min 10 --dof-max 15 --samples 500

# Specify output directory
python run_benchmarks.py --all --output results/custom
```

## Benchmark Components

### Main Scripts

#### `run_benchmarks.py`
Unified entry point for running all benchmarks. Provides a consistent interface to execute both Tier A and Tier B benchmarks with optional visualization.

**Key Features:**
- Run all or selected benchmark tiers
- Customizable parameters for each tier
- Integrated visualization pipeline
- Automatic result aggregation

#### `run_tier_a_benchmarks.py`
Benchmarks real-world robot configurations based on the UR5e manipulator.

**Test Configurations:**
- **UR5e** (6 DOF): Standard Universal Robots UR5e
- **UR5e+X** (7 DOF): UR5e with 1 prismatic rail
- **UR5e+XY** (8 DOF): UR5e with 2 prismatic rails
- **UR5e+XYZ** (9 DOF): UR5e with 3 prismatic rails

**Test Scenarios:**
- `ColdStart_Zero`: Initialize from zero configuration
- `ColdStart_Random`: Initialize from random configuration
- `WarmStart`: Initialize from nearby configuration
- `Trajectory`: Sequential solving along trajectory

**Usage:**
```bash
python run_tier_a_benchmarks.py --samples 1000 --seed 42
```

#### `run_tier_b_benchmarks.py`
Benchmarks synthetic robots with mixed revolute/prismatic joints.

**Features:**
- Configurable DOF range (default: 8-20)
- Mixed joint types (80% revolute, 20% prismatic)
- Real-time URDF generation
- Reproducible with seeds

**Usage:**
```bash
python run_tier_b_benchmarks.py --dof-min 8 --dof-max 20 --samples 500
```

### Supporting Modules

#### `oracle.py`
Forward kinematics oracle for generating ground-truth test cases.

**Components:**
- `FKOracle`: Computes FK to generate target poses
- `JointSampler`: Samples valid joint configurations within limits
- `RestrictedJointSampler`: Samples within restricted range to avoid extreme windings

#### `urdf_generator.py`
Generates synthetic URDF files for Tier B benchmarks.

**Features:**
- `MixedChainGenerator`: Creates serial chains with mixed joint types
- Configurable link lengths and joint limits
- Random DH parameters for realistic kinematic structures
- Deterministic generation with seed control

#### `visualize_benchmarks.py`
Generates publication-quality visualizations from benchmark results.

**Capabilities:**
- Parse Google Benchmark JSON format
- Multi-robot comparison charts
- Grouped bar charts by scenario
- Success rate and iteration count analysis
- Customizable output formats (PNG, PDF, SVG)

**Usage:**
```bash
python visualize_benchmarks.py --input results/*.json --output results/
```

## Results Structure

The `results/` directory contains:

```
results/
├── benchmark_summary.md          # Human-readable summary
├── python_ik_benchmarks.png      # Visualization of all benchmarks
├── tier_b_benchmark_results.json # Tier B detailed results
├── ur5e_results.json             # UR5e (6 DOF) results
├── ur5e+x_results.json           # UR5e+X (7 DOF) results
├── ur5e+xy_results.json          # UR5e+XY (8 DOF) results
└── ur5e+xyz_results.json         # UR5e+XYZ (9 DOF) results
```

### Result Metrics

Each benchmark tracks:
- **Execution Time**: Mean solve time per iteration (microseconds)
- **Iterations**: Average number of optimization iterations
- **Success Rate**: Percentage of successfully converged solutions
- **Position Error**: Mean position error for successful solves
- **Orientation Error**: Mean orientation error (quaternion distance)

## Key Findings

Based on the latest benchmark results:

### Tier A (Real-World Robots)

| Robot | Best Scenario | Time (µs) | Iterations | Success Rate |
|-------|---------------|-----------|------------|--------------|
| UR5e (6 DOF) | Trajectory | 35.60 | 12.2 | 87.3% |
| UR5e+X (7 DOF) | Trajectory | 12.14 | 4.2 | 97.0% |
| UR5e+XY (8 DOF) | WarmStart | 6.40 | 3.1 | 100.0% |
| UR5e+XYZ (9 DOF) | Trajectory | 4.83 | 2.6 | 100.0% |

**Observations:**
- Redundant DOF significantly improve convergence
- Warm start initialization reduces solve time by ~50%
- 7+ DOF configurations achieve >95% success rates
- Python bindings show minimal overhead vs C++

### Tier B (Synthetic Robots)

- Tests scalability across 8-20 DOF range
- Mixed joint types (revolute/prismatic) validate solver robustness
- Validates performance on diverse kinematic structures

## Dependencies

Required Python packages:
```bash
pip install numpy scipy matplotlib
```

The benchmarks require the urdfx Python bindings to be built and installed. See `bindings/python/README.md` for build instructions.

## Development

### Adding New Benchmarks

1. Define new test scenarios in the respective tier runner
2. Update `run_benchmarks.py` if adding new tiers
3. Ensure oracle and generator support new configurations
4. Run with visualization to verify results

### Customizing Visualizations

Edit `visualize_benchmarks.py` to:
- Change plot styles and colors
- Add new metrics
- Customize grouping and comparison logic

## Troubleshooting

### urdfx module not found
```bash
# Build and install Python bindings first
cd bindings/python
pip install -e .
```

### Visualization errors
```bash
# Install matplotlib if missing
pip install matplotlib
```

### Low success rates
- Increase `--max-iterations` (default: 100)
- Relax `--tolerance` (default: 1e-6)
- Check URDF file validity
- Verify joint limits are reasonable

## References

- Main Project: [urdfx on GitHub](https://github.com/Daoming-Chen/urdfx)
- Python Bindings: `bindings/python/README.md`
- C++ Benchmarks: `benchmarks/cpp/README.md`

## License

Same as the parent urdfx project.
