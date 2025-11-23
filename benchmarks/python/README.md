# urdfx Python Benchmarks

Python implementations of core library benchmarks, measuring urdfx performance through the Python binding layer.

## Overview

This directory contains:
- **Tier A benchmarks**: Real-world robots (UR5e variants, 6-10 DOF)
- **Tier B benchmarks**: Synthetic mixed-chain robots (8-20 DOF)
- **Master runner**: Orchestrates all benchmarks (Python + C++)

## Quick Start

### Run All Benchmarks

```bash
# Run everything (Python + C++)
python run_all_benchmarks.py

# Quick test with fewer samples
python run_all_benchmarks.py --samples 500

# Skip C++ benchmarks
python run_all_benchmarks.py --skip-cpp
```

### Run Python Benchmarks Only

```bash
# Run both Tier A and Tier B
python run_benchmarks.py --all

# Run only Tier A (real-world robots)
python run_benchmarks.py --tier-a

# Run only Tier B (synthetic robots)
python run_benchmarks.py --tier-b

# With visualization
python run_benchmarks.py --all --visualize
```

### Run Specific Tiers

```bash
# Tier A: Real-world robots
python run_tier_a_benchmarks.py --samples 2000

# Tier B: Synthetic robots with custom DOF range
python run_tier_b_benchmarks.py --dof-min 10 --dof-max 15 --robots-per-dof 5
```

## Benchmark Structure

### Tier A: Real-World Robots
Tests on UR5e configurations:
- `ur5e` (6-DOF)
- `ur5e+x` (7-DOF: UR5e + 1 prismatic)
- `ur5e+xy` (8-DOF: UR5e + 2 prismatic)
- `ur5e+xyz` (9-DOF: UR5e + 3 prismatic)

**Purpose**: Measure performance on real robot configurations

### Tier B: Synthetic Robots
Generated robots with:
- Mixed joint types (revolute + prismatic)
- 8-20 DOF configurations
- Varied link geometries

**Purpose**: Evaluate scalability and handling of diverse kinematic chains

## Test Scenarios

Each benchmark tests three initialization strategies:

1. **Cold Start (Zero)**: Initial guess = zero vector
   - Measures worst-case convergence
   - Reflects first-time solving

2. **Cold Start (Random)**: Initial guess = random configuration
   - Tests convergence from arbitrary states
   - Evaluates solver robustness

3. **Warm Start**: Initial guess = near-solution
   - Measures iteration reduction from warm starting
   - Reflects trajectory-following performance

## Results

Results are saved to `../../results/`:
- `{robot_name}_results.json`: Per-robot benchmark data
- `tier_b_benchmark_results.json`: Tier B aggregated results
- `benchmark_summary.md`: Human-readable summary

## Shared Tools

Common utilities are in `../tools/`:
- `MixedChainGenerator`: URDF generation
- `FKOracle`: Forward kinematics validation
- `JointSampler`: Joint configuration sampling
- Visualization scripts

Import example:
```python
from tools.urdf_generator import MixedChainGenerator
from tools.oracle import FKOracle, JointSampler
```

## Requirements

- urdfx Python bindings (built from project root)
- NumPy
- SciPy
- Matplotlib (for visualizations)

## Performance Targets

See parent `benchmarks/README.md` for detailed performance expectations.
