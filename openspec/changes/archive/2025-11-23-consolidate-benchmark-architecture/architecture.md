# Benchmark Architecture

This document describes the organization and purpose of benchmarks in the kinex project.

## Directory Structure

```
benchmarks/
├── cpp/                    # Native C++ performance benchmarks
�?  ├── ik_benchmarks.cpp
�?  ├── jacobian_benchmarks.cpp
�?  └── mixed_ik_benchmarks.cpp
├── python/                 # Python benchmark runners (core performance)
�?  ├── run_all_benchmarks.py
�?  ├── run_tier_a_benchmarks.py
�?  └── run_tier_b_benchmarks.py
├── tools/                  # Shared benchmark utilities
�?  ├── __init__.py
�?  ├── urdf_generator.py   # MixedChainGenerator
�?  ├── oracle.py           # FKOracle, JointSampler
�?  └── visualize.py        # Chart generation
└── results/                # Benchmark outputs (JSON, plots, reports)

bindings/python/benchmarks/  # Binding overhead tests only
└── test_binding_overhead.py
```

## Three Types of Benchmarks

### 1. Core C++ Benchmarks (`benchmarks/cpp/`)

**Purpose**: Measure native C++ library performance (baseline)

**What they test**:
- IK solver convergence speed and accuracy
- Jacobian computation performance
- Forward kinematics efficiency
- Scalability across different DOF (6-100+)

**Framework**: Google Benchmark

**When to add here**: When measuring core algorithm performance in C++

**Example**: "How fast does the SQP IK solver converge for a 20-DOF robot?"

### 2. Python Benchmarks (`benchmarks/python/`)

**Purpose**: Measure comprehensive performance of Python API

**What they test**:
- Tier A: Real-world robots (UR5e variants)
- Tier B: Synthetic mixed-joint robots
- End-to-end workflows (dataset generation �?solving �?reporting)

**Framework**: Custom Python harness using `kinex` Python bindings

**When to add here**: When measuring real-world performance through Python API

**Example**: "What's the success rate for IK solving on 50 random targets for a UR5e+xyz robot?"

### 3. Binding Overhead Tests (`bindings/python/benchmarks/`)

**Purpose**: Measure Python binding layer overhead vs C++ baseline

**What they test**:
- FK/IK/Jacobian computation time (Python vs C++)
- Data conversion costs (NumPy �?Eigen)
- Binding setup overhead

**Target**: <10% overhead for computational operations

**When to add here**: When measuring the cost of the binding layer itself

**Example**: "How much slower is `fk.compute(q)` in Python compared to C++?"

## Shared Tools (`benchmarks/tools/`)

These utilities are used by both C++ and Python benchmarks:

- **`MixedChainGenerator`**: Generate synthetic URDF files with mixed revolute/prismatic joints
- **`FKOracle`**: Forward kinematics wrapper for ground truth computation
- **`JointSampler`**: Sample random joint configurations within limits
- **Visualization**: Generate plots and reports from benchmark results

**Usage**:
```python
from benchmarks.tools import MixedChainGenerator, FKOracle, JointSampler
```

## Decision Tree: Where Should My Benchmark Go?

```
What are you measuring?
�?
├─ "How fast is the core C++ algorithm?"
�? └─ benchmarks/cpp/
�?
├─ "How well does it work for real robots via Python?"
�? └─ benchmarks/python/
�?
├─ "What's the Python binding overhead?"
�? └─ bindings/python/benchmarks/
�?
└─ "I'm creating a shared utility (generator, oracle, etc.)"
   └─ benchmarks/tools/
```

## Running Benchmarks

### C++ Benchmarks
```bash
cmake -B build -DBUILD_BENCHMARKS=ON
cmake --build build
./build/benchmarks/cpp/ik_benchmarks
./build/benchmarks/cpp/mixed_ik_benchmarks
```

### Python Benchmarks
```bash
cd benchmarks/python
python run_all_benchmarks.py --samples 1000 --visualize
```

### Binding Overhead Tests
```bash
cd bindings/python/benchmarks
python test_binding_overhead.py
```

## Adding New Benchmarks

### Adding a C++ Benchmark

1. Create `benchmarks/cpp/my_benchmark.cpp`
2. Add executable in `benchmarks/cpp/CMakeLists.txt`:
   ```cmake
   add_executable(my_benchmark my_benchmark.cpp)
   target_link_libraries(my_benchmark PRIVATE kinex benchmark::benchmark_main)
   ```
3. Use Google Benchmark macros: `BENCHMARK(MyFunction)`

### Adding a Python Benchmark

1. Create script in `benchmarks/python/`
2. Import tools: `from benchmarks.tools import MixedChainGenerator, FKOracle`
3. Generate results in `benchmarks/results/`
4. Update master runner if needed

### Adding a Binding Overhead Test

1. Add test function to `bindings/python/benchmarks/test_binding_overhead.py`
2. Measure identical operation in both C++ and Python
3. Compute and report overhead percentage
4. Verify against <10% threshold

## Migration from Old Structure

**Before** (incorrect):
```
bindings/python/benchmarks/
├── urdf_generator.py         # Core tool in wrong location
├── oracle.py                 # Core tool in wrong location
├── run_tier_a_benchmarks.py  # Core benchmark in wrong location
└── visualize_benchmarks.py   # Core tool in wrong location
```

**After** (correct):
```
benchmarks/
├── cpp/                      # Native benchmarks
├── python/                   # Python benchmark runners
├── tools/                    # Shared utilities
└── results/

bindings/python/benchmarks/
└── test_binding_overhead.py  # Only binding overhead tests
```

## Design Principles

1. **Single source of truth**: `benchmarks/` is the authoritative location for performance evaluation
2. **Clear separation**: Core performance vs binding overhead are distinct concerns
3. **Shared infrastructure**: Generators and oracles are reusable across all benchmarks
4. **Discoverable**: Natural first check (`benchmarks/`) contains everything you need

## Performance Targets

| Operation | C++ Target | Python Overhead Target |
|-----------|-----------|------------------------|
| FK (6-DOF) | <1ms | <10% |
| IK solve (6-DOF) | <10ms | <10% |
| Jacobian (6-DOF) | <0.5ms | <10% |
| NumPy conversion | N/A | <50μs per array |

## References

- Google Benchmark documentation: https://github.com/google/benchmark
- Tier A/B benchmark specification: `openspec/specs/benchmark-infrastructure/spec.md`
- Python bindings specification: `openspec/specs/python-bindings/spec.md`
