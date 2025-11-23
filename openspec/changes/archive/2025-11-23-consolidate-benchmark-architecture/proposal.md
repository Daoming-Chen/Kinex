# Change: Consolidate Benchmark Architecture

## Why

Currently, the core benchmark infrastructure (URDF generation, dataset creation, FK oracle, visualization) is located in `bindings/python/benchmarks/`, which is architecturally incorrect. This creates several problems:

1. **Misleading organization**: Core benchmarking tools appear to be Python-specific when they're actually project-wide infrastructure
2. **Unclear purpose**: Bindings should have minimal benchmarks focused on measuring binding overhead, not comprehensive performance analysis
3. **Reduced discoverability**: Developers looking for benchmarking tools naturally check the root `benchmarks/` directory first
4. **Maintenance confusion**: It's unclear which benchmarks test core library performance vs binding overhead

The root `benchmarks/` directory should be the single source of truth for performance evaluation, with binding-specific benchmarks limited to measuring binding overhead only.

## What Changes

**Structural reorganization:**
- Move core benchmark infrastructure from `bindings/python/benchmarks/` to `benchmarks/`
  - `urdf_generator.py` (MixedChainGenerator) → `benchmarks/tools/`
  - `oracle.py` (FKOracle, JointSampler) → `benchmarks/tools/`
  - Dataset visualization scripts → `benchmarks/tools/`
  - Master runners and Tier A/B benchmarks → `benchmarks/python/`
- Create `benchmarks/python/` for Python implementations of core benchmarks (measures Python binding performance)
- Create `benchmarks/cpp/` and move existing C++ benchmarks there for consistency
- Create `benchmarks/tools/` for shared utilities (generators, visualizers, oracles)
- Keep minimal binding overhead tests in `bindings/python/benchmarks/`

**New binding overhead benchmarks:**
- Simple FK/IK/Jacobian timing comparisons (Python vs C++)
- Data conversion overhead measurements (NumPy ↔ Eigen)
- GIL impact analysis for threaded scenarios

**Documentation updates:**
- Update all benchmark README files to clarify purpose and scope
- Add architecture diagram showing benchmark organization
- Document when to add benchmarks to each location

## Impact

**Affected specs:**
- `benchmark-infrastructure`: Add organizational requirements, clarify shared tools location
- `python-bindings`: Remove comprehensive benchmarking requirements, add binding overhead testing requirements

**Affected code:**
- `benchmarks/`: New subdirectories (`cpp/`, `python/`, `tools/`, `results/`)
- `bindings/python/benchmarks/`: Reduced to binding overhead tests only
- All benchmark runner scripts: Updated paths
- CMakeLists.txt files: Updated to reflect new structure
- CI/CD workflows: Updated benchmark execution paths

**Breaking changes:**
None - this is purely internal reorganization with no API changes. Script invocation paths will change but can be documented in migration guide.

**Migration guide needed:**
- Update developer documentation with new benchmark locations
- Provide script path mappings (old → new)
- Update CI/CD pipelines
