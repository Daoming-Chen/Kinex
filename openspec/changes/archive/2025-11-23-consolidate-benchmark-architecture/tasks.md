## 1. Directory Restructuring

- [x] 1.1 Create new benchmark subdirectories
  - [x] `benchmarks/cpp/` (for C++ benchmark executables)
  - [x] `benchmarks/python/` (for Python benchmark scripts)
  - [x] `benchmarks/tools/` (for shared utilities)
  - [x] `benchmarks/results/` (already exists, keep as-is)

- [x] 1.2 Move existing C++ benchmarks to `benchmarks/cpp/`
  - [x] Move `ik_benchmarks.cpp`
  - [x] Move `jacobian_benchmarks.cpp`
  - [x] Move `mixed_ik_benchmarks.cpp`
  - [x] Update `benchmarks/CMakeLists.txt` to reflect new paths

- [x] 1.3 Move core tools from `bindings/python/benchmarks/` to `benchmarks/tools/`
  - [x] Move `urdf_generator.py` (MixedChainGenerator)
  - [x] Move `oracle.py` (FKOracle, JointSampler)
  - [x] Move `visualize_benchmarks.py`
  - [x] Move `visualize_cpp_benchmarks.py`
  - [x] Create `benchmarks/tools/__init__.py` with exports

- [x] 1.4 Move Python benchmark runners to `benchmarks/python/`
  - [x] Move `run_all_benchmarks.py`
  - [x] Move `run_benchmarks.py`
  - [x] Move `run_tier_a_benchmarks.py`
  - [x] Move `run_tier_b_benchmarks.py`
  - [x] Create `benchmarks/python/__init__.py`

## 2. Binding Overhead Benchmarks

- [x] 2.1 Create binding overhead test framework in `bindings/python/benchmarks/`
  - [x] Create `test_binding_overhead.py` with timing infrastructure
  - [x] Add FK computation overhead test (Python vs C++)
  - [x] Add IK solving overhead test
  - [x] Add Jacobian computation overhead test
  - [x] Add data conversion overhead tests (NumPy ↔ Eigen)

- [x] 2.2 Create comparison reporting
  - [x] Add script to compare Python vs C++ benchmark results
  - [x] Generate overhead percentage reports
  - [x] Document acceptable overhead thresholds (target: <10%)

## 3. Import Path Updates

- [x] 3.1 Update imports in moved Python files
  - [x] Update `run_all_benchmarks.py` imports
  - [x] Update `run_tier_a_benchmarks.py` imports
  - [x] Update `run_tier_b_benchmarks.py` imports
  - [x] Update visualization script imports

- [x] 3.2 Create compatibility imports
  - [x] Add `benchmarks/tools/urdf_generator.py` with proper exports
  - [x] Add `benchmarks/tools/oracle.py` with proper exports
  - [x] Ensure backward compatibility during transition

## 4. Build System Updates

- [x] 4.1 Update CMake configuration
  - [x] Create `benchmarks/cpp/CMakeLists.txt` with executable definitions
  - [x] Update root `benchmarks/CMakeLists.txt` to add_subdirectory(cpp)
  - [x] Update benchmark executable output paths
  - [x] Verify DLL copying on Windows still works

- [x] 4.2 Update custom targets
  - [x] Update `run_benchmarks` target to use new paths
  - [x] Update `run_mixed_benchmarks` target
  - [x] Add convenience targets for Python benchmarks

## 5. Documentation Updates

- [x] 5.1 Create benchmark architecture documentation
  - [x] Create `docs/benchmarks/architecture.md`
  - [x] Add directory structure diagram
  - [x] Document when to add benchmarks to each location
  - [x] Add decision tree (core vs binding benchmark)

- [x] 5.2 Update existing benchmark READMEs
  - [x] Update `benchmarks/README.md` (create if missing) with overall structure
  - [x] Update `benchmarks/python/README.md` (moved from bindings)
  - [x] Create `bindings/python/benchmarks/README.md` for overhead tests
  - [x] Add migration guide for existing benchmark users

- [x] 5.3 Update project documentation
  - [x] Update `README.md` benchmark section with new paths
  - [x] Update `docs/` references to benchmark locations
  - [x] Update any tutorial/guide references

## 6. CI/CD Pipeline Updates

- [x] 6.1 Update GitHub Actions workflows
  - [x] Update benchmark execution paths in CI
  - [x] Update artifact upload paths
  - [x] Verify Windows/Linux/macOS paths all work

- [x] 6.2 Verify benchmark runs
  - [x] Run C++ benchmarks with new structure
  - [x] Run Python benchmarks with new structure
  - [x] Run binding overhead tests
  - [x] Verify all results are properly generated

## 7. Cleanup

- [x] 7.1 Remove old files after migration
  - [x] Remove `bindings/python/benchmarks/urdf_generator.py`
  - [x] Remove `bindings/python/benchmarks/oracle.py`
  - [x] Remove `bindings/python/benchmarks/run_*.py` files
  - [x] Remove `bindings/python/benchmarks/visualize_*.py` files

- [x] 7.2 Update Python package configuration
  - [x] Update `bindings/python/setup.py` if benchmark package was exposed
  - [x] Update `bindings/python/pyproject.toml` benchmark references
  - [x] Remove stale `__pycache__` directories

## 8. Testing and Validation

- [x] 8.1 Verify functionality
  - [x] Test URDF generator from new location
  - [x] Test oracle functionality from new location
  - [x] Test all benchmark runners execute successfully
  - [x] Test visualization scripts produce correct output

- [x] 8.2 Verify binding overhead tests
  - [x] Run binding overhead benchmarks
  - [x] Verify overhead is within acceptable range (<10%)
  - [x] Generate comparison reports

- [x] 8.3 Cross-platform verification
  - [x] Test on Linux
  - [x] Test on macOS (if available)
  - [x] Test on Windows
