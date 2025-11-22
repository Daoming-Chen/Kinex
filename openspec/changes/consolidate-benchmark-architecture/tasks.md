## 1. Directory Restructuring

- [ ] 1.1 Create new benchmark subdirectories
  - [ ] `benchmarks/cpp/` (for C++ benchmark executables)
  - [ ] `benchmarks/python/` (for Python benchmark scripts)
  - [ ] `benchmarks/tools/` (for shared utilities)
  - [ ] `benchmarks/results/` (already exists, keep as-is)

- [ ] 1.2 Move existing C++ benchmarks to `benchmarks/cpp/`
  - [ ] Move `ik_benchmarks.cpp`
  - [ ] Move `jacobian_benchmarks.cpp`
  - [ ] Move `mixed_ik_benchmarks.cpp`
  - [ ] Update `benchmarks/CMakeLists.txt` to reflect new paths

- [ ] 1.3 Move core tools from `bindings/python/benchmarks/` to `benchmarks/tools/`
  - [ ] Move `urdf_generator.py` (MixedChainGenerator)
  - [ ] Move `oracle.py` (FKOracle, JointSampler)
  - [ ] Move `visualize_benchmarks.py`
  - [ ] Move `visualize_cpp_benchmarks.py`
  - [ ] Create `benchmarks/tools/__init__.py` with exports

- [ ] 1.4 Move Python benchmark runners to `benchmarks/python/`
  - [ ] Move `run_all_benchmarks.py`
  - [ ] Move `run_benchmarks.py`
  - [ ] Move `run_tier_a_benchmarks.py`
  - [ ] Move `run_tier_b_benchmarks.py`
  - [ ] Create `benchmarks/python/__init__.py`

## 2. Binding Overhead Benchmarks

- [ ] 2.1 Create binding overhead test framework in `bindings/python/benchmarks/`
  - [ ] Create `test_binding_overhead.py` with timing infrastructure
  - [ ] Add FK computation overhead test (Python vs C++)
  - [ ] Add IK solving overhead test
  - [ ] Add Jacobian computation overhead test
  - [ ] Add data conversion overhead tests (NumPy ↔ Eigen)

- [ ] 2.2 Create comparison reporting
  - [ ] Add script to compare Python vs C++ benchmark results
  - [ ] Generate overhead percentage reports
  - [ ] Document acceptable overhead thresholds (target: <10%)

## 3. Import Path Updates

- [ ] 3.1 Update imports in moved Python files
  - [ ] Update `run_all_benchmarks.py` imports
  - [ ] Update `run_tier_a_benchmarks.py` imports
  - [ ] Update `run_tier_b_benchmarks.py` imports
  - [ ] Update visualization script imports

- [ ] 3.2 Create compatibility imports
  - [ ] Add `benchmarks/tools/urdf_generator.py` with proper exports
  - [ ] Add `benchmarks/tools/oracle.py` with proper exports
  - [ ] Ensure backward compatibility during transition

## 4. Build System Updates

- [ ] 4.1 Update CMake configuration
  - [ ] Create `benchmarks/cpp/CMakeLists.txt` with executable definitions
  - [ ] Update root `benchmarks/CMakeLists.txt` to add_subdirectory(cpp)
  - [ ] Update benchmark executable output paths
  - [ ] Verify DLL copying on Windows still works

- [ ] 4.2 Update custom targets
  - [ ] Update `run_benchmarks` target to use new paths
  - [ ] Update `run_mixed_benchmarks` target
  - [ ] Add convenience targets for Python benchmarks

## 5. Documentation Updates

- [ ] 5.1 Create benchmark architecture documentation
  - [ ] Create `docs/benchmarks/architecture.md`
  - [ ] Add directory structure diagram
  - [ ] Document when to add benchmarks to each location
  - [ ] Add decision tree (core vs binding benchmark)

- [ ] 5.2 Update existing benchmark READMEs
  - [ ] Update `benchmarks/README.md` (create if missing) with overall structure
  - [ ] Update `benchmarks/python/README.md` (moved from bindings)
  - [ ] Create `bindings/python/benchmarks/README.md` for overhead tests
  - [ ] Add migration guide for existing benchmark users

- [ ] 5.3 Update project documentation
  - [ ] Update `README.md` benchmark section with new paths
  - [ ] Update `docs/` references to benchmark locations
  - [ ] Update any tutorial/guide references

## 6. CI/CD Pipeline Updates

- [ ] 6.1 Update GitHub Actions workflows
  - [ ] Update benchmark execution paths in CI
  - [ ] Update artifact upload paths
  - [ ] Verify Windows/Linux/macOS paths all work

- [ ] 6.2 Verify benchmark runs
  - [ ] Run C++ benchmarks with new structure
  - [ ] Run Python benchmarks with new structure
  - [ ] Run binding overhead tests
  - [ ] Verify all results are properly generated

## 7. Cleanup

- [ ] 7.1 Remove old files after migration
  - [ ] Remove `bindings/python/benchmarks/urdf_generator.py`
  - [ ] Remove `bindings/python/benchmarks/oracle.py`
  - [ ] Remove `bindings/python/benchmarks/run_*.py` files
  - [ ] Remove `bindings/python/benchmarks/visualize_*.py` files

- [ ] 7.2 Update Python package configuration
  - [ ] Update `bindings/python/setup.py` if benchmark package was exposed
  - [ ] Update `bindings/python/pyproject.toml` benchmark references
  - [ ] Remove stale `__pycache__` directories

## 8. Testing and Validation

- [ ] 8.1 Verify functionality
  - [ ] Test URDF generator from new location
  - [ ] Test oracle functionality from new location
  - [ ] Test all benchmark runners execute successfully
  - [ ] Test visualization scripts produce correct output

- [ ] 8.2 Verify binding overhead tests
  - [ ] Run binding overhead benchmarks
  - [ ] Verify overhead is within acceptable range (<10%)
  - [ ] Generate comparison reports

- [ ] 8.3 Cross-platform verification
  - [ ] Test on Linux
  - [ ] Test on macOS (if available)
  - [ ] Test on Windows
