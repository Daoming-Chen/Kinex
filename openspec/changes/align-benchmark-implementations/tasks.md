# Tasks: Align Benchmark Implementations

## Phase 1: Analyze Current State
- [ ] Document exact differences between C++ and Python benchmark scenarios
- [ ] List all metrics reported by each implementation
- [ ] Identify shared test dataset requirements
- [ ] Review UR5e variant URDF availability

## Phase 2: Update C++ Benchmarks
- [ ] Add BM_IK_ColdStart_Random benchmark to ik_benchmarks.cpp
- [ ] Rename BM_IK_ColdStart to BM_IK_ColdStart_Zero for clarity
- [ ] Update BM_IK_Trajectory to match Python trajectory implementation
- [ ] Add support for multiple robot URDF files (UR5e variants)
- [ ] Ensure all benchmarks report consistent metrics:
  - avg_time_us (real_time)
  - success_rate (%)
  - iterations_per_solve
  - avg_position_error_mm
  - avg_rotation_error_deg
- [ ] Use seed=42 and num_samples=1000 for reproducibility
- [ ] Update CMakeLists.txt to configure multiple URDF paths if needed

## Phase 3: Update Python Benchmarks
- [ ] Add trajectory tracking scenario to run_tier_a_benchmarks.py
- [ ] Ensure trajectory implementation matches C++ logic (sequential warm start)
- [ ] Verify metric names match C++ output format
- [ ] Confirm JSON output format is compatible with visualization scripts
- [ ] Ensure seed=42 and num_samples=1000 are defaults

## Phase 4: Test Dataset Alignment
- [ ] Verify both implementations use same random seed (42)
- [ ] Verify both implementations use same sample count (1000)
- [ ] Confirm FK oracle produces identical results between C++ and Python
- [ ] Document any acceptable differences (e.g., floating point precision)

## Phase 5: Documentation Updates
- [ ] Update benchmarks/README.md with aligned scenario descriptions
- [ ] Document the three scenarios: ColdStart_Zero, ColdStart_Random, Trajectory
- [ ] Explain metric definitions and units
- [ ] Add examples showing C++ vs Python result comparison
- [ ] Note that Python Tier B is additional capability beyond C++

## Phase 6: Validation
- [ ] Run C++ benchmarks for all UR5e variants
- [ ] Run Python Tier A benchmarks for all UR5e variants
- [ ] Compare results and verify metrics are comparable (within binding overhead)
- [ ] Verify visualization scripts handle both outputs correctly
- [ ] Run `openspec validate align-benchmark-implementations --strict`

## Phase 7: Archive and Finalize
- [ ] Update benchmark-infrastructure spec with aligned requirements
- [ ] Create PR with all changes
- [ ] After merge, archive this change to openspec/changes/archive/
