# Tasks: Align Benchmark Implementations

## Phase 1: Analyze Current State
- [x] Document exact differences between C++ and Python benchmark scenarios
- [x] List all metrics reported by each implementation
- [x] Identify shared test dataset requirements
- [x] Review UR5e variant URDF availability

## Phase 2: Update C++ Benchmarks
- [x] Add BM_IK_ColdStart_Random benchmark to ik_benchmarks.cpp
- [x] Rename BM_IK_ColdStart to BM_IK_ColdStart_Zero for clarity
- [x] Update BM_IK_Trajectory to match Python trajectory implementation
- [x] Add support for multiple robot URDF files (UR5e variants)
- [x] Ensure all benchmarks report consistent metrics:
  - avg_time_us (real_time)
  - success_rate (%)
  - iterations_per_solve
  - avg_position_error_mm
  - avg_rotation_error_deg
- [x] Use seed=42 and num_samples=1000 for reproducibility
- [x] Update CMakeLists.txt to configure multiple URDF paths if needed

## Phase 3: Update Python Benchmarks
- [x] Add trajectory tracking scenario to run_tier_a_benchmarks.py
- [x] Ensure trajectory implementation matches C++ logic (sequential warm start)
- [x] Verify metric names match C++ output format
- [x] Confirm JSON output format is compatible with visualization scripts
- [x] Ensure seed=42 and num_samples=1000 are defaults

## Phase 4: Test Dataset Alignment
- [x] Verify both implementations use same random seed (42)
- [x] Verify both implementations use same sample count (1000)
- [x] Confirm FK oracle produces identical results between C++ and Python
- [x] Document any acceptable differences (e.g., floating point precision)

## Phase 5: Documentation Updates
- [x] Update benchmarks/README.md with aligned scenario descriptions
- [x] Document the three scenarios: ColdStart_Zero, ColdStart_Random, Trajectory
- [x] Explain metric definitions and units
- [x] Add examples showing C++ vs Python result comparison
- [x] Note that Python Tier B is additional capability beyond C++

## Phase 6: Validation
- [x] Run C++ benchmarks for all UR5e variants
- [x] Run Python Tier A benchmarks for all UR5e variants
- [x] Compare results and verify metrics are comparable (within binding overhead)
- [x] Verify visualization scripts handle both outputs correctly
- [ ] Run `openspec validate align-benchmark-implementations --strict`

## Phase 7: Archive and Finalize
- [ ] Update benchmark-infrastructure spec with aligned requirements
- [ ] Create PR with all changes
- [ ] After merge, archive this change to openspec/changes/archive/
