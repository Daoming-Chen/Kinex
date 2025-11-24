# Proposal: Align Benchmark Implementations

## Context

Currently, the C++ and Python benchmark implementations have inconsistencies in:
1. **Test scenarios**: C++ has ColdStart (zero init), WarmStart, and Trajectory, while Python has ColdStart_Zero, ColdStart_Random, and WarmStart
2. **Metrics**: Different metric reporting between C++ and Python
3. **Test datasets**: Different approaches to generating test cases
4. **Robot configurations**: Python Tier A tests UR5e variants, but C++ only tests base UR5e

This misalignment makes it difficult to compare performance between C++ and Python bindings and assess binding overhead accurately.

## Problem

Users need consistent benchmarks across C++ and Python to:
- Compare raw C++ performance vs Python binding performance
- Identify binding overhead accurately
- Validate that both implementations produce equivalent results
- Run the same test scenarios on the same robots with the same metrics

## Proposed Solution

Align the benchmark implementations so that:
1. **Test scenarios are identical**: Both C++ and Python implement:
   - Cold start from zero initialization
   - Cold start from random initialization  
   - Trajectory tracking (sequential waypoints with warm start)
2. **Metrics are consistent**: Both report:
   - Average solve time (µs)
   - Success rate (%)
   - Average iterations per solve
   - Average position error (mm)
   - Average rotation error (deg)
3. **Test datasets are shared**: Both use the same 1000 random test cases per robot
4. **Robot coverage matches**: C++ tests UR5e and variants (ur5e, ur5e+x, ur5e+xy, ur5e+xyz) like Python Tier A
5. **Python retains Tier B**: Python keeps run_tier_b_benchmarks.py for synthetic robot testing

## Scope

### In Scope
- Modify C++ benchmarks to add cold start random and test UR5e variants
- Modify Python benchmarks to add trajectory tracking scenario
- Align metric names and units between implementations
- Use same random seed (42) and sample count (1000) for reproducibility
- Update benchmark documentation to reflect alignment

### Out of Scope
- Changing IK solver algorithms or configurations
- Modifying visualization scripts (they already handle unified format)
- Adding new benchmark types beyond the three scenarios
- Performance optimization of benchmarks themselves

## Success Criteria

1. Both C++ and Python produce benchmark results with identical test scenarios
2. Metrics are reported with same names, units, and precision
3. Running benchmarks on UR5e in C++ and Python produces comparable results (accounting for binding overhead)
4. Documentation clearly explains the aligned benchmark structure
5. `openspec validate align-benchmark-implementations --strict` passes

## Alternatives Considered

1. **Separate benchmarks**: Keep C++ and Python benchmarks independent
   - Rejected: Makes performance comparison and validation difficult
   
2. **Python-only benchmarks**: Remove C++ benchmarks, use only Python
   - Rejected: Need native C++ performance baselines without binding overhead
   
3. **Single implementation**: Generate C++ from Python or vice versa
   - Rejected: Different frameworks (Google Benchmark vs custom runner) make this impractical

## Dependencies

- Existing benchmark infrastructure (benchmark-infrastructure spec)
- UR5e variant URDF files in examples/models/ur5/
- Google Benchmark library (C++)
- Python kinex bindings

## Risks

- **Increased C++ benchmark runtime**: Testing 4 robots instead of 1 will increase build/test time
  - Mitigation: Keep sample count reasonable (1000), provide --filter option
  
- **Breaking existing results**: Historical benchmark data may not be comparable
  - Mitigation: Archive old results, document the change, use semantic versioning

## Related Work

- benchmark-infrastructure spec defines overall structure
- python-bindings spec defines binding requirements
- Existing C++ benchmarks in benchmarks/cpp/
- Existing Python benchmarks in benchmarks/python/
