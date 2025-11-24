## Context

The urdfx project currently has an architecturally inconsistent benchmark organization where core benchmarking infrastructure (URDF generation, dataset creation, visualization) is located in `bindings/python/benchmarks/`, alongside the Python bindings. This creates confusion about:

1. **What is binding-specific vs project-wide**: Core tools are co-located with binding code
2. **Where to find benchmarks**: Natural first check is root `benchmarks/`, but key tools are elsewhere
3. **Purpose of bindings benchmarks**: Should measure binding overhead, not implement comprehensive performance analysis

Additionally, there's duplication between Python and C++ benchmarks for the same test scenarios, but without clear delineation of which tests are measuring what aspect of performance.

**Stakeholders:**
- Developers: Need clear mental model of where to add/find benchmarks
- CI/CD: Needs consistent paths for benchmark execution
- Users: Need to understand performance characteristics of both C++ core and Python bindings

## Goals / Non-Goals

**Goals:**
- Single authoritative location (`benchmarks/`) for all core performance evaluation
- Clear separation: core library benchmarks vs binding overhead benchmarks
- Shared infrastructure (generators, oracles, visualizers) accessible to all benchmark types
- Consistent directory structure: `benchmarks/{cpp,python,tools,results}/`
- Binding directories contain only binding-specific overhead tests

**Non-Goals:**
- Changing benchmark implementations or adding new benchmarks (pure refactor)
- Modifying public APIs of any Python modules
- Changing benchmark result formats or metrics
- Improving benchmark performance or adding new test scenarios
- Consolidating Python and C++ benchmarks into single implementation

## Decisions

### Decision 1: Three-tier benchmark organization

**Rationale:**
- `benchmarks/cpp/`: Native C++ performance benchmarks (Google Benchmark)
- `benchmarks/python/`: Python implementations measuring Python binding performance
- `benchmarks/tools/`: Shared utilities (generators, oracles, visualizers)

**Why:** Clearly separates language-specific implementations while sharing common infrastructure. Makes it obvious where to add new benchmarks based on what you're measuring.

**Alternatives considered:**
- Keep everything in `bindings/python/benchmarks/`: Rejected because it's architecturally confusing
- Merge Python and C++ into single test suite: Rejected because they serve different purposes
- Keep tools distributed: Rejected because it causes import complexity and duplication

### Decision 2: Minimal binding overhead tests

**Rationale:**
`bindings/python/benchmarks/` should only contain tests that directly measure the overhead of the binding layer itself:
- FK/IK/Jacobian computation time (Python vs C++ comparison)
- Data conversion overhead (NumPy array ↔ Eigen::VectorXd)
- GIL impact in multi-threaded scenarios

**Why:** The purpose of binding benchmarks is to answer "how much slower is the Python API compared to C++?" not "how fast is our IK solver?" (that's what core benchmarks answer).

**Target threshold:** <10% overhead for computational operations, documented in binding README.

**Alternatives considered:**
- Keep comprehensive benchmarks in bindings: Rejected because it duplicates core benchmark logic
- No binding benchmarks at all: Rejected because users need to know the binding cost

### Decision 3: Move entire runners, not just utilities

**Rationale:**
Move all Python benchmark runners (`run_all_benchmarks.py`, `run_tier_a_benchmarks.py`, etc.) to `benchmarks/python/`, not just the utilities.

**Why:** These runners orchestrate comprehensive performance evaluation of the core library (via Python bindings), not measurement of binding overhead. They belong with core benchmarks.

**Binding benchmarks** should have their own simple runner that produces overhead comparison reports.

### Decision 4: Preserve tools as separate package

**Rationale:**
`benchmarks/tools/` becomes a Python package that can be imported by both C++ benchmark harnesses (via dataset generation) and Python benchmark runners.

Structure:
```
benchmarks/tools/
├── __init__.py           # Exports: MixedChainGenerator, FKOracle, JointSampler
├── urdf_generator.py     # MixedChainGenerator
├── oracle.py             # FKOracle, JointSampler
├── visualize.py          # Combined visualization functions
└── dataset_formats.py    # Dataset I/O utilities (if needed)
```

**Why:** Allows clean imports like `from benchmarks.tools import MixedChainGenerator` and maintains tool cohesion.

## Risks / Trade-offs

### Risk: Import path breakage

**Description:** External scripts or documentation might reference old paths.

**Mitigation:**
- Comprehensive grep for all references before migration
- Add migration guide to PR description and release notes
- Consider temporary symlinks during transition (dev only)
- Update all CI/CD, docs, and examples in same PR

### Risk: Python package path issues

**Description:** Moving files might break imports if `benchmarks/` isn't in `sys.path`.

**Mitigation:**
- Add `benchmarks/` to `sys.path` in runner scripts
- Document import pattern in `benchmarks/README.md`
- Add `__init__.py` files at all package levels
- Test imports explicitly in validation step

### Trade-off: More directories

**Description:** Going from 2 locations (`benchmarks/`, `bindings/python/benchmarks/`) to 4 sub-locations (`benchmarks/{cpp,python,tools}/`, `bindings/python/benchmarks/`).

**Justification:** Increased structure reduces cognitive load by making purpose explicit. Each directory has a single clear responsibility.

## Migration Plan

### Phase 1: Create new structure (non-breaking)
1. Create `benchmarks/{cpp,python,tools}/` directories
2. Copy (don't move yet) files to new locations
3. Update CMake to support both old and new paths
4. Test that benchmarks run from new locations

### Phase 2: Update references
1. Update all imports in moved Python files
2. Update CMakeLists.txt to use new paths
3. Update CI/CD workflows
4. Update documentation

### Phase 3: Remove old locations
1. Delete files from `bindings/python/benchmarks/` (except new overhead tests)
2. Remove old CMake configuration
3. Verify no references remain to old paths

### Rollback plan
If issues discovered after merge:
1. Git revert the migration commit
2. All old paths still work (nothing external depends on new structure yet)
3. Fix issues in separate PR and re-attempt migration

## Open Questions

1. **Should we keep backward compatibility imports in `bindings/python/benchmarks/__init__.py`?**
   - Proposal: No, clean break. Document migration.
   - Rationale: Internal tool, no external users to break

2. **Should C++ benchmarks also use generated datasets from Python tools?**
   - Current: `mixed_ik_benchmarks.cpp` loads binary datasets
   - Proposal: Yes, use `benchmarks/tools/` to generate datasets for both
   - Action: Verify this is already the case

3. **Should we rename `run_all_benchmarks.py` to something more specific?**
   - Current: Ambiguous what "all" means
   - Proposal: `run_core_benchmarks.py` to distinguish from binding overhead
   - Decision: Defer to implementation review
