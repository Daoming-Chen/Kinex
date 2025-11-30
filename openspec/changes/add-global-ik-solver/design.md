## Design: Global IK Solver Architecture

### Context
The current `SQPIKSolver` provides single-start IK solving with ~60-80% success rate. Users need:
1. **Robust IK**: Highly reliable single-solution for production robotics
2. **Global IK**: Complete solution set exploration for path planning and optimization

Current limitations: single-threaded execution, no parallel exploration, basic random sampling only in tests.

### Goals / Non-Goals
**Goals:**
- Increase IK success rate from 60-80% to >99% for robust mode
- Provide comprehensive solution space exploration for global mode
- Maintain zero-impact on existing API and performance
- Enable easy configuration between robust and global modes

**Non-Goals:**
- Complete workspace analysis (beyond scope)
- Real-time guaranteed performance bounds
- Advanced path planning algorithms (RRT, PRM, etc.)

### Decisions

#### Decision 1: Parallel Multi-Start Architecture
**What**: Use `std::async` with thread pool for parallel SQP solves with different random seeds
**Why**:
- Leverages existing thread-safe `SQPIKSolver` implementation
- Minimal code changes vs building from scratch
- `std::async` provides portable parallelism without external dependencies
- Race-style execution matches "any success = overall success" semantics

**Alternatives considered:**
- Custom thread pool implementation: More complex, unnecessary overhead
- OpenMP: Additional dependency, less flexible execution control
- CUDA/GPU acceleration: Overkill, not portable to all targets

#### Decision 2: Two-Mode Configuration Strategy
**What**: Single `GlobalIKSolver` with `return_all_solutions` flag controlling mode
**Why**:
- Shared core implementation reduces code duplication
- Clear semantic distinction between robust (single best) and global (all solutions)
- Easy future extension to additional strategies (deterministic sampling, etc.)

**Trade-offs:**
- Slightly more complex API vs separate classes
- Unified implementation requires careful state management
- Benefits: code reuse, consistent configuration, easier maintenance

#### Decision 3: Quality-Based Solution Selection
**What**: Select best solution using `final_error_norm` primary, `iterations` secondary criteria
**Why**:
- Error norm directly measures solution accuracy
- Iterations serve as tiebreaker for numerical efficiency
- Matches user expectation of "best" solution in robust mode

**Implementation**: Sort solutions by `(error_norm, iterations)` pair and return first element

### Risks / Trade-offs

#### Risk 1: Thread Contention and Resource Usage
**Risk**: Parallel execution may cause memory bandwidth contention, high CPU usage
**Mitigation**:
- Configurable `num_threads` parameter (default = std::thread::hardware_concurrency)
- `max_time_ms` timeout to prevent excessive execution
- Per-thread `SQPIKSolver` instances avoid shared state conflicts

#### Risk 2: Random Seed Quality and Distribution
**Risk**: Poor random seed generation may lead to clustered initial configurations
**Mitigation**:
- Use high-quality `std::mt19937` with `std::random_device` seeding
- Stratified sampling across joint limits when `num_seeds` > joint DOF
- Configurable `unique_threshold` for solution deduplication

#### Risk 3: Solution Quality Inconsistency
**Risk**: Different initial guesses may converge to different local minima with varying quality
**Mitigation**:
- Best-solution selection based on solver metrics, not just first success
- Configurable quality weighting (position vs orientation error)
- Fallback to warm-start when user provides good initial guess

#### Trade-off 1: Memory vs Speed
**Trade-off**: Storing all solutions for global mode requires more memory
**Decision**: Accept memory cost for functionality; solutions are small (joint angles only)
**Impact**: Negligible for typical use (<100 solutions × ~12 DOF × 8 bytes ≈ 10KB)

#### Trade-off 2: Complexity vs Usability
**Trade-off**: More configuration options increase API complexity
**Decision**: Provide sensible defaults, expose advanced options through configuration struct
**Impact**: Beginners can use defaults, experts can fine-tune parameters

### Migration Plan

#### Phase 1: Core Implementation (Weeks 1-2)
1. Implement `GlobalIKSolver` with basic parallel execution
2. Add `GlobalSolverConfig` with essential parameters only
3. Implement solution aggregation and selection logic
4. Add comprehensive unit tests

#### Phase 2: Integration (Week 3)
1. Add `Robot::solveRobustIK()` and `Robot::solveGlobalIK()` methods
2. Update build system and headers
3. Add integration tests with existing `Robot` functionality
4. Verify no regression in existing API

#### Phase 3: Validation (Week 4)
1. Implement success rate benchmark (1000 random poses)
2. Performance profiling and optimization
3. Memory usage validation
4. Documentation and examples

#### Phase 4: Polish (Week 5)
1. Code review and refinement
2. Additional edge case handling
3. Final validation and CI/CD integration
4. User documentation and migration guide

### Open Questions
1. **Timeout Behavior**: Should timeout apply to total solve time or per-thread time limit?
2. **Solution Deduplication**: What is optimal `unique_threshold` for different robot types?
3. **Seed Strategy**: Should we incorporate deterministic seeds for reproducible results?
4. **Error Metrics**: Should position and orientation errors be weighted differently?
5. **Resource Limits**: Should we add memory or CPU usage caps for embedded systems?