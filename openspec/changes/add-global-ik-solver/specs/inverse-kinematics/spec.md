## ADDED Requirements

### Requirement: System SHALL provide GlobalIKSolver for parallel multi-start IK solving
The `GlobalIKSolver` SHALL solve inverse kinematics using multiple parallel SQP attempts with different initial configurations to increase success rate and explore solution space.

#### Scenario: Create GlobalIKSolver with RobotModel
**GIVEN** a RobotModel instance and target end-effector link name
**WHEN** the user creates `GlobalIKSolver solver(model, "tool0")`
**THEN** the global solver is initialized for the kinematic chain to "tool0"
**AND** the solver can execute parallel IK solving with configurable seeds and threads

#### Scenario: Solve IK with multiple parallel attempts
**GIVEN** a GlobalIKSolver configured with num_seeds=8 and num_threads=4
**AND** a target pose T_target within robot workspace
**WHEN** the user calls `solver.solve(T_target, solver_config, global_config)`
**THEN** up to 8 parallel SQP solves are executed with different random initial configurations
**AND** the solver returns either (a) the best successful solution or (b) all successful solutions depending on configuration
**AND** the overall success rate exceeds 95% for reachable poses

#### Scenario: Parallel execution respects thread safety
**GIVEN** a GlobalIKSolver instance
**AND** multiple concurrent solve requests from different threads
**WHEN** each thread calls solve() with different target poses
**THEN** all solves execute without data races or corruption
**AND** each thread receives correct results for its target pose
**AND** no exceptions occur due to concurrent access

### Requirement: System SHALL support configurable global solver parameters
The `GlobalSolverConfig` SHALL allow users to control parallel execution behavior, solution selection criteria, and resource limits.

#### Scenario: Configure number of parallel attempts
**GIVEN** a GlobalSolverConfig instance
**WHEN** the user sets `num_seeds=16` and `num_threads=4`
**THEN** the solver launches 16 SQP attempts using 4 concurrent threads
**AND** thread pool automatically schedules work to balance CPU usage

#### Scenario: Select solution return mode
**GIVEN** a GlobalSolverConfig with `return_all_solutions=false`
**WHEN** IK solving completes with 3 successful solutions
**THEN** the solver returns only the best solution (lowest error)
**AND** success status is set to SUCCESS if any solution converged

#### Scenario: Return all unique solutions
**GIVEN** a GlobalSolverConfig with `return_all_solutions=true` and `unique_threshold=1e-3`
**WHEN** IK solving completes with 5 successful solutions where 2 are duplicates within threshold
**THEN** the solver returns 3 unique solutions (duplicates filtered by L2 norm)
**AND** solutions are sorted by quality (error norm ascending)
**AND** success status is set to SUCCESS with solution count metadata

#### Scenario: Configure execution timeout
**GIVEN** a GlobalSolverConfig with `max_time_ms=100`
**WHEN** parallel IK solving would take longer than 100ms total
**THEN** the solver terminates remaining threads and returns current best results
**AND** status indicates MAX_TIME_REACHED with partial results

### Requirement: System SHALL provide high-success-rate robust IK solving
The Robot class SHALL provide `solveRobustIK()` method that returns a single best solution with very high success rate by trying multiple approaches sequentially.

#### Scenario: Robust IK with warm start fallback
**GIVEN** a Robot instance and a target pose with good initial guess q_guess
**WHEN** the user calls `robot.solveRobustIK(target, q_guess)` with default config (num_seeds=8)
**THEN** the system first attempts standard IK with q_guess
**AND** if that fails, automatically tries 8 random seeds in parallel
**AND** returns the single best solution or failure if all attempts fail
**AND** overall success rate exceeds 99% for reachable poses

#### Scenario: Robust IK fails gracefully
**GIVEN** a target pose outside the robot workspace
**WHEN** the user calls `robot.solveRobustIK(target, q_guess)`
**THEN** all parallel attempts fail after max iterations or timeout
**AND** the system returns failure status with best effort solution
**AND** error information from the best attempt is preserved
**AND** no exceptions are thrown

### Requirement: System SHALL provide global multi-solution IK exploration
The Robot class SHALL provide `solveGlobalIK()` method that returns all unique IK solutions for comprehensive configuration space exploration.

#### Scenario: Global IK for multiple solutions
**GIVEN** a Robot instance and a target pose with multiple feasible configurations
**WHEN** the user calls `robot.solveGlobalIK(target, config)` with `return_all_solutions=true`
**THEN** the solver executes parallel attempts and returns all unique solutions
**AND** solutions include elbow-up, elbow-down, and other distinct configurations
**AND** solutions are sorted by solution quality
**AND** metadata includes per-solution convergence information

#### Scenario: Solution deduplication for global IK
**GIVEN** a target pose where different initial guesses converge to similar configurations
**WHEN** `solveGlobalIK()` completes with `unique_threshold=1e-3`
**THEN** solutions within L2 distance threshold of each other are considered duplicates
**AND** only unique solutions are returned to the user
**AND** duplicate solutions are logged at DEBUG level for debugging purposes

### Requirement: System SHALL provide quality-based solution selection
The global solver SHALL select the best solution based on solver convergence metrics when multiple successful solutions are available.

#### Scenario: Best solution selection by error norm
**GIVEN** 3 successful IK solutions with final error norms 1e-4, 5e-5, and 2e-4
**WHEN** robust mode selects single best solution
**THEN** the solution with error norm 5e-5 is selected as it has the smallest error
**AND** the solution is returned regardless of which thread converged first

#### Scenario: Tie-breaking by iteration count
**GIVEN** 2 solutions with identical final error norms but different iteration counts (15 vs 25)
**WHEN** robust mode selects single best solution
**THEN** the solution with 15 iterations is selected as secondary quality criterion
**AND** solution selection is deterministic for identical quality metrics

#### Scenario: Solution quality metrics are exposed
**GIVEN** completed global IK solve with successful solutions
**WHEN** the user queries solution quality information
**THEN** each solution includes final error norm, iterations taken, convergence time, and constraint violations
**AND** this information can be used for offline analysis or visualization

### Requirement: System SHALL provide configuration space sampling
The GlobalIKSolver SHALL generate diverse initial configurations within joint limits to maximize exploration of solution space.

#### Scenario: Random configuration sampling within limits
**GIVEN** a RobotModel with joint limits [-π, π] for all revolute joints
**WHEN** the solver needs random initial configurations
**THEN** joint angles are uniformly sampled within the specified limits
**AND** all sampled configurations respect joint constraints
**AND** sampling uses high-quality random number generation (std::mt19937 + std::random_device)

#### Scenario: Stratified sampling for high-dimensional spaces
**GIVEN** a 7-DOF robot and num_seeds=14 (2x DOF)
**WHEN** generating initial configurations for parallel solving
**THEN** joint limits are stratified into regions and samples distributed across regions
**AND** this ensures better coverage of configuration space than pure uniform sampling
**AND** increases probability of finding distinct solution modes

### Requirement: System SHALL maintain backward compatibility
All existing Robot IK methods SHALL continue to work unchanged with no performance impact when new global functionality is not used.

#### Scenario: Existing IK methods unaffected
**GIVEN** existing code using `robot.inverseKinematics()` or `robot.solveIK()`
**WHEN** the new GlobalIKSolver functionality is added
**THEN** all existing method signatures remain identical
**AND** existing IK performance and behavior are unchanged
**AND** no additional memory or CPU overhead occurs when not using global methods

#### Scenario: Coexisting IK interfaces
**GIVEN** a Robot instance
**WHEN** the user calls both `robot.solveIK()` and `robot.solveRobustIK()` in the same program
**THEN** both methods execute correctly with their respective behaviors
**AND** internal state is properly managed between different solver instances
**AND** results are consistent with expected semantics for each method