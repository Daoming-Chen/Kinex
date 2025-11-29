## MODIFIED Requirements

### Requirement: ForwardKinematics SHALL provide `RobotState` and compute FK efficiently
The ForwardKinematics class SHALL provide a `computeRobotState(q, RobotState&)` API that computes the per-link transforms and pre-allocates all necessary transforms so FK/Jacobian fusion is possible. ForwardKinematics SHALL avoid heap allocations during repeated calls.

#### Scenario: Provide RobotState via FK
**GIVEN** a `ForwardKinematics` instance
**WHEN** the user calls `computeRobotState(q, state)`
**THEN** `state` is populated with per-link transforms and `compute` returns with no heap allocation in the hot path

### Requirement: FK and Jacobian fusion via `computeFKAndJacobian` overload
The ForwardKinematics or a combined helper API SHALL provide a `computeFKAndJacobian(q, RobotState&, J)` that returns both the end-effector pose and Jacobian using shared transforms without recomputation.

#### Scenario: Combined FK & Jacobian returns same values as separate calls
**GIVEN** a `ForwardKinematics` instance and `JacobianCalculator` instance
**WHEN** the user calls `computeFKAndJacobian(q, state, J)`, collecting both pose and Jacobian
**THEN** results match `compute(q)` and `JacobianCalculator::compute(q)` within numerical tolerance

### Requirement: FK SHALL allow `Robot::cloneForWorker` and fast per-thread state
The `Robot` class SHALL expose a `cloneForWorker()` or factory method suitable for TLS copying so that `ForwardKinematics` objects using the clone have fast, independent per-thread runtime state and minimal memory overhead.

#### Scenario: Clone for worker duplicates runtime state only
**GIVEN** a `Robot model` and `ForwardKinematics fk`
**WHEN** the user calls `auto worker_robot = robot.cloneForWorker()`
**THEN** `worker_robot` shares immutable model internals and contains independent FK and Jacobian caches
**AND** `worker_robot` uses less memory than a full deep clone