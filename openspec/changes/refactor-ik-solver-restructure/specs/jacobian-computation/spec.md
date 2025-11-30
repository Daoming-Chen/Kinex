## MODIFIED Requirements

### Requirement: JacobianCalculator SHALL accept RobotState and avoid recomputing transforms
The JacobianCalculator SHALL support an overloaded API to accept a precomputed `RobotState` and compute the Jacobian using the transforms in this state rather than recomputing transforms internally. This enables FK-Jacobian fusion and avoids repeated transform computations between FK and Jacobian calls.

#### Scenario: Compute Jacobian using provided RobotState
**GIVEN** a `RobotState` from `ForwardKinematics::computeRobotState(q, state)`
**WHEN** the user calls `JacobianCalculator::computeFromRobotState(state, J)`
**THEN** the Jacobian computation uses the transforms stored in `state` and performs only axis extraction, cross products, and minimal algebra
**AND** `JacobianCalculator` returns identical J to `compute(q)` within numerical tolerance

### Requirement: JacobianCalculator SHALL enforce zero-allocation during repeated calls
The JacobianCalculator SHALL use pre-allocated storage for intermediate values and must not allocate heap memory on repeated `compute` calls.

#### Scenario: Repeated Jacobian compute has no heap allocations
**GIVEN** a JacobianCalculator pre-initialized for a robot
**WHEN** `compute(q)` is invoked 1000 times
**THEN** no heap allocations occur in compute() and average per-call time is < 0.5 ms (6-DOF baseline)
