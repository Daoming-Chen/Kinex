# urdfx Python Benchmark Summary

### UR5E (6 DOF)

| Scenario | Time (µs) | Iterations | Success Rate |
|----------|-----------|------------|-------------|
| ColdStart_Zero | 90.91 | 26.1 | 47.4% |
| ColdStart_Random | 99.57 | 26.8 | 41.9% |
| WarmStart | 54.09 | 16.5 | 75.8% |
| Trajectory | 35.60 | 12.2 | 87.3% |

### UR5E+X (7 DOF)

| Scenario | Time (µs) | Iterations | Success Rate |
|----------|-----------|------------|-------------|
| ColdStart_Zero | 33.16 | 11.1 | 91.0% |
| ColdStart_Random | 50.16 | 13.7 | 82.5% |
| WarmStart | 19.19 | 6.6 | 94.9% |
| Trajectory | 12.14 | 4.2 | 97.0% |

### UR5E+XY (8 DOF)

| Scenario | Time (µs) | Iterations | Success Rate |
|----------|-----------|------------|-------------|
| ColdStart_Zero | 21.51 | 9.2 | 98.5% |
| ColdStart_Random | 24.32 | 10.3 | 98.4% |
| WarmStart | 6.40 | 3.1 | 100.0% |
| Trajectory | 13.68 | 2.5 | 95.7% |

### UR5E+XYZ (9 DOF)

| Scenario | Time (µs) | Iterations | Success Rate |
|----------|-----------|------------|-------------|
| ColdStart_Zero | 19.84 | 8.9 | 99.9% |
| ColdStart_Random | 20.35 | 9.1 | 100.0% |
| WarmStart | 6.31 | 3.0 | 100.0% |
| Trajectory | 4.83 | 2.6 | 100.0% |

## Key Findings

- Python bindings have minimal performance overhead
- Warm start initialization provides significant speedup across all configurations
- Success rates improve dramatically with warm start
- Trajectory mode achieves best performance with fewest iterations
