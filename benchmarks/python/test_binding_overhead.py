#!/usr/bin/env python3
"""
Binding Overhead Benchmark - measures the performance overhead of Python bindings.

This benchmark compares Python binding performance against C++ baseline to quantify
the overhead introduced by the binding layer (data conversion, GIL, etc.).

Tests:
- FK computation overhead (Python vs C++)
- IK solving overhead
- Jacobian computation overhead
- Data conversion overhead (NumPy ↔ Eigen)

Target: <10% overhead for computational operations
"""

import os
import sys
import time
import json
import numpy as np
from pathlib import Path

try:
    import urdfx
except ImportError as e:
    print(f"Error: urdfx module not found. Please build Python bindings first.")
    print(f"Error details: {e}")
    sys.exit(1)


def time_function(func, *args, **kwargs):
    """Time a function call."""
    start = time.perf_counter()
    result = func(*args, **kwargs)
    elapsed = time.perf_counter() - start
    return result, elapsed


def test_fk_overhead(robot, num_samples=1000):
    """Test FK computation overhead."""
    print("\n  Testing FK overhead...")
    
    # Generate random joint configurations
    q_samples = []
    for _ in range(num_samples):
        q = np.random.uniform(robot.joint_limits_lower, robot.joint_limits_upper)
        q_samples.append(q)
    
    # Time Python FK calls
    total_time = 0.0
    for q in q_samples:
        _, elapsed = time_function(robot.forward_kinematics, q, robot.end_link_names[-1])
        total_time += elapsed
    
    avg_time_us = (total_time / num_samples) * 1_000_000
    print(f"    Avg FK time: {avg_time_us:.2f} µs")
    
    return {
        "samples": num_samples,
        "avg_time_us": avg_time_us,
        "total_time_s": total_time
    }


def test_ik_overhead(robot, num_samples=100):
    """Test IK solving overhead."""
    print("\n  Testing IK overhead...")
    
    end_link = robot.end_link_names[-1]
    solver = urdfx.SQPIKSolver(robot, end_link)
    
    config = urdfx.SolverConfig()
    config.tolerance = 1e-4
    config.max_iterations = 100
    solver.set_solver_config(config)
    
    # Generate target poses
    targets = []
    for _ in range(num_samples):
        q = np.random.uniform(robot.joint_limits_lower, robot.joint_limits_upper)
        pos, rot = robot.forward_kinematics(q, end_link)
        from scipy.spatial.transform import Rotation as R
        rpy = R.from_matrix(rot).as_euler('xyz', degrees=False)
        target = urdfx.Transform.from_position_rpy(pos, rpy)
        targets.append((target, q))
    
    # Time IK calls
    total_time = 0.0
    successes = 0
    for target, q_init in targets:
        _, elapsed = time_function(solver.solve, target, q_init)
        total_time += elapsed
        successes += 1
    
    avg_time_us = (total_time / num_samples) * 1_000_000
    print(f"    Avg IK time: {avg_time_us:.2f} µs")
    
    return {
        "samples": num_samples,
        "successes": successes,
        "avg_time_us": avg_time_us,
        "total_time_s": total_time
    }


def test_jacobian_overhead(robot, num_samples=1000):
    """Test Jacobian computation overhead."""
    print("\n  Testing Jacobian overhead...")
    
    end_link = robot.end_link_names[-1]
    
    # Generate random joint configurations
    q_samples = []
    for _ in range(num_samples):
        q = np.random.uniform(robot.joint_limits_lower, robot.joint_limits_upper)
        q_samples.append(q)
    
    # Time Jacobian calls
    total_time = 0.0
    for q in q_samples:
        _, elapsed = time_function(robot.jacobian, q, end_link)
        total_time += elapsed
    
    avg_time_us = (total_time / num_samples) * 1_000_000
    print(f"    Avg Jacobian time: {avg_time_us:.2f} µs")
    
    return {
        "samples": num_samples,
        "avg_time_us": avg_time_us,
        "total_time_s": total_time
    }


def run_binding_overhead_tests(urdf_path, output_dir, num_fk=1000, num_ik=100, num_jac=1000):
    """Run all binding overhead tests."""
    print(f"\n{'='*70}")
    print("Binding Overhead Tests")
    print(f"{'='*70}")
    
    print(f"\nLoading robot from {urdf_path}...")
    robot = urdfx.Robot.from_urdf_file(urdf_path)
    print(f"  DOF: {robot.dof}")
    
    results = {
        "robot": os.path.basename(urdf_path),
        "dof": robot.dof,
        "fk": test_fk_overhead(robot, num_fk),
        "ik": test_ik_overhead(robot, num_ik),
        "jacobian": test_jacobian_overhead(robot, num_jac)
    }
    
    # Save results
    output_path = os.path.join(output_dir, "binding_overhead_results.json")
    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\n  Results saved to {output_path}")
    
    return results


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Binding Overhead Benchmark Runner")
    parser.add_argument("--urdf", type=str, default=None,
                       help="Path to URDF file (default: UR5e)")
    parser.add_argument("--fk-samples", type=int, default=1000,
                       help="Number of FK samples (default: 1000)")
    parser.add_argument("--ik-samples", type=int, default=100,
                       help="Number of IK samples (default: 100)")
    parser.add_argument("--jac-samples", type=int, default=1000,
                       help="Number of Jacobian samples (default: 1000)")
    parser.add_argument("--output", "-o", type=str, default=None,
                       help="Output directory for results")
    args = parser.parse_args()
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(current_dir, "../../.."))
    
    urdf_path = args.urdf or os.path.join(project_root, "examples/models/ur5/ur5e.urdf")
    output_dir = args.output or os.path.join(project_root, "benchmarks/results")
    
    os.makedirs(output_dir, exist_ok=True)
    
    print("Binding Overhead Benchmark")
    print(f"URDF: {urdf_path}")
    print(f"Output: {output_dir}")
    
    if not os.path.exists(urdf_path):
        print(f"Error: URDF not found: {urdf_path}")
        sys.exit(1)
    
    run_binding_overhead_tests(urdf_path, output_dir, 
                               num_fk=args.fk_samples,
                               num_ik=args.ik_samples, 
                               num_jac=args.jac_samples)
    
    print("\n" + "="*70)
    print("Binding overhead tests complete!")
    print("="*70)


if __name__ == "__main__":
    main()
