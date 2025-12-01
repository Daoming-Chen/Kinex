#!/usr/bin/env python3
"""
Run benchmarks on Tier A robots (UR5e configurations with 6-10 DOF).
Tests new solve_robust_ik and solve_global_ik methods with real-time generated test cases.
"""

import os
import sys
import time
import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# Add current directory to path (benchmarks/)
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

# Add Python bindings to path
project_root = os.path.abspath(os.path.join(current_dir, ".."))
python_bindings_path = os.path.join(project_root, "bindings/python")
if python_bindings_path not in sys.path:
    sys.path.insert(0, python_bindings_path)

try:
    import kinex
except ImportError as e:
    print(f"Error: kinex module not found. Please build Python bindings first.")
    print(f"Error details: {e}")
    print(f"Tried path: {python_bindings_path}")
    sys.exit(1)

from oracle import FKOracle, JointSampler

class RestrictedJointSampler:
    """
    Samples joint configurations within [-pi, pi] to avoid extreme winding,
    matching the updated C++ benchmark logic.
    """
    def __init__(self, robot, rng):
        self.dof = robot.dof
        self.rng = rng
        try:
            limits = robot.get_joint_limits()
            if not isinstance(limits, np.ndarray):
                limits = np.array(limits)
        except:
            # Fallback
            limits = np.array([[-np.pi, np.pi]] * self.dof)

        self.lower = np.maximum(limits[:, 0], -np.pi)
        self.upper = np.minimum(limits[:, 1], np.pi)

        # If limits are weird (low > high), fallback to limits
        mask = self.lower > self.upper
        self.lower[mask] = limits[mask, 0]
        self.upper[mask] = limits[mask, 1]

    def sample(self, n_samples=1):
        return self.rng.uniform(self.lower, self.upper, size=(n_samples, self.dof))

def run_benchmark(robot_name, urdf_path, output_dir, num_samples=1000, seed=42):
    """Run IK benchmark with real-time generated test cases."""
    print(f"\n{'='*70}")
    print(f"Benchmarking {robot_name}")
    print(f"{'='*70}")

    # Load robot using the new unified API
    print(f"Loading robot from {urdf_path}...")
    robot = kinex.Robot.from_urdf(urdf_path, "wrist_3_link")  # Using wrist_3_link as end effector
    dof = robot.dof
    print(f"  DOF: {dof}")
    print(f"  Test samples: {num_samples}")

    # Setup FK oracle and joint sampler
    oracle = FKOracle(robot.model, robot.end_link)
    sampler = RestrictedJointSampler(robot, rng=np.random.RandomState(seed))

    print(f"  End-effector: {robot.end_link}")

    # Configure solver tolerance
    robot.set_ik_tolerance(5e-4)

    # Configure global solver
    global_config = kinex.GlobalSolverConfig()
    global_config.num_seeds = 20  # Number of random restarts for robust/global IK
    global_config.num_threads = 4
    global_config.max_time_ms = 5000  # 5 second timeout
    global_config.return_all_solutions = True
    global_config.unique_threshold = 1e-3
    robot.set_global_solver_config(global_config)

    print(f"  Global solver config: {global_config.num_seeds} seeds, {global_config.max_time_ms}ms timeout")

    # Generate test samples (sample random joint configs once)
    print("\n  Generating test cases...")
    q_samples = sampler.sample(num_samples)

    # Run benchmarks with different test scenarios
    results = {
        "robot_name": robot_name,
        "dof": dof,
        "total_samples": num_samples,
        "seed": seed,
        "robust_ik": {},
        "global_ik": {}
    }

    # Test both solve_robust_ik and solve_global_ik
    for method_name in ["robust_ik", "global_ik"]:
        print(f"\n  Testing {method_name}...")

        successes = 0
        failures = 0
        total_time = 0.0
        pos_errors = []
        rot_errors = []
        num_solutions_list = []  # For global_ik to track number of solutions found

        for i, q_target in enumerate(q_samples):
            # 1. Compute target pose via FK (very fast)
            target_pos, target_rot = oracle.compute_pose(q_target)

            # Convert to Transform
            rpy = R.from_matrix(target_rot).as_euler('xyz', degrees=False)
            target_transform = kinex.Transform.from_position_rpy(target_pos, rpy)

            # 2. Choose initial guess (use zero for consistency)
            q_init = np.zeros(dof)

            # 3. Solve IK (timed)
            start_time = time.perf_counter()

            if method_name == "robust_ik":
                # solve_robust_ik returns (solution, status) pair
                solution, status = robot.solve_robust_ik(target_transform, q_init)
                # Only count as success if status.converged is True
                if status.converged:
                    solutions = [solution]
                else:
                    solutions = []  # Failure case
                statuses = [status]
            else:  # global_ik
                # solve_global_ik returns list of solutions
                solutions = robot.solve_global_ik(target_transform, q_init)
                # For global IK, we consider it successful if at least one solution is found
                # Create dummy statuses for each solution
                statuses = []
                for sol in solutions:
                    status = kinex.SolverStatus()
                    status.converged = True
                    status.iterations = 1  # Placeholder
                    statuses.append(status)

            solve_time = time.perf_counter() - start_time
            total_time += solve_time

            # 4. Verify solutions
            if len(solutions) > 0:
                successes += 1
                num_solutions_list.append(len(solutions))

                # Use the first solution for error calculation (consistent with robust IK)
                solution = solutions[0]

                # Compute achieved pose
                achieved_pos, achieved_rot = oracle.compute_pose(solution)

                # Position error
                pos_error = np.linalg.norm(achieved_pos - target_pos)
                pos_errors.append(pos_error)

                # Rotation error (angular distance in radians)
                q_target_quat = R.from_matrix(target_rot).as_quat()
                q_achieved_quat = R.from_matrix(achieved_rot).as_quat()
                dot_product = np.abs(np.dot(q_target_quat, q_achieved_quat))
                dot_product = np.clip(dot_product, -1.0, 1.0)
                rot_error = 2 * np.arccos(dot_product)  # Angular distance in radians
                rot_errors.append(rot_error)
            else:
                failures += 1
                num_solutions_list.append(0)

        # Compute statistics
        total_cases = len(q_samples)
        success_rate = successes / total_cases * 100
        avg_time = total_time / total_cases * 1_000_000  # Convert to µs
        avg_pos_error = np.mean(pos_errors) * 1000 if pos_errors else 0  # Convert to mm
        avg_rot_error_rad = np.mean(rot_errors) if rot_errors else 0
        avg_rot_error_deg = avg_rot_error_rad * 180.0 / np.pi  # Convert to degrees
        avg_num_solutions = np.mean(num_solutions_list) if num_solutions_list else 0

        results[method_name] = {
            "cases": total_cases,
            "successes": successes,
            "failures": failures,
            "success_rate": success_rate,
            "avg_time_us": avg_time,
            "avg_pos_error_mm": avg_pos_error,
            "avg_rot_error_deg": avg_rot_error_deg,
            "avg_num_solutions": avg_num_solutions
        }

        print(f"    Cases: {total_cases}")
        print(f"    Success rate: {success_rate:.1f}%")
        print(f"    Avg time: {avg_time:.1f} us")
        print(f"    Avg position error: {avg_pos_error:.6f} mm")
        print(f"    Avg rotation error: {avg_rot_error_deg:.6f} deg")
        if method_name == "global_ik":
            print(f"    Avg solutions per case: {avg_num_solutions:.2f}")

    # Save results in Google Benchmark JSON format
    import socket
    import platform
    from datetime import datetime

    benchmark_results = []

    # Helper to create benchmark entry
    def make_benchmark_entry(name, robot_name, case_data, num_samples):
        return {
            "name": f"BM_IK_{name}/{robot_name}",
            "run_name": f"BM_IK_{name}/{robot_name}",
            "run_type": "iteration",
            "repetitions": 1,
            "repetition_index": 0,
            "threads": 1,
            "iterations": num_samples,
            "real_time": case_data["avg_time_us"],
            "cpu_time": case_data["avg_time_us"],
            "time_unit": "us",
            "avg_position_error_mm": case_data["avg_pos_error_mm"],
            "avg_rotation_error_deg": case_data["avg_rot_error_deg"],
            "success_rate": case_data["success_rate"],
            "avg_num_solutions": case_data.get("avg_num_solutions", 1.0)
        }

    benchmark_results.append(make_benchmark_entry("RobustIK", robot_name,
                                                   results["robust_ik"],
                                                   results["robust_ik"]["cases"]))
    benchmark_results.append(make_benchmark_entry("GlobalIK", robot_name,
                                                   results["global_ik"],
                                                   results["global_ik"]["cases"]))

    # Create full Google Benchmark JSON structure
    output_data = {
        "context": {
            "date": datetime.now().strftime("%Y-%m-%dT%H:%M:%S%z"),
            "host_name": socket.gethostname(),
            "executable": sys.executable,
            "num_cpus": os.cpu_count() or 1,
            "library_build_type": "release",
            "robot_name": robot_name,
            "dof": dof,
            "seed": seed
        },
        "benchmarks": benchmark_results
    }

    output_path = os.path.join(output_dir, f"{robot_name}_robust_global_results.json")
    with open(output_path, 'w') as f:
        json.dump(output_data, f, indent=2)
    print(f"\n  Results saved to {output_path}")

    return results

def main():
    import argparse

    parser = argparse.ArgumentParser(description="Robust & Global IK Benchmark Runner")
    parser.add_argument("--samples", "-n", type=int, default=500,
                       help="Number of test samples per robot (default: 500)")
    parser.add_argument("--seed", "-s", type=int, default=42,
                       help="Random seed for reproducibility (default: 42)")
    parser.add_argument("--output", "-o", type=str, default=None,
                       help="Output directory for results")
    parser.add_argument("--visualize", "-v", action="store_true",
                       help="Generate visualization plots")
    args = parser.parse_args()

    # Navigate from benchmarks/ directory to project root
    project_root = os.path.abspath(os.path.join(current_dir, ".."))

    models_dir = os.path.join(project_root, "examples/models/ur5")
    output_dir = args.output or os.path.join(project_root, "benchmarks/results")

    os.makedirs(output_dir, exist_ok=True)

    print("MixKinBench Robust & Global IK Benchmark Runner")
    print(f"Project root: {project_root}")
    print(f"Models directory: {models_dir}")
    print(f"Output directory: {output_dir}")
    print(f"Samples per robot: {args.samples}")
    print(f"Random seed: {args.seed}")

    # Configs: (robot_name, urdf_filename)
    configs = [
        ("ur5e", "ur5e.urdf"),
        ("ur5e+x", "ur5e+x.urdf"),
        ("ur5e+xy", "ur5e+xy.urdf"),
        ("ur5e+xyz", "ur5e+xyz.urdf"),
    ]

    all_results = []

    for robot_name, urdf_file in configs:
        urdf_path = os.path.join(models_dir, urdf_file)

        if not os.path.exists(urdf_path):
            print(f"\nWARNING: URDF not found: {urdf_path}")
            continue

        try:
            result = run_benchmark(robot_name, urdf_path, output_dir,
                                 num_samples=args.samples, seed=args.seed)
            all_results.append(result)
        except Exception as e:
            print(f"\nERROR benchmarking {robot_name}: {e}")
            import traceback
            traceback.print_exc()

    # Generate summary report
    print(f"\n{'='*80}")
    print("BENCHMARK SUMMARY")
    print(f"{'='*80}")
    print(f"\n{'Robot':<15} {'DOF':<5} {'Robust':<12} {'Global':<12} {'Solutions':<10}")
    print("-" * 80)

    for result in all_results:
        robot = result['robot_name']
        dof = result['dof']
        robust_sr = result['robust_ik'].get('success_rate', 0)
        global_sr = result['global_ik'].get('success_rate', 0)
        avg_solutions = result['global_ik'].get('avg_num_solutions', 0)

        print(f"{robot:<15} {dof:<5} {robust_sr:>5.1f}% {global_sr:>10.1f}% {avg_solutions:>8.2f}")

    print("\n" + "="*80)
    print("Benchmark complete!")

    # Generate visualization if requested
    if args.visualize and all_results:
        print("\nGenerating visualizations...")
        generate_visualizations(all_results, output_dir)

def generate_visualizations(results, output_dir):
    """Generate visualization plots for benchmark results."""
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    # Set up the figure with subplots
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

    robots = [r['robot_name'] for r in results]
    dofs = [r['dof'] for r in results]

    # 1. Success Rate Comparison
    robust_success = [r['robust_ik']['success_rate'] for r in results]
    global_success = [r['global_ik']['success_rate'] for r in results]

    x = np.arange(len(robots))
    width = 0.35

    ax1.bar(x - width/2, robust_success, width, label='Robust IK', alpha=0.8, color='skyblue')
    ax1.bar(x + width/2, global_success, width, label='Global IK', alpha=0.8, color='lightcoral')
    ax1.set_xlabel('Robot Configuration')
    ax1.set_ylabel('Success Rate (%)')
    ax1.set_title('Success Rate Comparison')
    ax1.set_xticks(x)
    ax1.set_xticklabels(robots)
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2. Average Computation Time
    robust_time = [r['robust_ik']['avg_time_us'] for r in results]
    global_time = [r['global_ik']['avg_time_us'] for r in results]

    ax2.bar(x - width/2, robust_time, width, label='Robust IK', alpha=0.8, color='skyblue')
    ax2.bar(x + width/2, global_time, width, label='Global IK', alpha=0.8, color='lightcoral')
    ax2.set_xlabel('Robot Configuration')
    ax2.set_ylabel('Average Time (μs)')
    ax2.set_title('Computation Time Comparison')
    ax2.set_xticks(x)
    ax2.set_xticklabels(robots)
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. Position Error
    robust_pos_error = [r['robust_ik']['avg_pos_error_mm'] for r in results]
    global_pos_error = [r['global_ik']['avg_pos_error_mm'] for r in results]

    ax3.bar(x - width/2, robust_pos_error, width, label='Robust IK', alpha=0.8, color='skyblue')
    ax3.bar(x + width/2, global_pos_error, width, label='Global IK', alpha=0.8, color='lightcoral')
    ax3.set_xlabel('Robot Configuration')
    ax3.set_ylabel('Average Position Error (mm)')
    ax3.set_title('Position Error Comparison')
    ax3.set_xticks(x)
    ax3.set_xticklabels(robots)
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. Average Number of Solutions (Global IK only)
    avg_solutions = [r['global_ik']['avg_num_solutions'] for r in results]

    bars = ax4.bar(robots, avg_solutions, alpha=0.8, color='lightcoral')
    ax4.set_xlabel('Robot Configuration')
    ax4.set_ylabel('Average Number of Solutions')
    ax4.set_title('Global IK: Average Solutions per Test Case')
    ax4.grid(True, alpha=0.3)

    # Add value labels on bars
    for bar, sol in zip(bars, avg_solutions):
        height = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                f'{sol:.2f}', ha='center', va='bottom')

    plt.tight_layout()

    # Save the plot
    plot_path = os.path.join(output_dir, "robust_global_ik_benchmark.png")
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    print(f"Visualization saved to {plot_path}")

    # Also save as PDF for high quality
    pdf_path = os.path.join(output_dir, "robust_global_ik_benchmark.pdf")
    plt.savefig(pdf_path, bbox_inches='tight')
    print(f"High-quality PDF saved to {pdf_path}")

    plt.close()

if __name__ == "__main__":
    main()