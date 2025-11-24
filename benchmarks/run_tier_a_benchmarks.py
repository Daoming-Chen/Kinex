#!/usr/bin/env python3
"""
Run benchmarks on Tier A robots (UR5e configurations with 6-10 DOF).
Tests IK solver with real-time generated test cases.
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
    
    # Load robot
    print(f"Loading robot from {urdf_path}...")
    robot = kinex.Robot.from_urdf_file(urdf_path)
    dof = robot.dof
    print(f"  DOF: {dof}")
    print(f"  Test samples: {num_samples}")
    
    # Setup FK oracle and joint sampler
    oracle = FKOracle(robot)
    sampler = RestrictedJointSampler(robot, rng=np.random.RandomState(seed))
    end_link = oracle.end_link
    
    print(f"  End-effector: {end_link}")
    
    # Create IK solver
    print("Creating IK solver...")
    solver = kinex.SQPIKSolver(robot, end_link)
    
    # Configure solver (aligned with C++ benchmarks)
    config = kinex.SolverConfig()
    config.tolerance = 5e-4
    config.max_iterations = 64
    config.enable_warm_start = True
    config.regularization = 1e-5
    solver.set_solver_config(config)
    
    # Generate test samples (sample random joint configs once)
    print("\n  Generating test cases...")
    q_samples = sampler.sample(num_samples)
    
    # Run benchmarks with 4 test scenarios
    results = {
        "robot_name": robot_name,
        "dof": dof,
        "total_samples": num_samples,
        "seed": seed,
        "cold_start_zero": {},
        "cold_start_random": {},
        "warm_start": {},
        "trajectory": {}
    }
    
    for case_type in ["cold_start_zero", "cold_start_random", "warm_start", "trajectory"]:
        print(f"\n  Testing {case_type}...")
        
        successes = 0
        failures = 0
        total_time = 0.0
        total_iterations = 0
        pos_errors = []
        rot_errors = []
        
        # Generate random initial guesses for cold_start_random (reuse across samples)
        if case_type == "cold_start_random":
            random_inits = sampler.sample(num_samples)
        
        # Generate trajectory for trajectory scenario
        if case_type == "trajectory":
            # Generate multiple trajectories to match num_samples
            # Each trajectory has 100 waypoints
            traj_len = 100
            num_trajs = (num_samples + traj_len - 1) // traj_len
            
            trajectory_samples = []
            for _ in range(num_trajs):
                q_start = sampler.sample(1)[0]
                q_end = sampler.sample(1)[0]
                for k in range(traj_len):
                    t = k / (traj_len - 1)
                    q = q_start + (q_end - q_start) * t
                    trajectory_samples.append(q)
            
            # Truncate to exact number of samples requested
            trajectory_samples = np.array(trajectory_samples[:num_samples])
        
        # Determine which samples to iterate over
        if case_type == "trajectory":
            test_samples = trajectory_samples
        else:
            test_samples = q_samples
            
        prev_solution = None
        
        for i, q_target in enumerate(test_samples):
            # 1. Compute target pose via FK (very fast)
            target_pos, target_rot = oracle.compute_pose(q_target)
            
            # Convert to Transform
            rpy = R.from_matrix(target_rot).as_euler('xyz', degrees=False)
            target_transform = kinex.Transform.from_position_rpy(target_pos, rpy)
            
            # 2. Choose initial guess based on strategy
            if case_type == "cold_start_zero":
                q_init = np.zeros(dof)
            elif case_type == "cold_start_random":
                q_init = random_inits[i]
            elif case_type == "warm_start":
                # Add small noise to ground truth
                noise = np.random.normal(0, 0.1, size=dof)
                q_init = q_target + noise
            else:  # trajectory
                # Check if this is the start of a new trajectory segment
                is_new_traj = (i % 100 == 0)
                
                # Use previous solution if not start of new trajectory
                if prev_solution is not None and not is_new_traj:
                    q_init = prev_solution
                else:
                    # For the first point, assume we are at the start configuration
                    q_init = q_target
            
            # 3. Solve IK (timed)
            start_time = time.perf_counter()
            result = solver.solve(target_transform, q_init)
            solve_time = time.perf_counter() - start_time
            
            # Update previous solution for trajectory
            if case_type == "trajectory":
                prev_solution = result.solution
            
            total_time += solve_time
            
            # 4. Verify solution by comparing poses (not joint angles!)
            if result.status.converged:
                successes += 1
                total_iterations += result.status.iterations
                
                # Compute achieved pose
                achieved_pos, achieved_rot = oracle.compute_pose(result.solution)
                
                # Position error
                pos_error = np.linalg.norm(achieved_pos - target_pos)
                pos_errors.append(pos_error)
                
                # Rotation error (angular distance in radians)
                # Convert to quaternions for proper angular distance
                q_target_quat = R.from_matrix(target_rot).as_quat()
                q_achieved_quat = R.from_matrix(achieved_rot).as_quat()
                # Compute angular distance using quaternion inner product
                dot_product = np.abs(np.dot(q_target_quat, q_achieved_quat))
                dot_product = np.clip(dot_product, -1.0, 1.0)
                rot_error = 2 * np.arccos(dot_product)  # Angular distance in radians
                rot_errors.append(rot_error)
            else:
                failures += 1
        
        # Compute statistics
        total_cases = len(test_samples)
        success_rate = successes / total_cases * 100
        avg_time = total_time / total_cases * 1_000_000  # Convert to µs
        avg_iterations = total_iterations / successes if successes > 0 else 0
        avg_pos_error = np.mean(pos_errors) * 1000 if pos_errors else 0  # Convert to mm
        avg_rot_error_rad = np.mean(rot_errors) if rot_errors else 0
        avg_rot_error_deg = avg_rot_error_rad * 180.0 / np.pi  # Convert to degrees
        
        results[case_type] = {
            "cases": total_cases,
            "successes": successes,
            "failures": failures,
            "success_rate": success_rate,
            "avg_time_us": avg_time,
            "avg_iterations": avg_iterations,
            "avg_pos_error_mm": avg_pos_error,
            "avg_rot_error_deg": avg_rot_error_deg
        }
        
        print(f"    Cases: {total_cases}")
        print(f"    Success rate: {success_rate:.1f}%")
        print(f"    Avg time: {avg_time:.1f} us")
        print(f"    Avg iterations: {avg_iterations:.1f}")
        print(f"    Avg position error: {avg_pos_error:.6f} mm")
        print(f"    Avg rotation error: {avg_rot_error_deg:.6f} deg")
    
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
            "iterations_per_solve": case_data["avg_iterations"],
            "success_rate": case_data["success_rate"]
        }
    
    benchmark_results.append(make_benchmark_entry("ColdStart_Zero", robot_name, 
                                                   results["cold_start_zero"], 
                                                   results["cold_start_zero"]["cases"]))
    benchmark_results.append(make_benchmark_entry("ColdStart_Random", robot_name,
                                                   results["cold_start_random"], 
                                                   results["cold_start_random"]["cases"]))
    benchmark_results.append(make_benchmark_entry("WarmStart", robot_name,
                                                   results["warm_start"], 
                                                   results["warm_start"]["cases"]))
    benchmark_results.append(make_benchmark_entry("Trajectory", robot_name,
                                                   results["trajectory"], 
                                                   results["trajectory"]["cases"]))
    
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
    
    output_path = os.path.join(output_dir, f"{robot_name}_results.json")
    with open(output_path, 'w') as f:
        json.dump(output_data, f, indent=2)
    print(f"\n  Results saved to {output_path}")
    
    return results

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Tier A IK Benchmark Runner")
    parser.add_argument("--samples", "-n", type=int, default=1000,
                       help="Number of test samples per robot (default: 1000)")
    parser.add_argument("--seed", "-s", type=int, default=42,
                       help="Random seed for reproducibility (default: 42)")
    parser.add_argument("--output", "-o", type=str, default=None,
                       help="Output directory for results")
    args = parser.parse_args()
    
    # Navigate from benchmarks/ directory to project root
    project_root = os.path.abspath(os.path.join(current_dir, ".."))
    
    models_dir = os.path.join(project_root, "examples/models/ur5")
    output_dir = args.output or os.path.join(project_root, "benchmarks/results")
    
    os.makedirs(output_dir, exist_ok=True)
    
    print("MixKinBench Tier A Benchmark Runner")
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
    print(f"\n{'Robot':<15} {'DOF':<5} {'Cold(0)':<12} {'Cold(R)':<12} {'Warm':<12} {'Traj':<12}")
    print("-" * 80)
    
    for result in all_results:
        robot = result['robot_name']
        dof = result['dof']
        cold_zero_sr = result['cold_start_zero'].get('success_rate', 0)
        cold_rand_sr = result['cold_start_random'].get('success_rate', 0)
        warm_sr = result['warm_start'].get('success_rate', 0)
        traj_sr = result['trajectory'].get('success_rate', 0)
        
        print(f"{robot:<15} {dof:<5} {cold_zero_sr:>5.1f}% {cold_rand_sr:>10.1f}% {warm_sr:>10.1f}% {traj_sr:>10.1f}%")
    
    print("\n" + "="*80)
    print("Benchmark complete!")
    print("="*80)

if __name__ == "__main__":
    main()
