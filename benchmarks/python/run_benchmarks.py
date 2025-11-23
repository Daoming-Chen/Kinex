#!/usr/bin/env python3
"""
Unified Benchmark Runner for urdfx IK Solvers.

This script provides a unified interface to run benchmarks on both:
- Tier A: Real-world robots (UR5e and variants, 6-10 DOF)
- Tier B: Synthetic mixed-chain robots (8-20 DOF)

Usage:
    # Run all benchmarks (Tier A + Tier B)
    python run_benchmarks.py --all
    
    # Run only Tier A benchmarks
    python run_benchmarks.py --tier-a
    
    # Run only Tier B benchmarks
    python run_benchmarks.py --tier-b
    
    # Custom configurations
    python run_benchmarks.py --tier-a --samples 2000
    python run_benchmarks.py --tier-b --dof-min 10 --dof-max 15
    
    # With visualization
    python run_benchmarks.py --all --visualize
"""

import os
import sys
import argparse
import subprocess
import time
from pathlib import Path

# Add current directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)


def run_tier_a_benchmarks(args_dict):
    """Run Tier A benchmarks (real-world robots)."""
    print("\n" + "="*70)
    print("TIER A: Real-World Robots Benchmark")
    print("="*70)
    
    cmd = [sys.executable, "run_tier_a_benchmarks.py"]
    
    if args_dict.get('samples'):
        cmd.extend(['--samples', str(args_dict['samples'])])
    if args_dict.get('seed'):
        cmd.extend(['--seed', str(args_dict['seed'])])
    if args_dict.get('output'):
        cmd.extend(['--output', args_dict['output']])
    
    start_time = time.time()
    result = subprocess.run(cmd, cwd=current_dir)
    elapsed = time.time() - start_time
    
    if result.returncode != 0:
        print(f"\n✗ Tier A benchmarks failed with exit code {result.returncode}")
        return False
    
    print(f"\n✓ Tier A benchmarks completed in {elapsed:.2f}s")
    return True


def run_tier_b_benchmarks(args_dict):
    """Run Tier B benchmarks (synthetic robots)."""
    print("\n" + "="*70)
    print("TIER B: Synthetic Mixed-Chain Robots Benchmark")
    print("="*70)
    
    cmd = [sys.executable, "run_tier_b_benchmarks.py"]
    
    if args_dict.get('dof_min'):
        cmd.extend(['--dof-min', str(args_dict['dof_min'])])
    if args_dict.get('dof_max'):
        cmd.extend(['--dof-max', str(args_dict['dof_max'])])
    if args_dict.get('samples'):
        cmd.extend(['--samples', str(args_dict['samples'])])
    if args_dict.get('max_iterations'):
        cmd.extend(['--max-iterations', str(args_dict['max_iterations'])])
    if args_dict.get('tolerance'):
        cmd.extend(['--tolerance', str(args_dict['tolerance'])])
    if args_dict.get('robot_seed'):
        cmd.extend(['--robot-seed', str(args_dict['robot_seed'])])
    if args_dict.get('sample_seed'):
        cmd.extend(['--sample-seed', str(args_dict['sample_seed'])])
    if args_dict.get('output'):
        cmd.extend(['--output', args_dict['output']])
    # Don't pass --visualize to tier_b, we'll handle it separately
    
    start_time = time.time()
    result = subprocess.run(cmd, cwd=current_dir)
    elapsed = time.time() - start_time
    
    if result.returncode != 0:
        print(f"\n✗ Tier B benchmarks failed with exit code {result.returncode}")
        return False
    
    print(f"\n✓ Tier B benchmarks completed in {elapsed:.2f}s")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Unified Benchmark Runner for urdfx IK Solvers",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    # Tier selection
    tier_group = parser.add_argument_group('Tier Selection')
    tier_group.add_argument('--all', action='store_true',
                           help='Run all benchmarks (Tier A + Tier B)')
    tier_group.add_argument('--tier-a', action='store_true',
                           help='Run Tier A benchmarks (real-world robots)')
    tier_group.add_argument('--tier-b', action='store_true',
                           help='Run Tier B benchmarks (synthetic robots)')
    
    # Common configuration
    common_group = parser.add_argument_group('Common Configuration')
    common_group.add_argument('--samples', '-n', type=int,
                             help='Number of test samples (default: 1000 for Tier A, 1000 for Tier B)')
    common_group.add_argument('--output', '-o', type=str,
                             help='Output directory for results (default: benchmarks/results)')
    common_group.add_argument('--visualize', action='store_true',
                             help='Generate visualizations after benchmarking')
    
    # Tier A specific
    tier_a_group = parser.add_argument_group('Tier A Specific Options')
    tier_a_group.add_argument('--seed', type=int, default=42,
                             help='Random seed for Tier A (default: 42)')
    
    # Tier B specific
    tier_b_group = parser.add_argument_group('Tier B Specific Options')
    tier_b_group.add_argument('--dof-min', type=int, default=8,
                             help='Minimum DOF for Tier B (default: 8)')
    tier_b_group.add_argument('--dof-max', type=int, default=20,
                             help='Maximum DOF for Tier B (default: 20)')
    tier_b_group.add_argument('--max-iterations', type=int, default=500,
                             help='Maximum IK solver iterations for Tier B (default: 500)')
    tier_b_group.add_argument('--tolerance', type=float, default=1e-6,
                             help='IK solver tolerance for Tier B (default: 1e-6)')
    tier_b_group.add_argument('--robot-seed', type=int, default=42,
                             help='Seed for robot generation in Tier B (default: 42)')
    tier_b_group.add_argument('--sample-seed', type=int, default=42,
                             help='Seed for sample generation in Tier B (default: 42)')
    
    args = parser.parse_args()
    
    # Determine which tiers to run
    run_a = args.tier_a or args.all
    run_b = args.tier_b or args.all
    
    # If no tier specified, show help
    if not (run_a or run_b):
        parser.print_help()
        print("\nError: Please specify at least one tier to run (--tier-a, --tier-b, or --all)")
        return 1
    
    # Build arguments dict
    args_dict = {
        'samples': args.samples,
        'output': args.output,
        'visualize': args.visualize,
        'seed': args.seed,
        'dof_min': args.dof_min,
        'dof_max': args.dof_max,
        'max_iterations': args.max_iterations,
        'tolerance': args.tolerance,
        'robot_seed': args.robot_seed,
        'sample_seed': args.sample_seed,
    }
    
    # Set default output directory if not provided
    if not args_dict['output']:
        args_dict['output'] = os.path.join(
            os.path.abspath(os.path.join(current_dir, "../..")),
            "benchmarks/results"
        )
    
    # Print header
    print("\n" + "="*70)
    print("urdfx Unified Benchmark Runner")
    print("="*70)
    print(f"Running: {'Tier A' if run_a else ''} {'+ Tier B' if run_a and run_b else 'Tier B' if run_b else ''}")
    print(f"Output directory: {args_dict['output']}")
    print("="*70)
    
    # Run benchmarks
    overall_start = time.time()
    success = True
    
    if run_a:
        if not run_tier_a_benchmarks(args_dict):
            success = False
    
    if run_b and success:  # Only run Tier B if Tier A succeeded (or wasn't run)
        if not run_tier_b_benchmarks(args_dict):
            success = False
    
    overall_elapsed = time.time() - overall_start
    
    # Final summary
    print("\n" + "="*70)
    if success:
        print("✓ ALL BENCHMARKS COMPLETED SUCCESSFULLY")
    else:
        print("✗ SOME BENCHMARKS FAILED")
    print(f"Total time: {overall_elapsed:.2f}s")
    print("="*70)
    
    # Run visualizations if requested
    if success and args_dict.get('visualize'):
        print("\n" + "="*70)
        print("GENERATING VISUALIZATIONS")
        print("="*70)
        
        # Tools directory is in ../tools relative to this script
        tools_dir = os.path.abspath(os.path.join(current_dir, "../tools"))
        viz_script = os.path.join(tools_dir, "visualize_benchmarks.py")
        
        viz_cmd = [sys.executable, viz_script, 
                  "--results-dir", args_dict['output']]
        viz_result = subprocess.run(viz_cmd, cwd=current_dir)
        
        if viz_result.returncode == 0:
            print("\n✓ Visualizations generated successfully")
        else:
            print("\n✗ Visualization generation failed")
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
