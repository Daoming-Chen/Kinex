import kinex
import numpy as np
import os
import sys

def main():
    # Path to URDF
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/ur5/ur5e.urdf")
    
    if not os.path.exists(urdf_path):
        urdf_path = "examples/models/ur5/ur5e.urdf"
        
    if not os.path.exists(urdf_path):
        print(f"URDF file not found: {urdf_path}")
        sys.exit(1)

    # Load robot with unified API
    robot = kinex.Robot.from_urdf(urdf_path, "wrist_3_link")
    
    # Define target
    # Let's use FK to generate a reachable target
    target_q = np.array([0.5, -1.0, 1.5, -1.0, 0.5, 0.0])
    target_pose = robot.forward_kinematics(target_q)
    
    print("Target configuration (joint angles):")
    print(target_q)
    print("\nTarget Pose:")
    print(f"Translation: {target_pose.translation()}")
    
    # Solve IK
    initial_guess = np.zeros(6)
    print("\nSolving IK...")
    
    # Configure IK if needed
    robot.set_ik_tolerance(1e-4)
    
    solution, status = robot.inverse_kinematics(target_pose, initial_guess)
    
    if status.converged:
        print("IK Converged!")
        print("Solution (joint angles):")
        print(solution)
        print(f"Iterations: {status.iterations}")
        print(f"Error: {status.final_error_norm}")
        
        # Verify
        pose_sol = robot.forward_kinematics(solution)
        diff_pos = np.linalg.norm(pose_sol.translation() - target_pose.translation())
        print(f"\nPosition Error: {diff_pos:.6f}")
    else:
        print("IK Failed to converge")
        print(status.message)

if __name__ == "__main__":
    main()