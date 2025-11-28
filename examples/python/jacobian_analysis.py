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

    # Use unified Robot API
    robot = kinex.Robot.from_urdf(urdf_path, "wrist_3_link")
    
    # Analyze a trajectory
    steps = 20
    q_start = np.zeros(6)
    q_end = np.array([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])
    
    print(f"{ 'Step':<5} | {'Manipulability':<15} | {'Condition Num':<15} | {'Is Singular':<12}")
    print("-" * 55)
    
    for i in range(steps):
        alpha = i / (steps - 1)
        q = q_start * (1 - alpha) + q_end * alpha
        
        manip = robot.get_manipulability(q)
        cond = robot.get_condition_number(q)
        singular = robot.is_singular(q, threshold=1e-3)
        
        print(f"{i:<5} | {manip:<15.4f} | {cond:<15.4f} | {str(singular):<12}")
        
    print("-" * 55)
    
    # Check a singular configuration specifically
    q_sing = np.zeros(6) # Vertical up
    print("\nSingular Configuration (Vertical Up):")
    print(f"Manipulability: {robot.get_manipulability(q_sing)}")
    print(f"Singular: {robot.is_singular(q_sing)}")
    
    # Jacobian matrix
    J = robot.compute_jacobian(q_sing)
    print("\nJacobian Matrix at Singularity:")
    print(np.round(J, 3))

if __name__ == "__main__":
    main()