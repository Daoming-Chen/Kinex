import kinex
import numpy as np
import os
import sys
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

def main():
    # Path to URDF
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/ur5/ur5e.urdf")
    
    if not os.path.exists(urdf_path):
        urdf_path = "examples/models/ur5/ur5e.urdf"
        
    if not os.path.exists(urdf_path):
        print(f"URDF file not found: {urdf_path}")
        sys.exit(1)

    robot = kinex.Robot.from_urdf(urdf_path, "wrist_3_link")
    
    # Configure solver
    config = robot.get_solver_config()
    config.enable_warm_start = True
    robot.set_solver_config(config)
    
    # Generate a simple line trajectory
    start_pos = np.array([0.4, -0.4, 0.2])
    end_pos = np.array([0.4, 0.4, 0.5])
    steps = 50
    
    # Use a reference orientation (pointing down, roughly)
    roll, pitch, yaw = np.pi, 0.0, 0.0  # Pointing down
    
    traj_q = []
    current_q = np.zeros(6) # Initial guess
    
    print(f"Generating trajectory with {steps} steps...")
    
    for i in range(steps):
        alpha = i / (steps - 1)
        pos = start_pos * (1 - alpha) + end_pos * alpha
        
        # Create target pose with desired position and orientation
        target_pose = kinex.Transform.from_position_rpy(pos, np.array([roll, pitch, yaw]))
        
        # Use previous solution as guess
        solution, status = robot.inverse_kinematics(target_pose, current_q)
        
        if status.converged:
            current_q = solution
            traj_q.append(current_q)
        else:
            print(f"IK failed at step {i}")
            break
            
    traj_q = np.array(traj_q)
    print(f"Generated {len(traj_q)} waypoints")
    
    if HAS_MATPLOTLIB and len(traj_q) > 0:
        plt.figure(figsize=(10, 6))
        for j in range(6):
            plt.plot(traj_q[:, j], label=f'Joint {j+1}')
        plt.xlabel('Step')
        plt.ylabel('Joint Angle (rad)')
        plt.title('Joint Trajectory')
        plt.legend()
        plt.grid(True)
        print("Displaying plot...")
        # plt.show()
        print("Plot generation supported but disabled for non-interactive run")
    elif not HAS_MATPLOTLIB:
        print("Install matplotlib to visualize the trajectory")

if __name__ == "__main__":
    main()