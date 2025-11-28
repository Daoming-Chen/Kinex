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

    # Use new Unified Robot API
    print(f"Loading robot from {urdf_path}...")
    robot = kinex.Robot.from_urdf(urdf_path, "wrist_3_link")
    print(f"Loaded robot: {robot.name} with {robot.dof} DOF")

    # Compute FK for home position
    q_home = np.zeros(robot.dof)
    pose = robot.forward_kinematics(q_home)
    
    print("\nHome Position (q=0):")
    print(f"Translation: {pose.translation()}")
    print(f"Rotation:\n{pose.rotation()}")
    
    pos, quat = pose.as_position_quaternion()
    print(f"Quaternion (x,y,z,w): {quat}")

    # Compute for another configuration
    q_pose = np.array([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])
    pose2 = robot.compute_pose(q_pose) # Test alias
    
    print("\nPose 2:")
    print(f"Joints: {q_pose}")
    print(f"Translation: {pose2.translation()}")

if __name__ == "__main__":
    main()