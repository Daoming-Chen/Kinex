import pytest
import kinex
import numpy as np
import threading

def test_robot_static_factory():
    # Create a minimal valid URDF string
    urdf = """<?xml version="1.0"?>
    <robot name="test_robot">
        <link name="base_link"/>
        <link name="tip_link"/>
        <joint name="joint1" type="revolute">
            <parent link="base_link"/>
            <child link="tip_link"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
            <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"/>
        </joint>
    </robot>
    """
    robot = kinex.Robot.from_urdf_string(urdf, "tip_link")
    assert robot.name == "test_robot"
    assert robot.dof == 1
    assert robot.end_link == "tip_link"
    
    # Check low-level model access
    model = robot.model
    assert isinstance(model, kinex.RobotModel)
    assert model.get_name() == "test_robot"

def test_robot_kinematics():
    urdf = """<?xml version="1.0"?>
    <robot name="test_robot">
        <link name="base_link"/>
        <link name="tip_link"/>
        <joint name="joint1" type="revolute">
            <parent link="base_link"/>
            <child link="tip_link"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
            <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"/>
        </joint>
    </robot>
    """
    robot = kinex.Robot.from_urdf_string(urdf, "tip_link")
    
    # Forward Kinematics
    q = np.array([np.pi/2])
    pose = robot.forward_kinematics(q)
    
    # At pi/2 rotation around Z at (1,0,0):
    # Origin is translated by (1,0,0).
    # Joint rotates frame. 
    # Wait, if joint is at (1,0,0) and tip is at joint origin (since tip link has no other visuals/collision offsets implicitly, and joint connects parent origin to child origin).
    # The joint origin defines where the child link frame is relative to parent.
    # So tip link frame is at (1,0,0) relative to base.
    # Rotation happens AROUND the joint axis at the joint origin.
    # So the frame rotates, but its origin stays at (1,0,0).
    # Let's verify translation.
    assert np.allclose(pose.translation(), [1.0, 0.0, 0.0])
    
    # Inverse Kinematics
    target = pose
    q_init = np.array([0.0])
    sol, status = robot.inverse_kinematics(target, q_init)
    assert status.converged
    assert np.allclose(sol, q, atol=1e-3)
    
    # Jacobian
    J = robot.compute_jacobian(q)
    assert J.shape == (6, 1)

def test_robot_clone():
    urdf = """<?xml version="1.0"?>
    <robot name="test_robot">
        <link name="base_link"/>
        <link name="tip_link"/>
        <joint name="joint1" type="fixed">
            <parent link="base_link"/>
            <child link="tip_link"/>
        </joint>
    </robot>
    """
    robot = kinex.Robot.from_urdf_string(urdf, "tip_link")
    robot.set_ik_tolerance(1e-5)
    
    cloned = robot.clone()
    assert cloned.name == robot.name
    assert cloned.get_solver_config().tolerance == 1e-5
    
    # modify clone
    cloned.set_ik_tolerance(1e-2)
    assert robot.get_solver_config().tolerance == 1e-5 # original unchanged

def test_robot_thread_safety():
    urdf = """<?xml version="1.0"?>
    <robot name="test_robot">
        <link name="base_link"/>
        <link name="tip_link"/>
        <joint name="joint1" type="revolute">
            <parent link="base_link"/>
            <child link="tip_link"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
        </joint>
    </robot>
    """
    robot = kinex.Robot.from_urdf_string(urdf, "tip_link")
    
    def worker():
        local_robot = robot.clone()
        q = np.array([0.0])
        for _ in range(100):
            local_robot.forward_kinematics(q)
            
    threads = []
    for _ in range(4):
        t = threading.Thread(target=worker)
        threads.append(t)
        t.start()
        
    for t in threads:
        t.join()

