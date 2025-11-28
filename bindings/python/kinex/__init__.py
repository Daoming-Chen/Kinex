import os
import sys

if os.name == 'nt':
    # On Windows, we need to explicitly add the directory containing the DLLs
    # to the DLL search path for Python 3.8+
    # The DLLs are located in the same directory as this __init__.py
    kinex_dir = os.path.dirname(__file__)
    os.add_dll_directory(kinex_dir)

from ._kinex import *
from ._kinex import __version__

__all__ = [
    "Transform",
    "JointType",
    "GeometryType",
    "JacobianType",
    "JointLimits",
    "JointDynamics",
    "Geometry",
    "Visual",
    "Collision",
    "Inertial",
    "Link",
    "Joint",
    "RobotModel",
    "Robot",
    "URDFParser",
    "KinematicChain",
    "ForwardKinematics",
    "JacobianCalculator",
    "SolverConfig",
    "SolverStatus",
    "IKResult",
    "IKSolver",
    "SQPIKSolver",
]

