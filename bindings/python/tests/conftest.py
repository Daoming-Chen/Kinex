import pytest
import os
import urdfx

@pytest.fixture
def test_data_dir():
    # Find examples/models/ur5/ur5e.urdf relative to this file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Path relative to bindings/python/tests/conftest.py
    # bindings/python/tests/../../../examples/models/ur5 -> examples/models/ur5
    model_path = os.path.abspath(os.path.join(current_dir, "../../../examples/models/ur5"))
    
    if os.path.exists(os.path.join(model_path, "ur5e.urdf")):
        return model_path
            
    # If not found, rely on environment variable or fail
    if "URDFX_MODELS_DIR" in os.environ:
        return os.environ["URDFX_MODELS_DIR"]
        
    pytest.skip(f"UR5 model not found at {model_path}. Please set URDFX_MODELS_DIR env var.")

@pytest.fixture
def ur5_urdf_path(test_data_dir):
    return os.path.join(test_data_dir, "ur5e.urdf")

@pytest.fixture
def ur5_robot(ur5_urdf_path):
    return urdfx.Robot.from_urdf_file(ur5_urdf_path)

