import pytest
import pybullet as p
import camera as camera
import range_finder as range_finder
import iris as iris
from pathlib import Path
import os
import state
import pybullet_data

@pytest.fixture(scope="session")
def bullet_connect():
    physicsClient = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.80665)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    src = Path.joinpath(Path(__file__).parent.parent, "src")
    state.dir_path = Path.joinpath(src, "ardupilot/libraries/SITL/examples/JSON/pybullet/models")
    iris.Iris()

    

@pytest.fixture()
def camera_obj():
    return camera.Camera(attached_to_object=1,port=6000,direction=[0,0,-1],depth_map=True)

@pytest.fixture()
def range_finder_obj():
    return range_finder.Range_Finder(port=6004,direction=[0,0,-1],dist=100)

@pytest.fixture()
def iris_obj():
    dir_path = Path(os.path.dirname(os.path.abspath(__file__))).joinpath("ardupilot/libraries/SITL/examples/JSON/pybullet/models/iris/iris.urdf")
    state.robot_id=p.loadURDF(str(state.dir_path/"iris/iris.urdf"),[0,0,0.2])
    return iris.Iris()
