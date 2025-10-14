import pybullet as p
import pybullet_data
import time
import math
from datetime import datetime

# connect to server
physicsClient = p.connect(p.GUI)

# load models
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,0)
planeId = p.loadURDF("plane.urdf", [0,0,-1], useFixedBase=True)

startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot = p.loadURDF("summer/urdf/r2d2.urdf.xacro", startPos, startOrientation,
                   useFixedBase=True)