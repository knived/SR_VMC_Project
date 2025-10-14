import os
import pybullet as p
import pybullet_data

p.connect(p.GUI)

p.setGravity(0,0,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0,0,-5], useFixedBase=True)

robot = p.loadURDF("mich/VispaSat.urdf", useFixedBase=False)

while 1:
    p.stepSimulation()

p.disconnect()
