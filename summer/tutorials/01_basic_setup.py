import time
import pybullet as p
import pybullet_data

# connect to the PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# set gravity and load simple world
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
p.loadURDF("r2d2.urdf", [0, 0, 0.5])

# step simulation for 1 second
for _ in range(240):
    p.stepSimulation()
    time.sleep(10/240)

p.disconnect()
