# Initialisation
import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)

# for i in range(0, p.getNumJoints(boxId)):
#     print(p.getJointInfo(boxId, i))
#     print(i, p.getJointState(boxId, i))

p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

# Swivel Head Left and Right
maxForce = 500
p.setJointMotorControl2(bodyUniqueId=boxId,
    jointIndex=13,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity = -1,
    force = maxForce)

for i in range (240*1):
    p.stepSimulation()
    time.sleep(1./240.)
    
p.setJointMotorControl2(bodyUniqueId=boxId,
    jointIndex=13,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity = 2,
    force = maxForce)

for i in range (240*1):
    p.stepSimulation()
    time.sleep(1./240.)

p.setJointMotorControl2(bodyUniqueId=boxId,
    jointIndex=13,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity = -1,
    force = maxForce)

for i in range (240*1):
    p.stepSimulation()
    time.sleep(1./240.)

p.setJointMotorControl2(bodyUniqueId=boxId,
    jointIndex=13,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity = 0,
    force = maxForce)

for i in range (240*1):
    p.stepSimulation()
    time.sleep(1./240.)

# p.disconnect()

# Move Wheels
p.setJointMotorControlArray(boxId,
    jointIndices = (2, 3, 6, 7),
    controlMode = p.VELOCITY_CONTROL,
    targetVelocities = [-10]*4,
    forces = [maxForce]*4)

for i in range (240*4):
    p.stepSimulation()
    time.sleep(1./240.)

p.setJointMotorControlArray(boxId,
    jointIndices = (2, 3, 6, 7),
    controlMode = p.VELOCITY_CONTROL,
    targetVelocities = [10]*4,
    forces = [maxForce]*4)

for i in range (240*4):
    p.stepSimulation()
    time.sleep(1./240.)

p.setJointMotorControlArray(boxId,
    jointIndices = (2, 3, 6, 7),
    controlMode = p.VELOCITY_CONTROL,
    targetVelocities = [0]*4,
    forces = [maxForce]*4)

for i in range (240*4):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect() 
