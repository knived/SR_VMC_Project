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

load_flags = (
    p.URDF_USE_INERTIA_FROM_FILE
    | p.URDF_USE_SELF_COLLISION
    | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
)
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot = p.loadURDF("franka_panda/panda.urdf", startPos, startOrientation,
                   useFixedBase=False, flags=load_flags)

hold_id = p.createConstraint(robot, -1, -1, -1, p.JOINT_FIXED,
                             [0,0,0], [0,0,0], startPos, childFrameOrientation=startOrientation)

pos = [-0.65, 0.1, 0.2]
home = p.calculateInverseKinematics(robot, 11, pos, solver=0)
for i in range(p.getNumJoints(robot) - 4):
    p.resetJointState(robot, i, home[i])

for i in range (240*2):
    p.stepSimulation()
    time.sleep(1./240.) 

p.resetBaseVelocity(robot, [0,0,0], [0,0,0])
p.removeConstraint(hold_id)

prevPose = [-0.65, 0.1, 0.2]
prevPose1 = [-0.65, 0.1, 0.2]
hasPrevPose = 0

t = 0
while 1:    
    t = t + 0.002
    p.stepSimulation()

    pos = [-0.65, 0.1 * math.cos(t), 0.2 + 0.1 * math.sin(t)]

    jointPoses = p.calculateInverseKinematics(robot, 11, pos, solver=0)

    for i in range(p.getNumJoints(robot) - 4):
        p.setJointMotorControl2(bodyIndex=robot,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=0,
                                force=80,
                                positionGain=0.06,
                                velocityGain=1)
        
    ls = p.getLinkState(robot, 11)
    if (hasPrevPose):
        p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, 0)
    p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, 0)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1

p.disconnect()