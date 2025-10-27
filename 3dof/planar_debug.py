import os
import pybullet as p
import pybullet_data
import numpy as np
import imageio_ffmpeg
import matplotlib.pyplot as plt
import os
from datetime import datetime
import time

p.connect(p.GUI)
p.setRealTimeSimulation(0)
p.setGravity(0,0,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0,0,-5], useFixedBase=True)

flags = (
    p.URDF_USE_INERTIA_FROM_FILE |
    p.URDF_USE_SELF_COLLISION |
    p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT  # or INCLUDE_PARENT if needed
)
robot = p.loadURDF("3dof/planar_real.urdf", useFixedBase=False, flags=flags)

# set intial position
p.resetJointState(robot, 1, 3.14/4)
p.resetJointState(robot, 2, 3.14/4)

# find initial com
link_tot = (0, 0, 0)
for i in range(p.getNumJoints(robot)):
    link_tot = np.add(link_tot, p.getLinkState(robot, i)[0])
    # disable velocity motors
    p.setJointMotorControl2(robot, i, p.VELOCITY_CONTROL, force=0)
    p.changeDynamics(robot, i,
                     lateralFriction=0.0,    # surface Coulomb friction
                     spinningFriction=0.0,
                     rollingFriction=0.0,
                     linearDamping=0.0,
                     angularDamping=0.0,
                     restitution=0.0)
com1 = np.divide(link_tot, 3)

dt = 1/ 1200
freq = int(1 / dt)
p.setTimeStep(dt)
sim_time = 60

# debug x1 and x2
x1_pt = [0, 0, 0]
x2_pt = [0, 0, 0]
line_id = p.addUserDebugLine(x1_pt, x2_pt, [1, 0, 0], lineWidth=2, lifeTime=0)


for t in range(freq*sim_time):
    link_tot = (0, 0, 0)
    for i in range(p.getNumJoints(robot)):
        link_tot = np.add(link_tot, p.getLinkState(robot, i)[0])
    com2 = np.divide(link_tot, 3)
    # print(com1)
    com1 = com2
    # print(com2)

    # virtual mechanism calcs
    # get q1 and q2
    q1 = p.getJointState(robot, 1)[0]
    q2 = p.getJointState(robot, 2)[0]

    com = [0.25 + 0.15*np.cos(q1) + 0.05*np.cos(q1 + q2), 
            0.15*np.sin(q1) + 0.05*np.sin(q1 + q2)]
    # print(com)

    pos, orn = p.getBasePositionAndOrientation(robot)
    orn2 = p.getEulerFromQuaternion(orn)
    q0 = orn2[2]

    # due to urdf coordinates (x, y, z) --> (-y, x, z)
    com = [-com[1], com[0], 0.03]
    # print(com)
    # print("------------")
    
    # find x1 and x2 in terms of q1 and q2
    x1 = com1 - np.array(com)
    x1 = np.array(pos)
    x2 = [-(0.3*np.sin(q1) + 0.3*np.sin(q1 + q2)), 
                0.3 + 0.3*np.cos(q1) + 0.3*np.cos(q1 + q2), 0]
    
    x1 = np.array([[x1[0]], [x1[1]], [0.03]])
    x2 = np.array([[x2[0]], [x2[1]], [0.03], [1]])

    T02 = np.array([[np.cos(q0), -np.sin(q0), 0, float(x1[0])],
                 [np.sin(q0), np.cos(q0), 0, float(x1[1])], 
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
    
    x2w = np.matmul(T02, x2)

    x2 = x2w[:-1]

    # define f1 and f2
    k = 5
    f1 = k*(x1 - x2)
    f2 = k*(x2 - x1)
    p.addUserDebugLine([x1[0], x1[1], x1[2]+0.1], [x2[0], x2[1], x2[2]+0.1], [0, 0, 0], 1, 0)

    time.sleep(3)
    # apply force
    startPos, startOrn = p.getBasePositionAndOrientation(robot)
    endPos = [0.10865538, -0.00479139, -0.00014949] 
    endOrn = (6.695681063754675e-05, 0.00034720412585835883, 0.3141972285100262, 0.9493576652467177)
    # set intial position
    p.resetJointState(robot, 1, 0.8)
    p.resetJointState(robot, 2, 1.2)
    p.resetBasePositionAndOrientation(robot, endPos, endOrn)
    
    p.stepSimulation()

p.disconnect()
