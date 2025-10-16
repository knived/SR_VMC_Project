import os
import pybullet as p
import pybullet_data
import numpy as np

p.connect(p.GUI)

p.setGravity(0,0,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0,0,-5], useFixedBase=True)

robot = p.loadURDF("3dof/planar.urdf", useFixedBase=False)

# for i in range(0, p.getNumJoints(robot)):
#     print(p.getJointInfo(robot, i))
#     print(p.getJointState(robot, i))
#     print(p.getLinkState(robot, i))
#     print("---------------------------")
# p.disconnect()

# set intial position
p.resetJointState(robot, 1, 3.14/4)
p.resetJointState(robot, 2, 3.14/4)

# find initial com
link_tot = (0, 0, 0)
for i in range(p.getNumJoints(robot)):
    link_tot = np.add(link_tot, p.getLinkState(robot, i)[0])
    # disable velocity motors
    p.setJointMotorControl2(robot, i, p.VELOCITY_CONTROL, force=0)
com1 = np.divide(link_tot, 3)

while 1:
    # track com
    link_tot = (0, 0, 0)
    for i in range(p.getNumJoints(robot)):
        link_tot = np.add(link_tot, p.getLinkState(robot, i)[0])
    com2 = np.divide(link_tot, 3)
    # print(com2)
    p.addUserDebugLine(com1, com2, [1, 0, 0], 1, 0)
    com1 = com2

    # get q1 and q2
    q1 = p.getJointStates(robot, 1)[0]
    q2 = p.getJointState(robot, 2)[0]

    # find x1 and x2 in terms of q1 and q2
    x1 = 0
    x2 = 0
    x = np.array([[x1], [x2]])

    # define f1 and f2
    k = 5
    f1 = k*(x1 - x2)
    f2 = k*(x2 - x1)
    f = np.array([[f1], [f2]])

    # find t1 and t2
    t = np.array([[], []])

    # control motors
    p.setJointMotorControlArray(robot, [1, 2], p.TORQUE_CONTROL, forces = t)

    p.stepSimulation()

p.disconnect()
