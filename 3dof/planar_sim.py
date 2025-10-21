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
    q1 = p.getJointState(robot, 1)[0]
    q2 = p.getJointState(robot, 2)[0]

    com = [0.25 + 0.15*np.cos(q1) + 0.05*np.cos(q1 + q2), 
            0.15*np.sin(q1) + 0.05*np.sin(q1 + q2)]
    
    # due to urdf coordinates (x, y, z) --> (-y, x, z)
    com = [-com[1], com[0]]

    # find x1 and x2 in terms of q1 and q2
    x1 = com1[0:1] - com
    x2 = x1 + [-(0.3*np.sin(q1) + 0.3*np.sin(q1 + q2)), 
                0.3 + 0.3*np.cos(q1) + 0.3*np.cos(q1 + q2)]
    
    x1 = np.array([[x1[0]], [x1[1]]])
    x2 = np.array([[x2[0]], [x2[1]]])

    # define f1 and f2
    k = 10
    f1 = k*(x1 - x2)
    f2 = k*(x2 - x1)

    # find t1 and t2
    dx1dq1 = [0.15*np.cos(q1) + 0.05*np.cos(q1 + q2), 
                0.15*np.sin(q1) - 0.05*np.sin(q1 + q2)]
    dx1dq2 = [0.05*np.cos(q1 + q2), 0.05*np.sin(q1 +q2)]
    dh1dqT = np.array([dx1dq1, dx1dq2])

    dx2dq1 = dx1dq1 + np.array([-0.3*np.cos(q1) - 0.3*np.cos(q1 + q2),
                        -0.3*np.sin(q1) - 0.3*np.sin(q1 + q2)])
    dx2dq2 = dx1dq2 + np.array([-0.3*np.cos(q1 + q2), -0.3*np.sin(q1 +q2)])
    dh2dqT = np.array([dx2dq1, dx2dq2])

    t = np.matmul(dh1dqT, f1) + np.matmul(dh2dqT, f2)

    # control motors
    p.setJointMotorControlArray(robot, [1, 2], p.TORQUE_CONTROL, forces = [t[0], t[1]])

    p.stepSimulation()

p.disconnect()
