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
robot = p.loadURDF("3dof/urdf/planar_real.urdf", useFixedBase=False, flags=flags)

# get joint and link info
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

marker_radius = 0.03
vis_red  = p.createVisualShape(p.GEOM_SPHERE, radius=marker_radius, rgbaColor=[1,0,0,1])   # COM now
vis_blue = p.createVisualShape(p.GEOM_SPHERE, radius=marker_radius, rgbaColor=[0,0,1,1])   # COM prev (optional)
com_now  = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                             baseVisualShapeIndex=vis_red,  basePosition=com1)
com_prev = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                             baseVisualShapeIndex=vis_blue, basePosition=com1)


for t in range(freq*sim_time):
    test = "world_force"
    link_tot = (0, 0, 0)
    for i in range(p.getNumJoints(robot)):
        link_tot = np.add(link_tot, p.getLinkState(robot, i)[0])
    com2 = np.divide(link_tot, 3)
    com1 = com2

    # virtual mechanism calcs
    # get q1 and q2
    q1 = p.getJointState(robot, 1)[0]
    q2 = p.getJointState(robot, 2)[0]
    
    if test == "world_force":
        # find x1, x2 and com in terms of q1 and q2
        # assume base position and orinetation is measured
        pos, orn = p.getBasePositionAndOrientation(robot)
        orn2 = p.getEulerFromQuaternion(orn)
        q0 = orn2[2]

        # x1 in world frame
        x1 = np.array(pos)

        # x2 in base frame
        x2 = [-(0.3*np.sin(q1) + 0.3*np.sin(q1 + q2)), 
                    0.3 + 0.3*np.cos(q1) + 0.3*np.cos(q1 + q2)]
        
        # com in base frame
        com = [-(0.15*np.sin(q1) + 0.05*np.sin(q1 + q2)), 
            0.25 + 0.15*np.cos(q1) + 0.05*np.cos(q1 + q2)]
        
        # vectors
        x1 = np.array([[x1[0]], [x1[1]], [0.03]])
        x2 = np.array([[x2[0]], [x2[1]], [0.03], [1]])
        com = np.array([[com[0]], [com[1]], [0.03], [1]])

        # base frame to world transformation matrix (4x4)
        T02 = np.array([[np.cos(q0), -np.sin(q0), 0, float(x1[0])],
                    [np.sin(q0), np.cos(q0), 0, float(x1[1])], 
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
        
        # x2 and com to world vectors (4x1)
        x2w = np.matmul(T02, x2)
        comw = np.matmul(T02, com)

        # shorten vectors (3x1)
        x2 = x2w[:-1]
        com = comw[:-1]
        p.resetBasePositionAndOrientation(com_now,  com.flatten(), [0,0,0,1])
        p.addUserDebugLine(x1.flatten(), x2.flatten(), [0, 0, 0], 1, 0, replaceItemUniqueId=line_id)

        # define f1 and f2
        shape = "C"
        if shape == "line":
            k = 5
            f1 = k*(x1 - x2)
            f2 = k*(x2 - x1)
        elif shape == "C":
            k = -5
            ext = x1 - x2
            ext_mag = np.linalg.norm(ext)
            ext_hat = ext / ext_mag
            ext_mid = x1 + x2 / 2
            x01 = ext_mid + (0.15 * ext_hat)
            x02 = ext_mid - (0.15 * ext_hat)
            f1 = k*(x1 - x01)
            f2 = k*(x2 - x02)
        else:
            print("shape error") 
            p.disconnect()
        
        # apply force
        p.applyExternalForce(robot, 0, f1.flatten(), x1.flatten(), p.WORLD_FRAME)
        p.applyExternalForce(robot, 2, f2.flatten(), x2.flatten(), p.WORLD_FRAME)
    if test == "base_torque":
        com = [0.25 + 0.15*np.cos(q1) + 0.05*np.cos(q1 + q2), 
            0.15*np.sin(q1) + 0.05*np.sin(q1 + q2)]
    
        # due to urdf coordinates (x, y, z) --> (-y, x, z)
        com = [-com[1], com[0], 0.03]

        # find x1 and x2 in terms of q1 and q2
        x1 = com1 - com
        x2 = x1 + [-(0.3*np.sin(q1) + 0.3*np.sin(q1 + q2)), 
                    0.3 + 0.3*np.cos(q1) + 0.3*np.cos(q1 + q2), 0]
        p.resetBasePositionAndOrientation(com_now,  com, [0,0,0,1])
        p.addUserDebugLine(x1.flatten(), x2.flatten(), [0, 0, 0], 1, 0, replaceItemUniqueId=line_id)

        x1 = np.array([[x1[0]], [x1[1]]])
        x2 = np.array([[x2[0]], [x2[1]]])

        # define f1 and f2
        shape = "C"
        if shape == "line":
            k = 5
            f1 = k*(x1 - x2)
            f2 = k*(x2 - x1)
        elif shape == "C":
            k = -5
            ext = x1 - x2
            ext_mag = np.linalg.norm(ext)
            ext_hat = ext / ext_mag
            ext_mid = x1 + x2 / 2
            x01 = ext_mid + (0.15 * ext_hat)
            x02 = ext_mid - (0.15 * ext_hat)
            f1 = k*(x1 - x01)
            f2 = k*(x2 - x02)
        else:
            print("shape error") 
            p.disconnect()

        # find t1 and t2
        dx1dq1 = [0.15*np.cos(q1) + 0.05*np.cos(q1 + q2), 
                    0.15*np.sin(q1) - 0.05*np.sin(q1 + q2)]
        dx1dq2 = [0.05*np.cos(q1 + q2), 0.05*np.sin(q1 +q2)]
        dh1dqT = np.array([dx1dq1, dx1dq2])

        dx2dq1 = np.array([-0.3*np.cos(q1) - 0.3*np.cos(q1 + q2),
                            -0.3*np.sin(q1) - 0.3*np.sin(q1 + q2)])
        dx2dq2 = np.array([-0.3*np.cos(q1 + q2), -0.3*np.sin(q1 +q2)])
        dh2dqT = np.array([dx2dq1, dx2dq2])

        # control motors
        t = np.matmul(dh1dqT, f1) + np.matmul(dh2dqT, f2)
        p.setJointMotorControlArray(robot, [1, 2], p.TORQUE_CONTROL, forces = t.flatten())  
    
    p.stepSimulation()
    # time.sleep(3/1200)

p.disconnect()
