import os
import pybullet as p
import pybullet_data
import numpy as np
import imageio_ffmpeg
import matplotlib.pyplot as plt

p.connect(p.DIRECT)

p.setGravity(0,0,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0,0,-5], useFixedBase=True)

flags = (
    p.URDF_USE_INERTIA_FROM_FILE |
    p.URDF_USE_SELF_COLLISION |
    p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT  # or INCLUDE_PARENT if needed
)
robot = p.loadURDF("3dof/planar.urdf", useFixedBase=False, flags=flags)

marker_radius = 0.03
vis_red  = p.createVisualShape(p.GEOM_SPHERE, radius=marker_radius, rgbaColor=[1,0,0,1])   # COM now
vis_blue = p.createVisualShape(p.GEOM_SPHERE, radius=marker_radius, rgbaColor=[0,0,1,1])   # COM prev (optional)
com_now  = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                             baseVisualShapeIndex=vis_red,  basePosition=[0,0,0])
com_prev = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                             baseVisualShapeIndex=vis_blue, basePosition=[0,0,0])


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

# camera
# camera parameters
cam_target_pos = com1
cam_distance = 1
cam_yaw, cam_pitch, cam_roll = 0, 90, 0
cam_width, cam_height = 480, 360

cam_up, cam_up_axis_idx, cam_near_plane, cam_far_plane, cam_fov = [0, 0, 1], 2, 0.01, 100, 60

# precompute (optional)
cam_view_matrix = p.computeViewMatrixFromYawPitchRoll(
    cam_target_pos, cam_distance, cam_yaw, cam_pitch, cam_roll, cam_up_axis_idx
)
cam_projection_matrix = p.computeProjectionMatrixFOV(
    cam_fov, cam_width / float(cam_height), cam_near_plane, cam_far_plane
)

# avoid macro-block resize warning
vid = imageio_ffmpeg.write_frames(
    '3dof/results/vid.mp4', (cam_width, cam_height), fps=30,
    codec='h264', macro_block_size=1)
vid.send(None)

dt = 1/ 1200
p.setTimeStep(dt)
com_hist_t = []   
com_hist_x = []
com_hist_y = []  
com_hist_z = []

for t in range(1200*15):
    #camera
    if t % 40 == 0:
        w, h, rgba, depth, seg = p.getCameraImage(
            cam_width, cam_height, cam_view_matrix, cam_projection_matrix
            # no renderer arg needed; in DIRECT it falls back to TinyRenderer
        )

        # rgba may be list or bytes; make (H,W,4) uint8 then drop alpha
        if isinstance(rgba, (bytes, bytearray, memoryview)):
            arr = np.frombuffer(rgba, dtype=np.uint8)
        else:
            arr = np.asarray(rgba, dtype=np.uint8)
        frame = arr.reshape(h, w, 4)[:, :, :3]            # RGB
        vid.send(np.ascontiguousarray(frame))             # ensure C-contiguous

    # track com
    link_tot = (0, 0, 0)
    for i in range(p.getNumJoints(robot)):
        link_tot = np.add(link_tot, p.getLinkState(robot, i)[0])
    com2 = np.divide(link_tot, 3)
    # print(com2)
    p.addUserDebugLine(com1, com2, [1, 0, 0], 1, 0)

    t_sec = t * dt
    com_hist_t.append(t_sec)
    com_hist_x.append(float(com2[0]))
    com_hist_y.append(float(com2[1]))
    com_hist_z.append(float(com2[2]))
    
    z = 0.02  # tiny lift so it’s visible above the plane
    p.resetBasePositionAndOrientation(com_prev, [com1[0], com1[1], z], [0,0,0,1])
    p.resetBasePositionAndOrientation(com_now,  [com2[0], com2[1], z], [0,0,0,1])

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
    k = 5
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

vid.close()
p.disconnect()

# Time-series (x(t), y(t))
plt.figure(1)
plt.plot(com_hist_t, com_hist_x, label="COM x")
plt.plot(com_hist_t, com_hist_y, label="COM y")
plt.plot(com_hist_t, com_hist_y, label="COM z")
plt.xlabel("time [s]"); plt.ylabel("position [m]")
plt.legend(); plt.tight_layout()
plt.savefig("3dof/results/com_xy_vs_time.png", dpi=200)

# Planar trajectory (y vs x)
plt.figure(2)
plt.plot(com_hist_x, com_hist_y)
plt.xlabel("x [m]"); plt.ylabel("y [m]")
plt.axis("equal"); plt.tight_layout()
plt.savefig("3dof/results/com_path_xy.png", dpi=200)
plt.close("all")
