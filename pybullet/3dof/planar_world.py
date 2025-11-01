import os
import pybullet as p
import pybullet_data
import numpy as np
import numdifftools as nd
import imageio_ffmpeg
import matplotlib.pyplot as plt
import os
from datetime import datetime
import time

# terminal inputs
control = input("Torque or force control? [t/f]: ").strip().lower()
while control not in ("t", "f"):
    control = input("Please type t or f: ").strip().lower()

ans = input("Record data? [y/n]: ").strip().lower()
while ans not in ("y", "n"):
    ans = input("Please type y or n: ").strip().lower()

while True:
    user_entry = input("Simulation time? [seconds]: ").strip()
    if user_entry.isdigit():
        sim_time = int(user_entry)
        break
    else:
        print("Please enter a whole number.")

if ans == "y":
    folder_name = input("Folder name: ").strip().lower()
    record_data = True
else:
    record_data = False

if record_data:
    # Get the absolute path of the folder that this script lives in
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Create a results folder inside the same folder as the script (3dof/results)
    results_root = os.path.join(script_dir, "results")

    # Create a results folder with a timestamp
    run_name = f"{folder_name}_{datetime.now().strftime('%d_%m')}"
    results_dir = os.path.join(results_root, run_name)
    os.makedirs(results_dir, exist_ok=True)

if record_data:
    p.connect(p.DIRECT)
else:
    p.connect(p.GUI)

# load simulation environment
p.setGravity(0,0,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0,0,-5], useFixedBase=True)

flags = (
    p.URDF_USE_INERTIA_FROM_FILE |
    p.URDF_USE_SELF_COLLISION |
    p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT  # or INCLUDE_PARENT if needed
)
robot = p.loadURDF("3dof/urdf/planar_real.urdf", useFixedBase=False, flags=flags)

# set intial position
p.resetJointState(robot, 1, 3.14/4)
p.resetJointState(robot, 2, 3.14/4)

# find and plot initial com
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

marker_radius = 0.03
vis_red  = p.createVisualShape(p.GEOM_SPHERE, radius=marker_radius, rgbaColor=[1,0,0,1])   # COM now
vis_blue = p.createVisualShape(p.GEOM_SPHERE, radius=marker_radius, rgbaColor=[0,0,1,1])   # COM prev (optional)
com_now  = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                             baseVisualShapeIndex=vis_red,  basePosition=com1)
com_prev = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                             baseVisualShapeIndex=vis_blue, basePosition=com1)

# camera settings
if record_data:
    # camera
    # camera parameters
    cam_target_pos = com1
    cam_distance = 1
    cam_yaw, cam_pitch, cam_roll = 0, -90, 0
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
    video_path = os.path.join(results_dir, "vid.mp4")
    vid = imageio_ffmpeg.write_frames(
        video_path, (cam_width, cam_height), fps=30,
        codec='h264', macro_block_size=1)
    vid.send(None)

    com_hist_t = []   
    com_hist_x = []
    com_hist_y = []  
    com_hist_z = []

dt = 1/ 2400
freq = int(1 / dt)
p.setTimeStep(dt)

# debug x1 and x2
x1_pt = [0, 0, 0]
x2_pt = [0, 0, 0]
line_id = p.addUserDebugLine(x1_pt, x2_pt, [1, 0, 0], lineWidth=2, lifeTime=0)

t_hist = []
q0_hist = []   
q1_hist = []
q2_hist = []  
orn_hist = []

for t in range(freq*sim_time):
    #camera
    if record_data:
        if t % (int(freq/30)) == 0:
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

    # com plots
    if record_data:
        t_sec = t * dt
        com_hist_t.append(t_sec)
        com_hist_x.append(float(com2[0]))
        com_hist_y.append(float(com2[1]))
        com_hist_z.append(float(com2[2]))
    
    p.resetBasePositionAndOrientation(com_prev, [com1[0], com1[1], 0.03], [0,0,0,1])
    com1 = com2

    # virtual mechanism calcs
    # get q1 and q2
    q1 = p.getJointState(robot, 1)[0]
    q2 = p.getJointState(robot, 2)[0]
    
    # find x1, x2 and com in terms of q1 and q2
    # assume base position and orientation is measured
    pos, orn = p.getBasePositionAndOrientation(robot)
    orn2 = p.getEulerFromQuaternion(orn)
    q0 = orn2[2]
    t_sec = t * dt
    t_hist.append(t_sec)
    q0_hist.append(q0) 
    q1_hist.append(q1)
    q2_hist.append(q2) 
    orn_hist.append(orn2[2])

    # x1 in world frame
    x1 = np.array(pos)

    # x2 in x1 frame
    x2 = [-(0.3*np.sin(q1) + 0.3*np.sin(q1 + q2)), 
                0.3 + 0.3*np.cos(q1) + 0.3*np.cos(q1 + q2)]
    
    # com in x1 frame
    com = [-(0.15*np.sin(q1) + 0.05*np.sin(q1 + q2)), 
           0.25 + 0.15*np.cos(q1) + 0.05*np.cos(q1 + q2)]
    
    # vectors
    x1 = np.array([[x1[0]], [x1[1]], [0.03]])
    x2 = np.array([[x2[0]], [x2[1]], [0.03], [1]])
    com = np.array([[com[0]], [com[1]], [0.03], [1]])

    # x1 frame to world transformation matrix (4x4)
    T02 = np.array([[np.cos(q0), -np.sin(q0), 0, x1.flatten()[0]],
                 [np.sin(q0), np.cos(q0), 0, x1.flatten()[1]], 
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

    # shorten vectors (2x1)
    x1 = x1[:-1]
    x2 = x2[:-1]

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

    if control == "t":
        jac = "analytical"
        if jac == "analytical":
            # find t1 and t2
            # dx1dq1 = [0.15*np.cos(q1) + 0.05*np.cos(q1 + q2), 
            #             0.15*np.sin(q1) - 0.05*np.sin(q1 + q2)]
            # dx1dq2 = [0.05*np.cos(q1 + q2), 0.05*np.sin(q1 +q2)]
            # dh1dqT = np.array([dx1dq1, dx1dq2])

            dx2dq1 = np.array([-0.3*np.cos(q1) - 0.3*np.cos(q1 + q2),
                                -0.3*np.sin(q1) - 0.3*np.sin(q1 + q2)])
            dx2dq2 = np.array([-0.3*np.cos(q1 + q2), -0.3*np.sin(q1 +q2)])
            dh2dqT = np.array([dx2dq1, dx2dq2])

            R02 = [[np.cos(q0), -np.sin(q0)],
                 [np.sin(q0), np.cos(q0)]]
            J2 = np.matmul(R02, dh2dqT.T)

            # control motors
            #t = np.matmul(dh1dqT, f1) + np.matmul(dh2dqT, f2)
            t = np.matmul(J2.T, f2)

        elif jac == "numerical":
            def x1_func(q):
                pos, orn = p.getBasePositionAndOrientation(robot)
                return np.array(pos)[:-1]
            def x2_func(q):
                q1 = q[0]
                q2 = q[1]
                pos, orn = p.getBasePositionAndOrientation(robot)
                orn2 = p.getEulerFromQuaternion(orn)
                q0 = orn2[2]

                x1 = np.array(pos)
                x2 = [-(0.3*np.sin(q1) + 0.3*np.sin(q1 + q2)), 
                         0.3 + 0.3*np.cos(q1) + 0.3*np.cos(q1 + q2)]
                
                
                x1 = np.array([[x1[0]], [x1[1]], [0.03]])
                x2 = np.array([[x2[0]], [x2[1]], [0.03], [1]])
                
                T02 = np.array([[np.cos(q0), -np.sin(q0), 0, x1.flatten()[0]],
                 [np.sin(q0), np.cos(q0), 0, x1.flatten()[1]], 
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
                x2w = np.matmul(T02, x2)

                x2 = x2w[:-2]
                return x2.flatten()
            
            # J1 = nd.Jacobian(x1_func)([q1, q2])
            # print(J1)
            J2 = nd.Jacobian(x2_func)([q1, q2])
            #print(J2)

            t = np.matmul(J2.T, f2)
        else:
            print("jacobian error") 
            p.disconnect()

        p.setJointMotorControlArray(robot, [1, 2], p.TORQUE_CONTROL, forces = t.flatten())
    elif control == "f":
        f1 = np.vstack((f1, [0]))
        f2 = np.vstack((f2, [0]))
        x1 = np.vstack((x1, [0.03]))
        x2 = np.vstack((x2, [0.03]))
        p.applyExternalForce(robot, 0, f1.flatten(), x1.flatten(), p.WORLD_FRAME)
        p.applyExternalForce(robot, 2, f2.flatten(), x2.flatten(), p.WORLD_FRAME) 
    else:
        print("control error") 
        p.disconnect()

    p.stepSimulation()

if record_data:
    vid.close()
    p.disconnect()

    # COM Trajectory
    x = np.asarray(com_hist_x)
    y = np.asarray(com_hist_y)
    z = np.asarray(com_hist_z)
    t = np.asarray(com_hist_t)

    # --- XY path plot ---
    fig, ax = plt.subplots()
    ax.plot(x, y, linewidth=1)

    # recenter around mean COM
    x_mean, y_mean = np.mean(x), np.mean(y)
    span = 0.05
    ax.set_xlim(x_mean - span, x_mean + span)
    ax.set_ylim(y_mean - span, y_mean + span)

    # equal aspect, grid, origin lines
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, which='both', alpha=0.4)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("COM path in XY")
    fig.tight_layout()
    fig.savefig(os.path.join(results_dir, "com_path_xy.png"), dpi=200)

    # --- Boxplots of deviation from mean (x, y, z) on the same y-axis ---
    x_dev = x - x[0]
    y_dev = y - y[0]
    z_dev = z - z[0]

    fig, axes = plt.subplots(1, 3, figsize=(9, 3), sharey=True)
    for ax, data, label in zip(axes, [x_dev, y_dev, z_dev], ["x", "y", "z"]):
        ax.boxplot([data*1000], showfliers=False)
        ax.axhline(0, linewidth=0.8)              # common zero line
        ax.grid(True, axis="y", alpha=0.4)
        ax.set_title(f"{label} (Δ from mean)")

    axes[0].set_ylabel("Deviation (mm)")
    fig.suptitle("COM distributions")
    fig.tight_layout()
    fig.savefig(os.path.join(results_dir, "com_boxplots_dev.png"), dpi=200)

else:
    p.disconnect()

# plot torque and base angles for debugging
# fig, axs = plt.subplots(3, 1, sharex=True, figsize=(8, 8))

# axs[0].plot(t_hist, np.asarray(orn_hist)*(180/np.pi))
# axs[0].set_ylabel('base [deg]')

# axs[1].plot(t_hist, np.asarray(q1_hist)*(180/np.pi))
# axs[1].set_ylabel('q1 [deg]')

# axs[2].plot(t_hist, np.asarray(q2_hist)*(180/np.pi))
# axs[2].set_ylabel('q2 [deg]')

# plt.tight_layout()
# plt.show()