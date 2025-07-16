#!/usr/bin/env python3

import rospy
import numpy as np
import pandas as pd
import casadi as ca
import torch
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
import matplotlib.pyplot as plt

# Load residual model
res_model = torch.jit.load("residual_model_output/residual_model_traced.pt")
res_model.eval()

def get_residual(x_np, y_np, yaw_np, v_np, delta_np):
    inp = torch.tensor([[x_np, y_np, yaw_np, v_np, delta_np]], dtype=torch.float32)
    with torch.no_grad():
        return res_model(inp).numpy()[0]

# Load waypoints
waypoints = pd.read_csv("wps.csv", header=None).values[:, :3]

# MPC settings
N = 10
dt = 0.1
L = 1.75
Q = ca.diagcat(10, 10, 3)  # position and heading
Q_cte = 20.0               # explicit CTE penalty
R = ca.diagcat(0.5, 5.0)   # control effort penalty

v_max = 5.56
v_min = 0.0
delta_max = np.radians(25)
delta_min = -np.radians(25)

# CasADi symbols
x, y, yaw = ca.SX.sym('x'), ca.SX.sym('y'), ca.SX.sym('yaw')
v, delta = ca.SX.sym('v'), ca.SX.sym('delta')
states = ca.vertcat(x, y, yaw)
controls = ca.vertcat(v, delta)

def dynamics_fn(states, controls):
    x, y, yaw = states[0], states[1], states[2]
    v, delta = controls[0], controls[1]
    x_next = x + v * ca.cos(yaw) * dt
    y_next = y + v * ca.sin(yaw) * dt
    yaw_next = yaw + (v / L) * ca.tan(delta) * dt
    return ca.vertcat(x_next, y_next, yaw_next)

# Optimization problem
opt_vars = ca.SX.sym('X', 3, N+1)
opt_controls = ca.SX.sym('U', 2, N)
P = ca.SX.sym('P', 3 + N * 3)
obj = 0
constraints = [opt_vars[:, 0] - P[0:3]]

for k in range(N):
    st = opt_vars[:, k]
    con = opt_controls[:, k]
    st_next = opt_vars[:, k + 1]
    ref = P[3 + k * 3: 3 + (k + 1) * 3]
    obj += ca.mtimes([(st - ref).T, Q, (st - ref)]) + ca.mtimes([con.T, R, con])
    cte = ca.sin(ref[2] - st[2]) * ca.norm_2(st[0:2] - ref[0:2])
    obj += Q_cte * cte**2
    constraints.append(st_next - dynamics_fn(st, con))

nlp = {'f': obj,
       'x': ca.vertcat(ca.reshape(opt_vars, -1, 1), ca.reshape(opt_controls, -1, 1)),
       'g': ca.vertcat(*constraints),
       'p': P}
opts = {'ipopt.print_level': 0, 'print_time': 0}
solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

# Bounds
lbx, ubx = [], []
for _ in range(N + 1):
    lbx += [-ca.inf] * 3
    ubx += [ca.inf] * 3
for _ in range(N):
    lbx += [v_min, delta_min]
    ubx += [v_max, delta_max]

cte_log = []
last_idx = [0]
start_time = None

def get_ref_traj(current_pose):
    global waypoints
    dists = np.linalg.norm(waypoints[:, :2] - current_pose[:2], axis=1)
    sorted_idxs = np.argsort(dists)
    for idx in sorted_idxs:
        dx = waypoints[idx, 0] - current_pose[0]
        dy = waypoints[idx, 1] - current_pose[1]
        heading = np.array([np.cos(current_pose[2]), np.sin(current_pose[2])])
        vec = np.array([dx, dy])
        if np.dot(vec, heading) > 0:
            idx = min(idx, len(waypoints) - N)
            return waypoints[idx:idx+N]
    return np.tile(waypoints[-1], (N, 1))

def odom_callback(msg):
    global start_time
    if start_time is None:
        start_time = rospy.get_time()
    elapsed = rospy.get_time() - start_time

    if elapsed < 1.0:
        # rospy.loginfo(" Warmup phase — residuals OFF, neutral control")
        drive_msg = AckermannDrive()
        drive_msg.speed = 1.0
        drive_msg.steering_angle = 0.0
        pub.publish(drive_msg)
        if hasattr(odom_callback, "prev_sol"):
            del odom_callback.prev_sol
        return

    apply_residual = True

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw = 2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    if apply_residual:
        dx_r, dy_r, dyaw_r = get_residual(x, y, yaw, 0.0, 0.0)
        x += 0.2 * np.clip(dx_r, -0.05, 0.05)
        y += 0.2 * np.clip(dy_r, -0.05, 0.05)
        yaw += 0.2 * np.clip(dyaw_r, -0.05, 0.05)

    current_state = np.array([x, y, yaw])
    ref_traj = get_ref_traj(current_state)
    P_val = np.concatenate([current_state, ref_traj.flatten()])

    if hasattr(odom_callback, "prev_sol"):
        x0 = odom_callback.prev_sol
    else:
        init_state = np.tile(current_state, (N+1, 1)).flatten()
        init_controls = np.zeros((N * 2,))
        x0 = np.concatenate([init_state, init_controls])[:, None]

    sol = solver(x0=x0, p=P_val, lbg=0, ubg=0, lbx=lbx, ubx=ubx)
    odom_callback.prev_sol = sol['x']

    u_opt = sol['x'][(N+1)*3:(N+1)*3 + 2]
    v_cmd = float(u_opt[0])
    delta_cmd = float(u_opt[1])
    delta_cmd = np.clip(delta_cmd, delta_min, delta_max)
    if v_cmd < 0.1:
        v_cmd = 0.1

    # Cross-track error for logging
    dx = ref_traj[0, 0] - x
    dy = ref_traj[0, 1] - y
    yaw_ref = ref_traj[0, 2]
    alpha = np.arctan2(np.sin(yaw_ref - yaw), np.cos(yaw_ref - yaw))
    L = np.linalg.norm([dx, dy])
    cte = np.sin(alpha) * L
    cte_log.append(cte)

    # Adaptive velocity scaling
    v_cmd *= (1.0 - abs(delta_cmd) / delta_max)
    v_cmd = max(1.5, min(v_cmd, v_max))

    drive_msg = AckermannDrive()
    drive_msg.speed = v_cmd
    drive_msg.steering_angle = delta_cmd
    pub.publish(drive_msg)

    if len(cte_log) % 50 == 0:
        np.savetxt("cross_track_error.csv", cte_log, delimiter=",")
        plt.clf()
        plt.plot(cte_log)
        plt.title("Cross-Track Error Over Time")
        plt.xlabel("Step")
        plt.ylabel("Error (m)")
        plt.grid()
        plt.savefig("cross_track_error.png")

    rospy.loginfo(f"[Control] v: {v_cmd:.2f} m/s | δ: {np.degrees(delta_cmd):.1f}° | CTE: {cte:.3f}")

rospy.init_node("mpc_controller_node")
pub = rospy.Publisher("/gem/ackermann_cmd", AckermannDrive, queue_size=10)
rospy.Subscriber("/gem/base_footprint/odom", Odometry, odom_callback)
# rospy.loginfo(" MPC Controller Node with CTE Penalty and Warmup Fix Started")
rospy.spin()
