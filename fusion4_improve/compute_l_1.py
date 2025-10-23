# 更新，理解错误了
# 这个所谓的r并不是轮子半径，当前速度的转弯半径


# 这是一个读取sad的数据，其中里程计Odom的脉冲， pl, pr
# 然后计算左右速度，vl = (pl/ N) * 2*PI*r / dt,vr = (pr/ N) * 2*PI*r / dt
# 通过速度可以得到轮间距，l = (vr - vl) * 2r/ (vr + vl)
# 最后通过计算平均间距得到结果 l_ = sum(l) / len(l)

# double velo_l = options_.wheel_radius_ * (odom.left_pulse_ / options_.circle_pulse_) * 2 * M_PI / options_.odom_span_;   //3.76
# double velo_r = options_.wheel_radius_ * (odom.right_pulse_ / options_.circle_pulse_) * 2 * M_PI / options_.odom_span_;  //3.76


###################### V1
# import numpy as np
# from os import path

# circle_pulse = 1024.0
# wheel_radius_ = 0.155
# odom_span_ = 0.1

# l_list = []
# def compute_l():
#     # path_sad = "fusion4_improve/data/10_small_sad.txt"
#     path_sad = "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/data/ch3/10.txt"
#     with open(path_sad, 'r') as f:
#         for lines in f:
#             line = lines.strip().split()

#             type = line[0]
#             if type == 'ODOM':
#                 pl = float(line[2])
#                 pr = float(line[3])
#                 if pl == 0 and pr == 0:
#                     continue
#                 if pl == pr:
#                     continue
#                 vl = (pl / circle_pulse) * 2 * np.pi * wheel_radius_ / odom_span_
#                 vr = (pr / circle_pulse) * 2 * np.pi * wheel_radius_ / odom_span_
#                 l = (vr - vl) * 2 * wheel_radius_ / (vr + vl)
#                 l_list.append(abs(l))
#     l_average = sum(l_list) / len(l_list)
#     print(f"计算得到的轮间距 l = {l_average} m")
#     return l_average

# if __name__ == "__main__":
#     compute_l()
#     print("done")



###################### V2
#!/usr/bin/env python3
"""
estimate_wheel_base.py

Usage:
    python3 estimate_wheel_base.py --encoder encoder.txt --truth truth.txt
    或者用于测试:
    python3 estimate_wheel_base.py --simulate

Expectations:
- encoder.txt: whitespace columns: time(s)  left_pulse  right_pulse
- truth.txt:   whitespace columns: time(s)  px  py  pz  qx  qy  qz  qw
               (qw = scalar component)
If your truth file is in other format (e.g. save_vec3 / save_quat lines), convert it to the above table.
"""
import numpy as np
import argparse
import math

wheel_radius = 0.155
circle_pulse = 1024.0
odom_span = 0.1  # seconds, same as your odom_span_

def quat_to_yaw(qx, qy, qz, qw):
    # Assuming quaternion (qx,qy,qz,qw) where qw is scalar
    # yaw (z) from quaternion
    # formula: yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    # careful with ordering; this implementation assumes (x,y,z,w) input
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def read_encoder(path):
    # returns arrays time, left_pulse, right_pulse
    data = np.loadtxt(path)
    # expect columns: t, l, r
    return data[:,0], data[:,1], data[:,2]

def read_truth(path):
    # returns arrays time, px, py, pz, qx, qy, qz, qw
    data = np.loadtxt(path)
    if data.shape[1] < 8:
        raise RuntimeError("truth file must have at least 8 columns: t px py pz qx qy qz qw")
    return data[:,0], data[:,1], data[:,2], data[:,3], data[:,4], data[:,5], data[:,6], data[:,7]

def resample_deltas(enc_t, l_p, r_p, truth_t, truth_yaw, span=odom_span):
    """
    Partition timeline into contiguous windows of length `span`.
    For each window compute:
      delta_pulse_L, delta_pulse_R (using first and last sample within window)
      delta_yaw (unwrap before differencing)
    Returns arrays of sL, sR, dtheta
    """
    t0 = max(enc_t[0], truth_t[0])
    t1 = min(enc_t[-1], truth_t[-1])
    # build window boundaries
    edges = np.arange(t0, t1, span)
    sL = []
    sR = []
    dtheta = []
    # unwrap yaw
    yaw_unwrapped = np.unwrap(truth_yaw)
    for i in range(len(edges)-1):
        a = edges[i]
        b = edges[i+1]
        # encoder indices inside window
        mask_e = (enc_t >= a) & (enc_t < b)
        mask_t = (truth_t >= a) & (truth_t < b)
        if mask_e.sum() < 2 or mask_t.sum() < 2:
            continue
        # use first and last sample in window to compute delta pulse
        l0 = l_p[mask_e][0]; lf = l_p[mask_e][-1]
        r0 = r_p[mask_e][0]; rf = r_p[mask_e][-1]
        # compute delta pulses
        dp_l = float(lf - l0)
        dp_r = float(rf - r0)
        # compute delta yaw from truth
        yaw0 = float(yaw_unwrapped[mask_t][0])
        yawf = float(yaw_unwrapped[mask_t][-1])
        dyaw = yawf - yaw0
        # push
        sL.append(dp_l * 2.0 * np.pi * wheel_radius / circle_pulse)
        sR.append(dp_r * 2.0 * np.pi * wheel_radius / circle_pulse)
        dtheta.append(dyaw)
    return np.array(sL), np.array(sR), np.array(dtheta)

def estimate_L_least_squares(sL, sR, dtheta, min_abs_dtheta=1e-4):
    # filter small dtheta windows
    mask = np.abs(dtheta) > min_abs_dtheta
    if mask.sum() == 0:
        raise RuntimeError("No windows with sufficient dtheta to estimate L.")
    y = (sR - sL)[mask]
    x = dtheta[mask]
    L_hat = np.dot(y, x) / np.dot(x, x)
    # also compute residuals and std
    residuals = y - L_hat * x
    rmse = np.sqrt(np.mean(residuals**2))
    return L_hat, rmse, mask.sum()

def run(encoder_file, truth_file):
    enc_t, l_p, r_p = read_encoder(encoder_file)
    t, px, py, pz, qx, qy, qz, qw = read_truth(truth_file)
    # compute yaw array
    yaw = np.array([quat_to_yaw(qxi, qyi, qzi, qwi) for qxi,qyi,qzi,qwi in zip(qx,qy,qz,qw)])
    sL, sR, dtheta = resample_deltas(enc_t, l_p, r_p, t, yaw, span=odom_span)
    print(f"Got {len(sL)} windows for estimation (span={odom_span}s).")
    if len(sL) == 0:
        raise RuntimeError("No valid windows - check overlap and sampling.")
    L_hat, rmse, n = estimate_L_least_squares(sL, sR, dtheta)
    print(f"Estimated wheel base L = {L_hat:.6f} m  (using {n} windows), residual RMSE = {rmse:.6e} m")
    return L_hat

def simulate_and_run():
    # generate synthetic data with true L=0.9
    L_true = 0.9
    t = np.arange(0, 60, 0.01)  # 100 Hz
    # let's synthesize some motion: rotate back and forth and move
    # generate left/right wheel angular velocities (rad/s)
    # pulses are cumulative count
    left_pulses = np.zeros_like(t)
    right_pulses = np.zeros_like(t)
    yaw = np.zeros_like(t)
    x = np.zeros_like(t); y = np.zeros_like(t)
    # simple motion: alternate segments where robot rotates in place and moves
    pulse_per_rev = circle_pulse
    wheel_circ = 2*np.pi*wheel_radius
    # create angular wheel linear speeds (m/s)
    for i in range(1,len(t)):
        ti = t[i]
        # create some commanded angular velocity for body:
        if (ti % 10) < 5:
            v = 0.2   # forward
            omega = 0.05  # small yaw rate
        else:
            v = 0.0
            omega = 0.2 * (1 if (ti%20)<10 else -1)
        # convert body v,omega to wheel linear speeds
        v_r = v + omega * L_true / 2.0
        v_l = v - omega * L_true / 2.0
        # convert to pulses increment
        dp_r = (v_r * (t[i]-t[i-1])) / wheel_circ * pulse_per_rev
        dp_l = (v_l * (t[i]-t[i-1])) / wheel_circ * pulse_per_rev
        right_pulses[i] = right_pulses[i-1] + dp_r
        left_pulses[i] = left_pulses[i-1] + dp_l
        # integrate pose
        yaw[i] = yaw[i-1] + omega*(t[i]-t[i-1])
        x[i] = x[i-1] + v * math.cos(yaw[i-1])*(t[i]-t[i-1])
        y[i] = y[i-1] + v * math.sin(yaw[i-1])*(t[i]-t[i-1])
    # write temp files
    enc = np.column_stack([t, left_pulses, right_pulses])
    truth_q = np.zeros((len(t),8))
    # convert yaw -> quaternion (zx convention)
    for i, yi in enumerate(yaw):
        cy = math.cos(yi*0.5); sy = math.sin(yi*0.5)
        # assuming roll=pitch=0, yaw=yi => quaternion (qx,qy,qz,qw) = (0,0,sy,cy)
        truth_q[i,0] = t[i]; truth_q[i,1]=x[i]; truth_q[i,2]=y[i]; truth_q[i,3]=0.0
        truth_q[i,4]=0.0; truth_q[i,5]=0.0; truth_q[i,6]=sy; truth_q[i,7]=cy
    np.savetxt("sim_encoder.txt", enc, fmt="%.6f")
    np.savetxt("sim_truth.txt", truth_q, fmt="%.6f")
    print("Simulated files written: sim_encoder.txt, sim_truth.txt")
    return run("sim_encoder.txt", "sim_truth.txt")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--encoder", help="encoder file: t l_pulse r_pulse")
    ap.add_argument("--truth", help="truth file: t px py pz qx qy qz qw")
    ap.add_argument("--simulate", action="store_true", help="simulate sample data")
    args = ap.parse_args()
    if args.simulate:
        L = simulate_and_run()
        print("Simulated ground-truth L was 0.9 m.")
    else:
        if not args.encoder or not args.truth:
            print("Provide --encoder and --truth or use --simulate")
            raise SystemExit(1)
        L = run(args.encoder, args.truth)
    print("Done.")
