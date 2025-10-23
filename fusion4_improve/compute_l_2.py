#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
estimate_L_from_repo.py
针对你给出的样例格式：
- truth: ./data/ch3/gins.txt 每行类似：
  <time> px py pz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz
  （脚本只使用 time, qx,qy,qz,qw）
- encoder: ./data/ch3/10.txt（传感器融合日志），脚本只解析以 "ODOM" 开头的行：
  ODOM <time> <left_pulse> <right_pulse>

运行:
    python3 estimate_L_from_repo.py

输出:
    打印估计的 L (m), 使用的窗口数量, 残差 RMSE。
"""
import numpy as np
import math
import os

# 已知参数（你给的）
wheel_radius = 0.155
circle_pulse = 1024.0
odom_span = 0.1  # s

truth_path = "/home/keyirobot/Desktop/qixing_ws/learn/imu-wheel/imu_encoder_fusion/fusion4_improve/data/gins.txt"
encoder_path = "/home/keyirobot/Desktop/qixing_ws/learn/imu-wheel/imu_encoder_fusion/fusion4_improve/data/10.txt"

def read_gins_truth(path):
    """
    读取 gins.txt，提取每行的 time, qx,qy,qz,qw（如果行列少于7则跳过）
    返回 arrays: t, qx,qy,qz,qw
    """
    t_list = []
    qx_list = []; qy_list = []; qz_list = []; qw_list = []
    if not os.path.exists(path):
        raise FileNotFoundError(f"truth file not found: {path}")
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            toks = line.split()
            # 期待至少 1(time) + 7 (px py pz qx qy qz qw) = 8 列，总实际样例有 17 列
            if len(toks) < 8:
                continue
            try:
                t = float(toks[0])
                # index of quaternion in your sample: after time and 3 pos -> toks[4..7] are qx qy qz qw
                qx = float(toks[4]); qy = float(toks[5]); qz = float(toks[6]); qw = float(toks[7])
            except:
                continue
            t_list.append(t)
            qx_list.append(qx); qy_list.append(qy); qz_list.append(qz); qw_list.append(qw)
    return np.array(t_list), np.array(qx_list), np.array(qy_list), np.array(qz_list), np.array(qw_list)

def read_encoder_dom(path):
    """
    从 10.txt 中只读取以 'ODOM' 开头的行:
    ODOM <time> <left_pulse> <right_pulse>
    返回 arrays: t, left_pulse, right_pulse (累积或计数取决于文件)
    """
    t=[]; l=[]; r=[]
    if not os.path.exists(path):
        raise FileNotFoundError(f"encoder file not found: {path}")
    with open(path,'r') as f:
        for line in f:
            line=line.strip()
            if not line: continue
            if line.startswith("ODOM"):
                toks=line.split()
                # sample: ODOM 1624426287.19101906 0 0
                if len(toks) < 4: continue
                try:
                    tt = float(toks[1])
                    lp = float(toks[2])
                    rp = float(toks[3])
                except:
                    continue
                t.append(tt); l.append(lp); r.append(rp)
    return np.array(t), np.array(l), np.array(r)

def quat_to_yaw(qx,qy,qz,qw):
    # 假定四元数顺序为 (qx,qy,qz,qw) qw scalar
    siny = 2.0*(qw*qz + qx*qy)
    cosy = 1.0 - 2.0*(qy*qy + qz*qz)
    return np.arctan2(siny, cosy)

def resample_windows(enc_t, l_p, r_p, truth_t, truth_yaw, span=odom_span):
    # 建立 time 窗口，从共同起点到共同终点
    t0 = max(enc_t[0], truth_t[0])
    t1 = min(enc_t[-1], truth_t[-1])
    if t1 <= t0:
        return np.array([]), np.array([]), np.array([])
    edges = np.arange(t0, t1+1e-9, span)
    sL=[]; sR=[]; dtheta=[]
    yaw_unw = np.unwrap(truth_yaw)
    for i in range(len(edges)-1):
        a = edges[i]; b = edges[i+1]
        mask_e = (enc_t >= a) & (enc_t < b)
        mask_t = (truth_t >= a) & (truth_t < b)
        if mask_e.sum() < 2 or mask_t.sum() < 2:
            continue
        l0 = l_p[mask_e][0]; lf = l_p[mask_e][-1]
        r0 = r_p[mask_e][0]; rf = r_p[mask_e][-1]
        dp_l = float(lf - l0)
        dp_r = float(rf - r0)
        yaw0 = float(yaw_unw[mask_t][0]); yawf = float(yaw_unw[mask_t][-1])
        dyaw = yawf - yaw0
        # 转成米
        sL.append(dp_l * 2.0 * math.pi * wheel_radius / circle_pulse)
        sR.append(dp_r * 2.0 * math.pi * wheel_radius / circle_pulse)
        dtheta.append(dyaw)

        # ---- 在读取完 enc_t, and truth_t 后加入 ----
        print("enc time range: ", enc_t[0], " ... ", enc_t[-1])
        print("truth time range:", t[0], " ... ", t[-1])

        # 计算中心差（建议偏移）
        enc_mid = 0.5*(enc_t[0] + enc_t[-1])
        truth_mid = 0.5*(t[0] + t[-1])
        suggested_shift = truth_mid - enc_mid
        print(f"suggested time shift to align enc->truth: {suggested_shift:.6f} seconds")

        # 若你想自动应用（谨慎开启）
        auto_apply_shift = True   # 改为 False 则只打印不修改
        if auto_apply_shift:
            print("Applying time shift to encoder timestamps...")
            enc_t = enc_t + suggested_shift
            # 然后继续原来的处理（resample 等）

    return np.array(sL), np.array(sR), np.array(dtheta)

def estimate_L(sL,sR,dtheta,thr=1e-4):
    mask = np.abs(dtheta) > thr
    if mask.sum() == 0:
        raise RuntimeError("没有足够的角增量窗口，请检查时间重合与数据量")
    y = (sR - sL)[mask]
    x = dtheta[mask]
    L_hat = np.dot(y,x) / np.dot(x,x)
    residuals = y - L_hat*x
    rmse = np.sqrt(np.mean(residuals**2))
    return L_hat, rmse, mask.sum()

def main():
    print("读取 truth ...", truth_path)
    t,qx,qy,qz,qw = read_gins_truth(truth_path)
    print(f"truth frames: {len(t)}")
    print("读取 encoder (ODOM lines) ...", encoder_path)
    et, lp, rp = read_encoder_dom(encoder_path)
    print(f"encoder ODOM samples: {len(et)}")
    if len(t)==0 or len(et)==0:
        raise RuntimeError("truth 或 encoder 数据为空，请检查文件路径与格式")
    yaw = np.array([quat_to_yaw(a,b,c,d) for a,b,c,d in zip(qx,qy,qz,qw)])
    sL, sR, dtheta = resample_windows(et, lp, rp, t, yaw, span=odom_span)



    print(f"有效窗口: {len(sL)}")
    if len(sL)==0:
        raise RuntimeError("没有有效窗口，请调整 odom_span 或检查时间重合")
    L_hat, rmse, nwin = estimate_L(sL,sR,dtheta)
    print("=== 估计结果 ===")
    print(f"Estimated wheel base L = {L_hat:.6f} m")
    print(f"使用窗口数 = {nwin}, residual RMSE = {rmse:.6e} m")
    # 可选：保存 sL sR dtheta 到文件以便检查
    np.savetxt("lsr_dtheta_windows.txt", np.column_stack([sL,sR,dtheta]), fmt="%.9e",
               header="sL[m] sR[m] dtheta[rad]")
    print("窗口数据已保存到 lsr_dtheta_windows.txt（用于离线检查）")

if __name__ == "__main__":
    main()
    
