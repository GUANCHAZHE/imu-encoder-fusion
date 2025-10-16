#!/usr/bin/env python3
"""
Python version of true_odom/main.cpp

Reads odom_data.txt (each line contains timestamp and pose values),
draws the trajectory using OpenCV and displays it.

Usage:
    python3 true_odom/main.py [path/to/odom_data.txt]

This mirrors the behavior of the original C++ program.
"""
from re import L
import sys
import os
import cv2
import argparse
from matplotlib import pyplot as plt
import numpy as np

N_encoder = 1024.0  # 编码器每圈脉冲数
r_circle = 0.155    # 轮子半径 m
l =  0.9            # 车轮间距 m

l_pulse = []
r_pulse = []
s_lpulse = []
s_rpulse = []
delta_s = []
delta_xita = []

x = []
y = []
xita = []
# 转换sad格式的数据
def data_convert():
    path_sad = "fusion4_improve/data/10_small_sad.txt"
    path_output = "fusion4_improve/data/10_small_output_converted.txt"
    path_odom = "fusion4_improve/data/10_small_odom.txt"
    # 读取相关的数据保存为列表
    with open(path_sad, 'r') as f, open(path_output, 'w') as fout:
        for lines in f:
            line = lines.strip().split()
            if not line:
                continue
            data_type = line[0]
            if data_type == 'ODOM':
                timestamp = float(line[1])
                l_pulse.append(float(line[2]))
                r_pulse.append(float(line[3]))
            else:
                continue
        f.close()
        fout.close()
    
    # 开始数据转换
    for i in range(len(l_pulse)):
        if i == 0:
            s_lpulse.append(0.0)
            s_rpulse.append(0.0)
        else:
            # 单位时间内的增量为 δs = (脉冲差值) * 2π *r / N
            s_lpulse.append((l_pulse[i] - l_pulse[i-1]) * 2 * np.pi * r_circle / N_encoder)
            s_rpulse.append((r_pulse[i] - r_pulse[i-1]) * 2 * np.pi * r_circle / N_encoder)
        delta_s.append((s_lpulse[i] + s_rpulse[i]) / 2.0)
        delta_xita.append((s_rpulse[i] - s_lpulse[i]) / l)
    
    # 奖结果写入到文本中
    with open(path_output, 'w') as fout:
        for i in range(len(l_pulse)):
            fout.write(f"odom {delta_s[i]} {delta_xita[i]}\n")
        fout.close()

    # 将delta_s delta_xita 转换为 pose [x y xita]
    pose = [0.0, 0.0, 0.0]
    pose_pre = [0.0, 0.0, 0.0]

    with open(path_odom, 'w') as fout:
        for i in range(len(delta_s)):
            pose[0] = pose_pre[0] + delta_s[i] * np.cos( pose_pre[2] )
            pose[1] = pose_pre[1] + delta_s[i] * np.sin( pose_pre[2] )
            pose[2] = pose_pre[2] + delta_xita[i]

            x.append(pose[0])
            y.append(pose[1])
            xita.append(pose[2])

            pose_pre = pose.copy()

            fout.write(f"0.0 {pose[0]} {pose[1]} {pose[2]}\n")
        fout.close()
    print(f"数据转换完成，结果保存在 {path_output} 中")

    plt.plot(x, y)
    plt.xlabel("x/m")
    plt.ylabel("y/m")
    plt.title("true odom trajectory")
    plt.axis("equal")
    plt.grid()
    plt.show()

# def run(data_path: str):
#     if not os.path.isfile(data_path):
#         print(f"Failed to open the simulation file: {data_path}")
#         return 1

#     print(f"Reading data from: {data_path}")
    
#     # image setup (same size as C++ version)
#     # create blank image directly (900x900, 3 channels)
#     image = np.zeros((900, 900, 3), dtype=np.uint8)

#     # draw origin
#     cv2.circle(image, (450, 450), 3, (0, 0, 255), -1)

#     cv2.namedWindow('trajectory', cv2.WINDOW_NORMAL)
#     cv2.imshow('trajectory', image)
#     cv2.waitKey(1)

#     count = 0

#     try:
#         with open(data_path, 'r') as f:
#             for line in f:
#                 line = line.strip()
#                 if not line:
#                     continue

#                 toks = line.split()
#                 # original C++ reads first token then reads two tokens into pose[0], pose[1]
#                 if len(toks) < 3:
#                     continue

#                 # parse x, y from tokens[1], tokens[2]
#                 try:
#                     x = float(toks[1])
#                     y = float(toks[2])
#                 except ValueError:
#                     # skip malformed lines
#                     continue

#                 # print(f"frame: {count}, x = {x}, y = {y}")

#                 px = int(round(800 - x * 15))
#                 py = int(round(450 - y * 15))

#                 # bound check
#                 if 0 <= px < image.shape[1] and 0 <= py < image.shape[0]:
#                     cv2.circle(image, (px, py), 1, (0, 255, 0), -1)

#                 cv2.imshow('trajectory', image)
#                 # wait 10 ms like C++
#                 key = cv2.waitKey(10) & 0xFF
#                 if key == 27:  # ESC to exit early
#                     break

#                 count += 1

#     except Exception as e:
#         print('Error while reading file:', e)
#         return 2

#     # final pause
#     cv2.waitKey(0)
#     return 0


# def parse_args():
#     path_test = "/home/keyirobot/Desktop/qixing_ws/learn/imu-wheel/imu_encoder_fusion/fusion4_improve/data/odom_pose_output.txt"
#     ap = argparse.ArgumentParser()
#     ap.add_argument('datafile', nargs='?', default='data/odom_data.txt', help='path to odom_data.txt')
#     # ap.add_argument('datafile', nargs='?', default=path_test, help='path to odom_data.txt')
    
#     return ap.parse_args()


if __name__ == '__main__':
    # args = parse_args()
    # data_path = args.datafile
    # # if given relative path, prefer file in same directory as script
    # if not os.path.isabs(data_path):
    #     script_dir = os.path.dirname(__file__)
    #     candidate = os.path.join(script_dir, data_path)
    #     if os.path.exists(candidate):
    #         data_path = candidate

    # sys.exit(run(data_path))
    data_convert()