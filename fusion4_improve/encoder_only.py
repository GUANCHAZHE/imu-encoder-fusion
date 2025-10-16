#!/usr/bin/env python3
"""
读取相关的文件画出轨迹

用法：
python3 encoder_only.py
"""
import cv2
import numpy as np


# encoder 时间戳
# encoder time ? s θ ？

def main():
    image = np.zeros((700 , 1500, 3), dtype=np.uint8)  # 画布大小，宽高反的
    
    print(f"开始读取文件")
    path = "/home/keyirobot/Desktop/qixing_ws/learn/imu_encoder_fusion/encoder_only/imu_encoder_file.txt"

    pose = [0, 0, 0]
    pose_pre =  [0, 0, 0]

    count = 0
    with open(path, 'r') as f:
        for lines in f:
            print(f"count = {count}")
            line = lines.strip().split()
            # 处理异常
            if not line :
                continue

            data_type = line[0]
            if data_type == 'encoder':
                time_stamp = float(line[1])
                delta_s = float(line[3])
                delta_theta = float(line[4])
                pose[0] = pose_pre[0] + delta_s * np.cos(pose[2] + delta_theta / 2)
                pose[1] = pose_pre[1] + delta_s * np.sin(pose[2] + delta_theta / 2)
                pose[2] = pose_pre[2] + delta_theta

                pose_pre = pose.copy()

                # 开始画图
                cv2.circle(image,   # 绘制的图像
                # 绘制的原点  (0,0) 为左上角， 现在到 (800 450)
                (int(1000 - pose[0] * 20), int(500 - pose[1] * 20)), 
                3,           # 绘制小圆的半径 像素为单位
                (0, 0, 255), # 绘制小圆的颜色 黑色
                -1)          # 实心圆
                cv2.imshow("trajectory", image)
                cv2.waitKey(5)

            else:
                continue
            count += 1
        cv2.waitKey(0)
        f.close()

if __name__ == "__main__":
    main()