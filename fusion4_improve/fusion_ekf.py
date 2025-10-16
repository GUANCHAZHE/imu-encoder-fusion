import pandas as pd
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt

from eskf import ESKF


def read_data(path):
    """
    读取数据
    """
    data = pd.read_csv(path, header=None, sep=' ')
    return data

# 转换sad格式的数据

def main():

    print("--------------------- EKF FUSION--------------------")

    # For displaying
    image = np.zeros((700 , 1500, 3), dtype=np.uint8)  # 画布大小，宽高反的

    # Instantiate EKF filter
    ekf = ESKF()

    path = "/home/keyirobot/Desktop/qixing_ws/learn/imu_encoder_fusion/fusion3/data/imu_encoder_file_sorted.txt"

    path_sad = "fusion4_improve/data/10_small_sad.txt"

    imu_data = np.zeros(4)
    encoder_data = np.zeros(6)
    delta_theta_list = []       # 用于展示他们之间的区别
    time_list = []              # 用于展示他们之间的区别
    pre_time = 0.0         # 默认初始时间
    is_init =False         # 积分中值 第一次出现的问题
    pre_gz = 0.0           # 默认的初始的角加速度
    delta_theta = 0.0      # 默认的初始的角度
    
    count = 0              # 计数 读取多少数据
    count_imu = 0          # 读取多少IMU数据
    count_encoder = 0      # 读取多少编码器数据
    #### 打印初始化参数
    print(f"pre_time = {pre_time}, pre_gz = {pre_gz}, theta = {delta_theta}")
    
    # open output file to write odom poses (timestamp x y)
    output_file_path = "fusion4_improve/data/odom_pose_output.txt"
    fout = open(output_file_path, 'w')

    with open(path, 'r') as file:
        # 读取txt的数据，用作后续处理
        for lines in file:
            print("开始循环计数 count", count)

            line = lines.strip().split()
            if not line:
                continue

            data_type = line[0]
            timestamp = float(line[1])

            # 正常的数据处理流 imu encoder imu imu imu encoder
            if data_type == 'encoder':
                print(f"开始处理编码器数据")
                encoder_data = [timestamp, 
                                float(line[2]), 
                                float(line[3]),  #2 s
                                float(line[4]),  #3 θ
                                float(line[5]), 
                                float(line[6])]
                count_encoder += 1
                print(f"encoder_data = {encoder_data}")
               # print(encoder_data)

                ##### 1 预测
                input = encoder_data[2:4]
                ekf.predict(input)   # 预测 xp Pp

                ##### 2 更新
                ekf.update(delta_theta)  # 2 更新 x p

                # 统计他们之间的误差
                delta_theta_list.append(delta_theta)
                time_list.append(timestamp)

                delta_theta = 0.0        # 重置IMU的角度差累计

                pose = ekf.getStateX()   # 此时pose为(3,1)的列向量

                print(f"pose = {pose.shape}")
                cv2.circle(image,   # 绘制的图像
                           # 绘制的原点  (0,0) 为左上角， 现在到 (800 450)
                           (int(1000 - pose[0].item() * 20), int(500 - pose[1].item() * 20)), 
                            3,           # 绘制小圆的半径 像素为单位
                            (0, 0, 255), # 绘制小圆的颜色 黑色
                            -1)          # 实心圆
                cv2.imshow("trajectory", image)
                # cv2.waitKey(5)
                # write odom pose to output file: timestamp pose_x pose_y
                try:
                    px = float(pose[0].item())
                    py = float(pose[1].item())
                    fout.write(f"{encoder_data[0]} {px} {py}\n")
                    fout.flush()
                except Exception as _:
                    pass

            # 数据的含义 time gx, gy, gz
            if data_type == 'imu':
                print(f"开始处理IMU数据")
                imu_data[0] = timestamp
                imu_data[1] = float(line[2])
                imu_data[2] = float(line[3])
                imu_data[3] = float(line[4])
                count_imu += 1
                print(f"imu_data = {imu_data}")
               # print(imu_data)

                now_time = imu_data[0]
                # 处理得到第一个 pre_time
                if  is_init == False:
                    print(f"IMU 初始化参数")
                    pre_time = now_time
                    pre_gz = imu_data[3]
                    is_init = True
                    print(f"pre_time = {pre_time}, now_time = {now_time}, pre_gz = {pre_gz}")
                    continue
                
                # 积分计算陀螺仪的角度差 
                delta_t = (now_time - pre_time) / 1000.0
                print(f"delta_t = {delta_t}")
                # 中值积分的方法
                delta_theta += delta_t * (pre_gz + imu_data[3]) * 0.5 
                print(f"delta_theta = {delta_theta}")

                pre_gz = imu_data[3]
                pre_time = now_time
            
            count +=1
    output_path = "/home/keyirobot/Desktop/qixing_ws/learn/imu_encoder_fusion/fusion3/data/delta_theta_output.txt"
    with open(output_path, 'w') as f:
        for timestamp, delta_theta_1 in delta_theta_list:
            f.write(f"{timestamp} {delta_theta_1}\n")
            f.close
    plt.plot(time_list, delta_theta_list)
    plt.xlabel('timestamp')
    plt.ylabel('delta_theta')
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()