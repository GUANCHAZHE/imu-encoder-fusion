# 判断编码器静止的时候，读取IMU，判断静止的磁力计的R，来看看自己的估计
#TODO 轮速计的噪声估计数据Q要根据跟更准的真值来确定，这个需要真实的轨迹
# 只能通过更准的轨迹来推测参数
import numpy as np
import matplotlib.pyplot as plt
def main():

    path = "/home/keyirobot/Desktop/qixing_ws/learn/imu_encoder_fusion/encoder_only/imu_encoder_file.txt"
    is_static = False

    imu_list = []
    timestamp_list = []
    ## 读取数据
    with open(path, 'r') as f:
        for lines in f:
            line = lines.strip().split()
            if not line:
                continue

            data_type = line[0]
     
            if data_type == 'encoder':
                delta_s = float(line[3])
                delta_theta = float(line[4])
                if( delta_s == 0  and delta_theta == 0):
                    print(f"静止")
                    is_static = True
                    continue
                else:
                    break
            
            if data_type == 'imu':
                if is_static:
                    timestamp_list.append(float(line[1]))
                    imu_list.append([float(line[2]), float(line[3]), float(line[4])])
    f.close()

    ## 可视化这这些函数
    plt.plot(timestamp_list, imu_list)
    plt.xlabel('timestamp')
    plt.ylabel('imu')
    plt.legend(['gx', 'gy', 'gz'])
    # plt.show()
    # plt.close()

    # 计算R
    imu_list = np.array(imu_list)

    gx_var = np.var(imu_list[:, 0])
    gy_var = np.var(imu_list[:, 1])
    gz_var = np.var(imu_list[:, 2])
    print(f"gx_var = {gx_var}, gy_var = {gy_var}, gz_var = {gz_var}")

    ## v2
    g_var = np.var(imu_list, axis=0 )
    print(f"g_var = {g_var}")

    print(f"imu_list{imu_list.shape}")

if __name__ == "__main__":
    main()