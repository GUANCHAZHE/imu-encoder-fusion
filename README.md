这是一个参考别人的相关的代码
主要的方法为先实现以轮速计为主的EKF，然后采取IMU作为监督

测试git的代理
第二次测试
关闭clash的tun模式
然后本地导入设置代理


# 数据格式如下
## sad的数据集
fusion4_improve/data/10_small_sad.txt      
传感器参数为：
wheel_radius_ = 0.155  # 轮子半径 单位米m  
odom_span_ = 0.1       # 里程计测量时间间隔 单位秒s
circle_pulse 1024.0    # 编码器每圈脉冲数
分辨率：

格式   
ODOM time                l r
ODOM 1624426287.29101515 0 0

## 马尔科夫的真值数据
fusion4_improve/data/odom_data.txt         
格式   odom x y ？
fusion4_improve/data/odom_pose_output.txt  融合结果的数据
格式为：time x y

## 马尔科夫IMU+encoder数据
fusion3/data/imu_encoder_file_sorted.txt   
格式：
encoder   time(ms)       ?        s           θ           ？       ？     
encoder 1655267147295 0.421497 0.00934194 0.000571163 36194816 37717232
imu 1655267147309 -0.121141 0.006945 0.0367644