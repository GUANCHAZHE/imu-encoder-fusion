import numpy as np

class ESKF:

    """
    初始化拓展卡尔曼滤波器
    : 初始变量 X = [x,y,theta]
    : 更新变量 X = [x,y,theta] 
    : 初始化协方差矩阵P
    : 初始化运动噪声矩阵Q
    : 初始化测量噪声V
    """
    def __init__(self):
        print("eskf init end")
        # 当前k - 1时刻的状态变量（x，y，theta）
        self.x = np.zeros(3)  # 零矩阵 行向量array([0., 0., 0.])

        # k 运动更新的状态 
        self.x_new = np.zeros(3)  

        # 运动过程的协方差矩阵 P
        self.P = np.identity(3)   # 对角阵
        self.P_new = np.identity(3)   # 对角阵

        # 状态协方差矩阵 F
        self.F = np.identity(3)   # 对角阵

        # 观测矩阵H
        self.H = np.zeros((1, 3))

        # 运动噪声矩阵 Q
        self.Q = np.identity(3) * 10

        # 观测噪声误差 V
        self.V = np.identity(1) * 1

        # 初始化的标志位
        self.is_init = False
        print("eskf init end")

    ######## 预测
    def predict(self, u_now):
        """
        预测步骤
        直接对轮速编码器和差速机器人建模
        u_now 对应的是 del_s del_theta
        """
        if self.is_init == False:
            self.is_init = True
            return

        print("start predict")

        ## 1 根据运动方程 预测状态 轮速的割线模型
        # xk = xk-1 + del_s * cos (theta_k-1 + 0.5 * del_theta_k-1)
        self.x_new[0] = self.x[0] + u_now[0] * np.cos(self.x[2] + 0.5 *u_now[1])
        # yk = yk-1 + del_s * sin (theta_k-1 + 0.5 * del_theta_k-1)
        self.x_new[1] = self.x[1] + u_now[0] * np.sin(self.x[2] + 0.5 *u_now[1])
        # theta_k = theta_k-1 + del_theta_k-1
        self.x_new[2] = self.x[2] + u_now[1]
        print("X.shape", self.x_new.shape)


        ## 2 计算状态雅可比矩阵 F
        self.F[0, 2] = -u_now[0] * np.sin(self.x[2] + 0.5 * u_now[1])
        self.F[1, 2] =  u_now[0] * np.cos(self.x[2] + 0.5 * u_now[1]) 
        print("F.shape", self.F.shape)
        
        ## 3 计算状态协方差矩阵 P
        #P = FPF^T + Q
        self.P_new = self.F @ self.P @ self.F.T + self.Q
        print("P_new.shape", self.P_new.shape)

        print("纯预测的Δθ", self.x_new[2] - self.x[2])
        print("end predict")


######## 更新
# 读取IMU的z轴的角加速度，通过中值积分得到角度增量 Δθ
    def update(self, z):
        if self.is_init == False:
            return

        print("start update")
        # 1 观测估计方程
        self.H = np.array([[0, 0, 1]])  # H 为列向量 1*3
        print("H.shape", self.H.shape)


        # 计算误差  Δθ  h = H * x_new -x 
        # 也就是通过运动模型得到角度θk - 原始角度θk-1 = Δθ
        h = self.H @ self.x_new - self.x[2]
        print("h.shape =", h.shape)
        print("预测角度差值Δθ h =", h[0])
        print("观测角度差值Δθ z =", z)

        # 2 观测误差
        ## 这里表示的是角度增量的误差，预测值 - 测量值
        error = z - h
        print("观测角度增量误差 error= z - h =", error[0])

        # 3 卡尔曼增益
        # k = P* H^T (H P H^T + V)^-1
        K = self.P_new @ self.H.T @ np.linalg.inv(self.H @ self.P_new @ self.H.T + self.V)
        print("K.shape", K.shape)
        # 4 状态更新

        # 更新x
        self.x_new = self.x_new + K @ error
        # 这里出问题了，显示的是 3*3 之前使用的是 * 而非矩阵乘法

        # 更新P
        self.P_new = (np.identity(3) - K @ self.H) @ self.P_new

        print("end update")
        
        # 更新x和p，将他们替代成
        self.x = self.x_new.copy()   # .copy()深拷贝
        self.P = self.P_new.copy()   
        # 如果不写成深拷贝的格式，那么会出现相关的问题


    def getStateX(self):
        return self.x_new