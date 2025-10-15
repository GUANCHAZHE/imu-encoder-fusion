# Evaluate the model's performance
"""
这是一个评价的函数
输入的是俩个轨迹
输出的是他们的差值
可视化他们的差值
"""
# fmt : on
import numpy as np
import matplotlib.pyplot as plt

def evaluate(true_trajectory, predicted_trajectory):


    # 计算均方误差 (MSE)
    mse = np.mean((np.array(true_trajectory) - np.array(predicted_trajectory))**2)
    
    # 可视化轨迹
    plt.figure(figsize=(10, 5))
    plt.plot(true_trajectory, label='True Trajectory', color='blue')
    plt.plot(predicted_trajectory, label='Predicted Trajectory', color='red', linestyle='--')
    plt.title('Trajectory Comparison')
    plt.xlabel('Time Step')
    plt.ylabel('Position')
    plt.legend()
    plt.grid()
    plt.show()
    
    return mse
