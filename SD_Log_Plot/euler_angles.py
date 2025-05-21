import numpy as np
import matplotlib.pyplot as plt

# 读取TXT文件（假设数据以空格分隔，无表头）
data = np.loadtxt('test_1.txt')  # 若为其他分隔符，添加delimiter=','等参数[6,8](@ref)

# 提取三列数据（假设列顺序为roll, pitch, yaw）
time = np.arange(len(data))  # 生成时间序列（若无时间列）
roll = data[:, 0]            # 第一列
pitch = data[:, 1]           # 第二列
yaw = data[:, 2]             # 第三列

# 创建可视化画布
plt.figure(figsize=(12, 6))

# 绘制三条曲线
plt.plot(time, roll, label='Roll (X-axis)', color='r', linewidth=1.5)
plt.plot(time, pitch, label='Pitch (Y-axis)', color='g', linewidth=1.5)
plt.plot(time, yaw, label='Yaw (Z-axis)', color='b', linewidth=1.5)

# 设置图形属性
plt.title('Euler Angles Visualization', fontsize=14)
plt.xlabel('Time Steps', fontsize=12)
plt.ylabel('Rotation Angle (degrees)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(loc='best')
plt.tight_layout()

# 显示图形
plt.show()



