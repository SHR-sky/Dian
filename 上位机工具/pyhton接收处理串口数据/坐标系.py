import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import time

# 初始化串口
ser = serial.Serial('COM13', 115200, timeout=1)

# 初始化速度和位置
velocity = np.array([0.0, 0.0, 0.0])
position = np.array([0.0, 0.0, 0.0])

# 时间间隔
dt = 1.0

# 存储位置数据用于绘制轨迹图
positions = [position]

try:
    while True:
        # 从串口读取数据
        data = ser.readline().decode().strip().split(',')
        acceleration_x = float(data[0])
        acceleration_y = float(data[1])
        acceleration_z = float(data[2])

        # 更新速度
        velocity[0] += acceleration_x * dt
        velocity[1] += acceleration_y * dt
        velocity[2] += acceleration_z * dt

        # 更新位置
        position[0] += velocity[0] * dt
        position[1] += velocity[1] * dt
        position[2] += velocity[2] * dt

        positions.append(position.copy())

        # 绘制三维轨迹图
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        positions = np.array(positions)
        ax.plot(positions[:,0], positions[:,1], positions[:,2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

except KeyboardInterrupt:
    ser.close()