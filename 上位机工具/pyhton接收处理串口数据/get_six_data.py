import serial
import csv

# 打开串口
ser = serial.Serial('COM13', 115200)  # 请将'COM3'替换为你的串口端口号
print("Serial port opened")

# 创建CSV文件并写入表头
with open('sensor_data.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['a/g', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])

    # 读取串口数据并写入CSV
    while True:
        line = ser.readline().decode('utf-8').strip()  # 读取一行串口数据
        if line.startswith("a/g:"):
            data = line.split('\t')[1:]  # 分割数据
            writer.writerow(data)
            print("Data written to CSV:", data)

# 关闭串口
ser.close()
print("Serial port closed")