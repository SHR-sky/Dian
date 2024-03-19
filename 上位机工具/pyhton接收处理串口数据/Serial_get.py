import serial
import csv

# 打开串口
ser = serial.Serial('COM13', 115200)  # 串口号和波特率

# 创建 CSV 文件并写入数据
with open('accelerometer_data_raw.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time', 'Acceleration_X', 'Acceleration_Y', 'Acceleration_Z'])  # 写入 CSV 文件的标题行

    try:
        while True:
            # 从串口读取数据
            line = ser.readline().decode().strip()
            
            # 判断是否是加速度数据行
            if line.startswith("aworld"):
                data = line.split('\t')
                timestamp = data[0]
                accel_x = float(data[1])
                accel_y = float(data[2])
                accel_z = float(data[3])

                # 写入 CSV 文件
                writer.writerow([timestamp, accel_x, accel_y, accel_z])
                
    except KeyboardInterrupt:
        print("程序终止")
        ser.close()