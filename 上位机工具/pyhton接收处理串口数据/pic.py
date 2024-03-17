import serial
import csv
from collections import deque
import time

import csv
from pyecharts.charts import Line
from pyecharts import options as opts

y_data_x = []
y_data_y = []
y_data_z = []

def read_csv_data():
    with open('accelerometer_data.csv', mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header
        for row in reader:
            y_data_x.append(float(row[1]))
            y_data_y.append(float(row[2]))
            y_data_z.append(float(row[3]))

def generate_line_chart() -> Line:
    line = Line()
    x_data = list(range(1, len(y_data_x) + 1))  # 自动生成从1开始的数字作为x轴
    line.add_xaxis(x_data)
    line.add_yaxis("Acceleration X", y_data_x, is_smooth=True)
    line.add_yaxis("Acceleration Y", y_data_y, is_smooth=True)
    line.add_yaxis("Acceleration Z", y_data_z, is_smooth=True)
    line.set_global_opts(title_opts=opts.TitleOpts(title="Real-time Acceleration Data"))
    return line

# 定义队列长度
QUEUE_LENGTH = 150

cnt = 0

# 打开串口
ser = serial.Serial('COM13', 115200)  # 串口号和波特率

# 创建 CSV 文件并写入数据
with open('accelerometer_data1111.csv', mode='w', newline='') as file:
    #writer = csv.writer(file)
    #writer.writerow(['Time', 'Acceleration_X', 'Acceleration_Y', 'Acceleration_Z'])  # 写入 CSV 文件的标题行

    pos_writer_file = open('position.csv',mode='w',newline='')
    pos_writer = csv.writer(file)
    pos_writer.writerow(['Time','x','y','z'])


    # 创建一个固定长度的队列
    data_queue = deque(maxlen=QUEUE_LENGTH)

    try:
        while True:
            # 从串口读取数据
            writer_file = open('accelerometer_data.csv',mode='w',newline='')
            writer = csv.writer(writer_file)
            line = ser.readline().decode().strip()
            
            
            # 判断是否是加速度数据行
            if line.startswith("aworld"):
                data = line.split('\t')
                timestamp = cnt
                cnt = cnt + 1
                accel_x = float(data[1])
                accel_y = float(data[2])
                accel_z = float(data[3])
                
                # 更新队列
                data_queue.append([timestamp, accel_x, accel_y, accel_z])

                # 此处应该清空一下CSV文件
                # 写入一个空行来清空文件
                # 写入 CSV 文件
                writer.writerows(data_queue)
            writer_file.close()
            y_data_x = []
            y_data_y = []
            y_data_z = []
            read_csv_data()  # 读取所有数据
            line = generate_line_chart()
            line.render("realtime_acceleration.html")
            time.sleep(0.01)

    except KeyboardInterrupt:
        pos_writer_file.close()
        writer_file.close()
        print("程序终止")
        ser.close()

def calc_pos(timescale,ax,ay,az):
    x = 0
    y = 0
    z = 0
    vx = 0
    vy = 0
    vz = 0
    while(True):
        vx += timescale*ax
        vy += timescale*ay
        vz += timescale*az
        x += timescale*vx
        y += timescale*vy
        z += timescale*vz
        time.sleep(timescale)
    

    return [x,y,z]



# 此处要改为，判断是否串口开启，且读到数据
# while(1):
#     read_csv_data()  # 读取所有数据
#     line = generate_line_chart()
#     line.render("realtime_acceleration.html")








# -----------------------------------
# 草稿





# import csv
# from pyecharts.charts import Line
# from pyecharts import options as opts

# y_data_x = []
# y_data_y = []
# y_data_z = []

# def read_csv_data():
#     with open('accelerometer_data.csv', mode='r') as file:
#         reader = csv.reader(file)
#         next(reader)  # Skip header
#         for row in reader:
#             y_data_x.append(float(row[1]))
#             y_data_y.append(float(row[2]))
#             y_data_z.append(float(row[3]))

# def generate_line_chart() -> Line:
#     line = Line()
#     x_data = list(range(1, len(y_data_x) + 1))  # 自动生成从1开始的数字作为x轴
#     line.add_xaxis(x_data)
#     line.add_yaxis("Acceleration X", y_data_x, is_smooth=True)
#     line.add_yaxis("Acceleration Y", y_data_y, is_smooth=True)
#     line.add_yaxis("Acceleration Z", y_data_z, is_smooth=True)
#     line.set_global_opts(title_opts=opts.TitleOpts(title="Real-time Acceleration Data"))
#     return line

# read_csv_data()  # 读取所有数据

# if len(y_data_x) > 0:
#     line = generate_line_chart()
#     line.render("realtime_acceleration.html")
#     print("数据已读取完毕，图表已生成。")
# else:
#     print("未找到有效数据，无法生成图表。")



# 图表动态更新数据

# import serial
# import csv
# from collections import deque

# # 定义队列长度
# QUEUE_LENGTH = 150


# # 打开串口
# ser = serial.Serial('COM13', 115200)  # 串口号和波特率

# # 创建 CSV 文件并写入数据
# with open('accelerometer_data.csv', mode='w', newline='') as file:
#     writer = csv.writer(file)
#     writer.writerow(['Time', 'Acceleration_X', 'Acceleration_Y', 'Acceleration_Z'])  # 写入 CSV 文件的标题行

#     # 创建一个固定长度的队列
#     data_queue = deque(maxlen=QUEUE_LENGTH)

#     try:
#         while True:
#             # 从串口读取数据
#             line = ser.readline().decode().strip()
            
#             # 判断是否是加速度数据行
#             if line.startswith("aworld"):
#                 data = line.split('\t')
#                 timestamp = data[0]
#                 accel_x = float(data[1])
#                 accel_y = float(data[2])
#                 accel_z = float(data[3])
                
#                 # 更新队列
#                 data_queue.append([timestamp, accel_x, accel_y, accel_z])

#                 # 写入 CSV 文件
#                 writer.writerows(data_queue)

#     except KeyboardInterrupt:
#         print("程序终止")
#         ser.close()

# # 函数有待合成





