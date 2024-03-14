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

read_csv_data()  # 读取所有数据

if len(y_data_x) > 0:
    line = generate_line_chart()
    line.render("realtime_acceleration.html")
    print("数据已读取完毕，图表已生成。")
else:
    print("未找到有效数据，无法生成图表。")