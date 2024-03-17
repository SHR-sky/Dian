# Dian

a repo to store the code of Dian Group

该库内有两个文件夹，ESP32S3文件夹，为基于arduino框架的工程

IDF为基于IDF框架的工程

为了在规定时间内完成所有基础要求和扩展要求，所以采用了arduino框架加速开发，IDF框架内预计只会包含基础要求和一两项扩展

### ESP32S3

~~类似于stm32，对每个功能进行模块化封装~~ 

事实证明，arduino框架的封装，导致自己封装的模块会出现奇奇怪怪的问题，遂放弃，只在IDF框架采用

基础部分已完成:

- 点灯 3.12
- 串口 3.12
- I2C读取MPU6050数据 3.12
- 解算MPU6050速度 3.13
- 数据优化 3.15



扩展部分已完成：

- 数据传输到上位机，使用电脑进行数值计算 3.13
- WIFI传输数据（手机热点作为AP，ESP8266作为STA） 3.13
- WEB搭建，展示数据 3.14
- 屏幕显示 3.16
- 基于lvgl的UI制作 3.16



LEVEL 1 

- [x] 点灯

LEVEL 2

- [x] 串口
- [x] IIC

LEVEL 2 ==扩展==

- [x] 连接AP，获取IP
- [x] WiFi传输六轴数据到电脑
- [x] 搭建WEB，实现逻辑交互

LEVEL 3

- [x] 解算速度
- [x] 优化数据

LEVEL 3 ==扩展==

- [x] 计算欧拉角
- [ ] 计算移动方向，移动速度
- [ ] 计算移动轨迹

LEVEL 4 ==扩展==

- [x] 传输到电脑，电脑进行数值计算

LEVEL 5 ==扩展==

- [x] 屏幕显示
- [x] LVGL移植





### IDF

类似于stm32，对每个功能进行模块化封装

基础部分已完成：

- 点灯封装
- 串口封装 3.16
- i2c封装 3.17
- mpu6050姿态解算封装 3.17