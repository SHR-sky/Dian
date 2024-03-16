#include <stdio.h>
#include <string.h>
#include "myiic.h"



i2c_config_t myiic_Init(void)
{
    i2c_config_t config = {
    .mode = I2C_MODE_MASTER,                 // 主机方式启动IIC
    .sda_io_num = I2C_MASTER_SDA_IO,         // SDA
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_MASTER_SCL_IO,         // SCL
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,  // 时钟频率
    // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    i2c_param_config(I2C_NUM_0, &config);

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    return config;

}


void MPU6050_init()
{
    uint8_t check;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 发送地址，以及写指令，命令之后需要带ACK
    i2c_master_write_byte(cmd, MPU_CMD_WHO_AM_I, true);                   // 发送 WHO_MI_I 寄存器地址 0x75
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &check, I2C_MASTER_LAST_NACK);              // 读取数据
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);                                             // 删除指令
    if (check != 0x68)
    {
        ESP_LOGE(TAG, "MPU6050 不在线!( %02X )", check);
        return;
    }
    ESP_LOGI(TAG, "MPU6050 检测到在线，开始初始化...");

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_PWR_MGMT_1, true);                 // 写入电源管理和复位控制
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（设置陀螺仪采样率分频器）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_SMPLRT_DIV, true);                 // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x07, true);                               // 写入寄存器数据 Sample rate = 1kHz/(7+1) = 125Hz
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（数字低通滤波器配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_CONFIG, true);                     // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Gyroscope：260Hz 0ms，Accelerometer：256Hz 0.98ms 8Khz
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（陀螺仪配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_GYRO_CONFIG, true);                // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Gyroscope: +/- 250 dps
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（加速度传感器配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_CONFIG, true);               // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Accelerometer: +/- 2g
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    ESP_LOGI(TAG, "MPU6050 初始化完毕!");
}
/**
 * @brief 获得 X 轴加速度
 * @return 返回 X 轴加速度
 */
int16_t get_accel_x()
{
    union
    {
        uint8_t bytes[4];
        int16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_XOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    return data.value;
}
