#include <stdio.h>
#include <string.h>
#include "myiic.h"
#include <math.h>

#define g 9.794 //武汉重力加速度
#define N_data 6

#define Kp 100.0f                // 比例增益支配率收敛到加速度计/磁强计
#define Ki 0.002f                // 积分增益支配率的陀螺仪偏见的衔接
#define halfT 0.001f             // 采样周期的一半
 
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;         // 按比例缩小积分误差
 
extern float Yaw,Pitch,Roll;  //偏航角，俯仰角，翻滚角


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


void mpu6050_get_six(int16_t *gx, int16_t *gy, int16_t *gz, int16_t *ax, int16_t *ay, int16_t *az) 
{
    i2c_config_t esp32 = myiic_Init();

    uint8_t data[14];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();	//创建一个命令链表
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); //选择通信对象
    i2c_master_write_byte(cmd, 0x3B, true);	//读取位置
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);	//读操作
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);	//读14个byte
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd); //free cmd

    // 解析数据并输出(去掉温度部分)
    *ax = (data[0] << 8) | data[1];
    *ay = (data[2] << 8) | data[3];
    *az = (data[4] << 8) | data[5];
	*gx = (data[8] << 8) | data[9];
	*gy = (data[10] << 8) | data[11];
	*gz = (data[12] << 8) | data[13];
}



void mpu6050_get_data(void) {
    i2c_config_t esp32 = myiic_Init();

    uint8_t data[14];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();	//创建一个命令链表
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); //选择通信对象
    i2c_master_write_byte(cmd, 0x3B, true);	//读取位置
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);	//读操作
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);	//读14个byte
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd); //free cmd

    // 解析数据并输出(去掉温度部分)
    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];
	int16_t gx = (data[8] << 8) | data[9];
	int16_t gy = (data[10] << 8) | data[11];
	int16_t gz = (data[12] << 8) | data[13];

	IMUupdate((float)gx, (float)gy, (float)gz, (float)ax, (float)ay, (float)az);

}

/*
    由于FS_SEL默认为3，陀螺仪量程为-2000到2000，而16位有符号二进制可表示数字-32767到32767，并且为线性对应
    -32767对应-2000度每秒的陀螺仪角速度，32767对应2000度每秒的陀螺仪角速度。把32767除以2000，就可以得到16.40 
    即为灵敏度，也就是16.40个单位对应1度每秒
    同理，当AFS_SEL=3时，数字-32767对应-16g，32767对应16g。
    把32767除以16，就可以得到2048，即灵敏度。
    把从加速度计读出的数字除以2048，就可以换算成加速度的数值（以g为单位）
*/

void change_unit(int16_t *gx, int16_t *gy, int16_t *gz, int16_t *ax, int16_t *ay, int16_t *az, float uint[])
{
    float temp_gx,temp_gy,temp_gz,temp_ax,temp_ay,temp_az;
    temp_ax = (float) *ax;
    temp_ay = (float) *ay;
    temp_az = (float) *az;
    temp_gx = (float) *gx;
    temp_gy = (float) *gy;
    temp_gz = (float) *gz;

    temp_ax = temp_ax / 2048 * g;
    temp_ay = temp_ay / 2048 * g;
    temp_az = temp_az / 2048 * g;

    temp_gx = temp_gx / 16.4;
    temp_gy = temp_gy / 16.4;
    temp_gz = temp_gz / 16.4;

    uint[0] = temp_gx;
    uint[1] = temp_gy;
    uint[3] = temp_gz;
    uint[4] = temp_ax;
    uint[5] = temp_ay;
    uint[6] = temp_az;

}



void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
        float norm;
        float vx, vy, vz;
        float ex, ey, ez;  
 
        // 测量正常化
        norm = sqrt(ax*ax + ay*ay + az*az);

        //单位化   
        ax = ax / norm;                   
        ay = ay / norm;
        az = az / norm;      
 
        // 估计方向的重力
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
 
        // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);
 
        // 积分误差比例积分增益
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
 
        // 调整后的陀螺仪测量
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
 
        // 整合四元数率和正常化
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
 
        // 正常化四元
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
 
        Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch ,转换为度数
        Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv
        Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;                // yaw
}
