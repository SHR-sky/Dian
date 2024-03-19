// 有待移植ESP32
#include<stdio.h>
#include<math.h>
#include<string.h>

// 变量定义
#define g 9.796
#define Kp 100.0f                // 比例增益支配率收敛到加速度计/磁强计
#define Ki 0.002f                // 积分增益支配率的陀螺仪偏见的衔接
#define halfT 0.001f             // 采样周期的一半
 
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;         // 按比例缩小积分误差
 
float Yaw,Pitch,Roll;  //偏航角，俯仰角，翻滚角

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
        printf("%f\n",Yaw);
}

// BUG未解决，改用python
int main()
{
    int a[6];
    int cnt = 0;
    FILE* fp=fopen("..//mpu6050_data.csv", "r");
    // fp = fopen("F:/Dian/上位机工具/mpu6050_data.csv", "r");
    printf("%p",fp);
    if (fp != NULL)
	    printf("11111111111\n");
    else printf("222222222222\n");
    char* line,* p;//字符指针
    char buffer[60],s[60];//存储所有字符
    const char* delim = ",";
    int k = 0;
	//一次获取所有字符，用strtok进行分割
    fseek(fp, 0L, SEEK_SET);
    line = fgets(buffer,60,fp);//先获取所有
    strcpy(s, line);//字符指针转给字符常量，否则不能用strtok
    printf("s=%s\n", s);
    p = strtok(s, delim);
    while (p != NULL) 
    {
	    a[k] = atof(p);//字符型转换为浮点型
	    printf("a[%d]=%f\n",k, a[k]);
	    p = strtok(NULL, delim);
	    k++;
        if(cnt == 5)
        {
            cnt = 0;
            IMUupdate(a[0],a[1],a[2],a[3],a[4],a[5]);
        }
    }
}


//单位换算，给显示用

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