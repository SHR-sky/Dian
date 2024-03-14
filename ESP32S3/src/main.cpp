//ESP32数据网页

#include <ESPAsyncWebServer.h>    // 包含异步Web服务器库文件
#include <WiFi.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
 

MPU6050 accelgyro;
 
//三轴的旋转角度及三轴加速度
int16_t ax, ay, az;
int16_t gx, gy, gz;
 

//13脚为指示灯
#define LED_PIN 13
bool blinkState = false;

const char *ssid = "荣耀Magic4 Pro";
const char *password = "123321123";
 
AsyncWebServer server(80);        // 创建WebServer对象, 端口号80
// 功能为展示MPU收集到的数据
// 一个储存网页的数组
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
	<meta charset="utf-8">
</head>
<body>
	<h2>ESP32 网页</h2>
	<!-- 创建一个ID为dht的盒子用于显示获取到的数据 -->
	<div id="dht">
	</div>
	<button onclick="set()"> 发送数据 </button>
</body>
<script>
	// 按下按钮会运行这个JS函数
	function set() {
		var payload = "ESP32"; // 需要发送的内容，ESP32接受指令后可进行某些操作
		// 通过get请求给 /set
		var xhr = new XMLHttpRequest();
		xhr.open("GET", "/set?value=" + payload, true);
		xhr.send();
	}
	// 设置一个定时任务, 1000ms执行一次
	setInterval(function () {
		var xhttp = new XMLHttpRequest();
		xhttp.onreadystatechange = function () {
			if (this.readyState == 4 && this.status == 200) {
				// 此代码会搜索ID为dht的组件，然后使用返回内容替换组件内容
				document.getElementById("dht").innerHTML = this.responseText;
			}
		};
		// 使用GET的方式请求 /dht
		xhttp.open("GET", "/dht", true);
		xhttp.send();
	}, 1000)
</script>)rawliteral";

String Merge_Data(void)
{

  // 将温湿度打包为一个HTML显示代码
  String dataBuffer = "<p>";
  dataBuffer += "<h1>传感器数据 </h1>";
  dataBuffer += "<b>x轴加速度: </b>";
  dataBuffer += String(ax);
  dataBuffer += "<br/>";
  dataBuffer += "<b>y轴加速度: </b>";
  dataBuffer += String(ay);
  dataBuffer += "<br/>";
  dataBuffer += "<b>z轴加速度: </b>";
  dataBuffer += String(az);
  dataBuffer += "<br/>";
  dataBuffer += "<b>x轴角速度: </b>";
  dataBuffer += String(gx);
  dataBuffer += "<br/>";
  dataBuffer += "<b>y轴角速度: </b>";
  dataBuffer += String(gy);
  dataBuffer += "<br/>";        
  dataBuffer += "<b>z轴角速度: </b>";
  dataBuffer += String(gz);
  dataBuffer += "<br /></p>";
  // 最后要将数组返回出去
  return dataBuffer;
}
 
// 下发处理回调函数
void Config_Callback(AsyncWebServerRequest *request)
{
  if (request->hasParam("value")) // 如果有值下发
  {
    String HTTP_Payload = request->getParam("value")->value();    // 获取下发的数据
    Serial.printf("[%lu]%s\r\n", millis(), HTTP_Payload.c_str()); // 打印调试信息
  }
  request->send(200, "text/plain", "OK"); // 发送接收成功标志符
}
  
void setup()
{
    //初始化I2C，采用软件I2C
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(6,7,10000);
    #endif
    
    //打开串口，传输数据
    Serial.begin(115200);

    Serial.println("Initializing I2C devices..."); 
    //初始化MPU6050
    accelgyro.initialize();
 
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection is successful" : "MPU6050 connection is failed");

    //指示灯配置
    pinMode(LED_PIN, OUTPUT);

    Serial.println();

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid,password);

    while(WiFi.status() == WL_CONNECT_FAILED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());

  // 添加HTTP主页，当访问的时候会把网页推送给访问者
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html); });
  // 设置反馈的信息，在HTML请求这个Ip/dht这个链接时，返回打包好的传感器数据
  server.on("/dht", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/plain", Merge_Data().c_str()); });
  server.on("/set", HTTP_GET, Config_Callback);   // 绑定配置下发的处理函数
  server.begin();  // 初始化HTTP服务器
  Serial.print("I am OK");
}

void loop()
{

    //获取姿态数据
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //灯闪烁，说明数据正常传输
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(1000); //保持和网页刷新速度一致

}




/*
//WIFI传输数据

#include <WiFi.h>

const char *ssid = "荣耀Magic4 Pro";
const char *password = "123321123";

const IPAddress serverIP(192,168,90,123);
uint16_t serverPort = 8010;

WiFiClient client;

void setup()
{
    Serial.begin(115200);
    Serial.println();

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid,password);

    while(WiFi.status() == WL_CONNECT_FAILED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());
}

void loop()
{
    Serial.println("try to connect the AP:");
    if(client.connect(serverIP,serverPort))
    {
        Serial.println("OK! Let say hello world");
        client.printf("This is a try to covery the data from STA");
        while(client.connected() || client.available())
        {
            if(client.available())
            {
                String line = client.readStringUntil('\n');
                Serial.print("I get it!");
                Serial.println(line); //测试能否将数据回传到ESP，并且利用ESP进行处理
            }
        }
        Serial.println("Bye bye!");
        client.stop(); //客户端关闭
    }
    else
    {
        Serial.println("There seemed to be a problem!");
        client.stop(); //客户端关闭
    }
    delay(5000);
}

*/



/*
//加速度，欧拉角以及速度计算

#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050_6Axis_MotionApps612.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
 

MPU6050 mpu;

//三轴速度
static int16_t x_v =0;
static int16_t y_v =0;
static int16_t z_v =0;

//#define OUTPUT_READABLE_YAWPITCHROLL

#define OUTPUT_READABLE_WORLDACCEL
 
#define INTERRUPT_PIN 15 
#define LED_PIN 13
bool blinkState = false;
 
// MPU 控制/状态 量
bool dmpReady = false;  // DMP初始化成功标志
uint8_t mpuIntStatus;   // 中断状态
uint8_t devStatus;      // 操作状态 (0 = success, !0 = error)
uint16_t packetSize;    // DMP数据包大小 (默认 42 bytes)
uint16_t fifoCount;     // 字节数 in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// 运动/方向 量
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 
// 包格式
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};
 
//中断配置
 
volatile bool mpuInterrupt = false; // 标志中断脚是否被拉高（使能）
ICACHE_RAM_ATTR void dmpDataReady()
{
    mpuInterrupt = true;
}
 
//初始化
 
void setup()
{
    //开启I2C
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(6,7,400000); //初始化I2C
        Wire.setClock(400000); 
    #endif

    //开启串口
    Serial.begin(115200);
    while (!Serial); // 等待串口连接

 
    //初始化
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    //指示灯
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection is successful") : F("MPU6050 connection is failed"));

 
    //初始化加载 DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // 给偏置，初始校准，经验数据，用于手动校0
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    Serial.printf("devStatus=%d\n", devStatus);

    if (devStatus == 0) //正常工作
    {
        // 校准，给初始偏置
        mpu.CalibrateAccel(6);
         
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        // 使能DMP
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
 
        // 使能中断
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
 
        // DMP开启成功标志，中断开启标志位置1
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
 
        // 获取数据包大小
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else //报错
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
 

    pinMode(LED_PIN, OUTPUT);
    
}
 
//循环调用
 
void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet


 //坐标系坐标的实部与虚部
#ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
#endif
 
 //欧拉角
#ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        Serial.print("euler\t");
        Serial.print(euler[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180 / M_PI);
#endif
 
 //偏航角、俯仰角和滚转角
#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
#endif
 //真实加速度
#ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
#endif
 //固定坐标系下的真实加速度
#ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        Serial.print("aworld\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.println(aaWorld.z);
#endif
 
#ifdef OUTPUT_TEAPOT
        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        Serial.write(teapotPacket, 14);
        teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
       

        x_v = aaWorld.x + x_v;
        y_v = aaWorld.y + y_v;
        z_v = aaWorld.z + z_v;


        //闪灯指示运行
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

*/


//姿态数据

/*
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
 

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
 

MPU6050 accelgyro;
 
//三轴的旋转角度及三轴加速度
int16_t ax, ay, az;
int16_t gx, gy, gz;
 
 
 
//输出十进制可读的数据
#define OUTPUT_READABLE_ACCELGYRO

//13脚为指示灯
#define LED_PIN 13
bool blinkState = false;


void setup() {

    //初始化I2C，采用软件I2C
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(6,7,10000);
    #endif
    
    //打开串口，传输数据
    Serial.begin(115200);

    Serial.println("Initializing I2C devices..."); 
    //初始化MPU6050
    accelgyro.initialize();
 
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection is successful" : "MPU6050 connection is failed");

    //指示灯配置
    pinMode(LED_PIN, OUTPUT);
}
 
void loop() {

    //获取姿态数据
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO

        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    //灯闪烁，说明数据正常传输
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(100);
}
*/

/*
#include <Arduino.h>
#include <HardwareSerial.h>
#include "LED.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define LED 2
String rev="hello_world";

void setup() {
  Serial.begin(115200);
  
}

void loop() {

    Serial.println(rev);


}

*/