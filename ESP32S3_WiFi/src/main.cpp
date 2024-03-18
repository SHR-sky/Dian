#define ESP32_WiFi 0

#if ESP32_WiFi

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

<iframe src="http://127.0.0.1:5500/realtime_acceleration.html" width="1000" height="600" frameborder="0">
  <p>您的浏览器不支持iframe元素。</p>
</iframe>


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




#else



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

    pinMode(13,OUTPUT);

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
                if(line == "LED_ON") //测试能否接收指令
                {
                    digitalWrite(13,HIGH);
                }
                if(line == "LED_OFF")
                {
                    digitalWrite(13,LOW);
                }
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

#endif
