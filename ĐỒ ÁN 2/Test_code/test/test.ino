//#include <Adafruit_MAX31865.h>
#include <stdlib.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include "Capacitive_Soil.h"
#include "Max31865_Khanh.h"

//Khai báo VSPI Max31865
#define CS_A 5

#define SDI 23
#define SDO 19
#define CLK 18

//DHT11 chân 15
#define DHTPIN 15
#define DHTTYPE DHT11
#define LED_ONBOARD 2
#define quat 12  // là công tắc điện tử, kích khi có yêu cầu thông gió - cân bằng nhiệt độ
#define den 14   // là công tắc điện tử, kích khi có yêu cầu PID nhiệt độ
uint16_t phantramdat = 0;
DHT dht(DHTPIN, DHTTYPE);


// const char* ssid = "OPPO A9 2020";
// const char* password = "annhien2016";
const char* mqtt_server = "172.20.10.9";

// const char* ssid = "ehehe";
// const char* password = "nhatdeptrai";

const char* ssid = "BTK";
const char* password = "88888888";

// const char* ssid = "THAI BINH";
// const char* password = "ngoinhahanhphuc";  




WiFiClient esp32;
PubSubClient client(esp32);
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
bool PID_Nhietdo = 0;
bool batquat_1 = 0;
bool batquat_2 = 0;
bool batquat_3 = 0;
bool phun_suong = 1;
bool tuoicay = 0;
bool bat_quat = 0;
bool manual = 0;
bool error_t_h = 0;
bool batbom = 0;
unsigned long lastMsg = 0;
unsigned long delay_ngat = 0;
unsigned long time1;
unsigned long time2;
unsigned long time_sheet;
unsigned long time_hoitiep;
unsigned long now_phunsuong;
unsigned long now_quat;
unsigned long laymau_dat;
float h;
float t;

//khai bao cac biến cho mạch phát hiện điểm 0
uint32_t demngat = 0;
bool zero_cross_detected = 0;
uint32_t max_firing_delay = 7000;
bool fix = 0;
bool fix_oke = 0;
#define ngatdiem0 22
#define firing_pin 17
// mạch kích triac


//Khai báo các biến ngoại vi
char data_temp[12] = "";
char data_soil[5] = "";
char data_hum[12] = "";
char data_temp_PT100[12] = "";
//char data
float muccanhbao_nhietdo_nguongduoi = 27.5;
float muccanhbao_nhietdo_nguongtren = 28.5;
uint8_t muccanhbao_soil_nguongduoi = 70  ;
uint8_t muccanhbao_soil_nguongtren = 85;

//uint32_t Freq = 0;

//biến PID

float setpoint = 28.0f;  // nhiệt độ tốt - đặt cho cây phát triển.
//Đồ án sẽ kiểm soát nhiệt độ của nhà kính dao động từ 27,5 đến 28,5.
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
//PID constants
int kp = 2000;
float ki = 50;
int kd = 1500;
float PID_p = 0;
float PID_i = 0;
float PID_d = 0;

// setting PWM properties
const int suongPin = 13;     // GPIO13 - PWM phun sương
const int freq = 15000;      // Tần số 15KHz
const int suongChannel = 1;  // chọn channel 1
const int resolution = 8;    // độ phân giải analog 8 bit, tối đa là 16 bit
const int duty_suong = 200;

// void ngat trên chân 22

void IRAM_ATTR ngat() {
  if (digitalRead(ngatdiem0) == 0) {
    zero_cross_detected = 1;
    //demngat += 1;
    while (digitalRead(ngatdiem0) == 0) {
      // nếu nhiệt độ nhà kính đang trong quá trình PID mà nhiệt độ môi trường >= 27.5, thì công tắc đèn sẽ bị tắt , dẫn đến chân ngắt(zerocross) luôn là mức 0, vì thế ta cần lệnh if để thoát khỏi vòng lặp.
      if (digitalRead(den) == 0) {
        break;
        Serial.println("ket trong if");
      }
         Serial.println("ket trong while");
    }
  }
}

void setup_wifi() {
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  //WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("ESP32 connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(String topic, byte* message, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message_string;
  for (int i = 0; i < length; i++) { 
    Serial.print((char)message[i]);
    message_string += (char)message[i];
  }
  Serial.println();

  //Ham nay de sua muc canh bao từ dashboard gửi

  //Serial.print("chuoi_truoc");
  // Serial.println(message_string);
  if (topic == "nhietdo/MCB/nguongtren") {
    muccanhbao_nhietdo_nguongtren = message_string.toInt();
    Serial.print("muccanhbaonhietdo_nguongtren: ");
    Serial.println(muccanhbao_nhietdo_nguongtren);
  } else if (topic == "nhietdo/MCB/nguongduoi") {
    muccanhbao_nhietdo_nguongduoi = message_string.toInt();
    Serial.print("muccanhbaonhietdo_nguongduoi: ");
    Serial.println(muccanhbao_nhietdo_nguongduoi);
  }

  else if (topic == "doamdat/MCB/nguongtren") {
    muccanhbao_soil_nguongtren = message_string.toInt();
    Serial.print("muccanhbaodoam_nguongtren: ");
    Serial.println(muccanhbao_soil_nguongtren);
  } else if (topic == "doamdat/MCB/nguongduoi") {
    muccanhbao_soil_nguongduoi = message_string.toInt();
    Serial.print("muccanhbaodoam_nguongduoi: ");
    Serial.println(muccanhbao_soil_nguongduoi);
  }
  if (phantramdat >= muccanhbao_soil_nguongtren || phantramdat <= muccanhbao_soil_nguongduoi) {
    client.publish("warning/soil", "độ ẩm đất vượt ngưỡng");
    time1 = millis();
  } else if (PT100_Temperature >= muccanhbao_nhietdo_nguongtren || PT100_Temperature <= muccanhbao_nhietdo_nguongduoi) {
    client.publish("warning/temp", "nhiệt độ nhà kính vượt ngưỡng");
    time2 = millis();
  }


  // Chop tat led
  if (topic == "manual") {
    if (message_string == "on") {
      Serial.println("che do manual duoc bat....");
      Serial.println();
      digitalWrite(LED_ONBOARD, HIGH);  // led sáng thì manual được bật
      digitalWrite(quat, 0);
      ledcWrite(suongChannel, 0);
      digitalWrite(den, 0);
      manual = 1;
    } else if (message_string == "off") {
      Serial.println("che do manual duoc tat....");
      digitalWrite(LED_ONBOARD, LOW);  // Turn the LED off by making the voltage HIGH
      manual = 0;
    }
  }
  //chọn chế độ, nếu có tín hiệu manual nodered, thì dừng hết tất cả các hàm như PID, phun sương kiểm soát, quạt kiểm soát v.v...
  if (topic == "quathut" || topic == "phunsuong") {
    // tắt quạt và tắt sương trước khi thực hiện thay đổi trạng thái on - off
    if (topic == "quathut") {
      if (message_string == "on" && manual == 1) {
        Serial.println("bat quat");
        digitalWrite(quat, HIGH);  // Turn the LED on (Note that LOW is the voltage level
      } else if (message_string == "off" && manual == 1) {
        digitalWrite(quat, LOW);  // Turn the LED off by making the voltage HIGH
      }
    }
    if (topic == "phunsuong") {
      if (message_string == "on" && manual == 1) {
        Serial.println("bat bom suong");
        ledcWrite(suongChannel, 255);
      } else if (message_string == "off" && manual == 1) {
        ledcWrite(suongChannel, 0);
      }
    }
  } 
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32";
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("manual");
      client.subscribe("doamdat/MCB/nguongtren");
      client.subscribe("doamdat/MCB/nguongduoi");
      client.subscribe("nhietdo/MCB/nguongtren");
      client.subscribe("nhietdo/MCB/nguongduoi");
      client.subscribe("quathut");
      client.subscribe("phunsuong");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void setup() {
  // Set pin mode
    Serial.begin(115200);

  pinMode(SOIL, INPUT);
  SPI.begin();
  pinMode(LED_ONBOARD, OUTPUT);
  dht.begin();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(CS_A, OUTPUT);
  pinMode(ngatdiem0, INPUT);
  pinMode(quat, OUTPUT);
  pinMode(den, OUTPUT);
  pinMode(firing_pin, OUTPUT);
  ledcSetup(suongChannel, freq, resolution);
  ledcAttachPin(suongPin, suongChannel);
  attachInterrupt(ngatdiem0, ngat, FALLING);
  time_sheet = millis();
  delay_ngat = millis();
  laymau_dat = millis();

  //    Freq = getCpuFrequencyMhz();
  // Serial.print("CPU Freq = ");
  // Serial.println(Freq);
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  // tuoi_cay();
  // Serial.print("cam bien dat = ");
  // Serial.println(cambiendat);

  //nếu manual == 0, thì hệ thống được tự động điều chỉnh nhiệt độ và độ ẩm đất
  if(manual == 0)
  {
  //PID khi nhiệt độ dưới 27.5
  if (t < 27.5 && PT100_Temperature < 27.5 && tuoicay == 0) {
    PID_Nhietdo = 1;
    batquat_1 = 0;
    batquat_2 = 0;
    batquat_3 = 0;
    if (digitalRead(quat) == 1) {
      digitalWrite(quat, 0);
    }
    ledcWrite(suongChannel, 0);
  }
  //tắt PID khi nhiệt độ ngoài trời đã lớn hơn 27.5
  //khi nhiệt độ ngoài trời ngang với mức nhiệt độ mong muốn, chỉ cần bật quạt liên tục để thông gió - cân bằng nhiệt độ trong và ngoài nhà kính
  if ((t >= 27.5 && t <= 28.5) && tuoicay == 0) {
    PID_Nhietdo = 0;
    batquat_1 = 1;
    batquat_2 = 0;
    batquat_3 = 0;
    if (digitalRead(den) == 1) {
      digitalWrite(den, 0);
    }
    ledcWrite(suongChannel, 0);
  }
  //khi nhiệt độ ngoài trời đã cao hơn ngưỡng mong muốn, ta cần dùng đến hệ thống phun sương có kiểm soát kết hợp quạt thông gió
  if ((t > 28.5 && PT100_Temperature > 28.5) && tuoicay == 0) {
    PID_Nhietdo = 0;
    batquat_1 = 0;
    batquat_2 = 1;
    batquat_3 = 0;
    if (digitalRead(den) == 1) {
      digitalWrite(den, 0);
    }
  }

  // trường hợp ngoại lệ
  if ((t < 27.5 && PT100_Temperature > 28.5) && tuoicay == 0)  // vì một lý do nào đó nhiệt độ môi trường khá thấp, nhưng nhiệt độ trong nhà kính > 28.5
  {
    PID_Nhietdo = 0;
    batquat_1 = 0;
    batquat_2 = 0;
    batquat_3 = 1;
    if (digitalRead(den) == 1) {
      digitalWrite(den, 0);
    }
    ledcWrite(suongChannel, 0);
  }

  if (PID_Nhietdo == 1) {
    PID_Temp();
  }

  if (batquat_1 == 1) {
    if (digitalRead(quat) == 0) {
      digitalWrite(quat, 1);
    }
  }

  if (batquat_2 == 1) {
    // if(PT100_Temperature < 27.5) // trường hợp tưới cây khi trời năng nóng và độ ẩm đất < 80, ta ưu tiên tưới cây, nhưng tưới cây xong nhiêt độ trong phòng giảm mạnh
    // {
    //   if(digitalRead(quat) == 0)
    //     {
    //     digitalWrite(quat, 1);
    //     }
    //   ledcWrite(suongChannel, 0);
    // }
    // nếu phun sương khiến cho nhà kính giảm nhiệt độ xuống dưới 28.5, lúc này không cần phun nữa. Chỉ cần đợi khi sức nóng của buổi trưa làm cho nhà kính nóng hone 28.5 thì sẽ bắt đầu phun tiếp và quạt tiếp
    if (phun_suong == 1 && PT100_Temperature > 28.5) {
      digitalWrite(quat, 0);
      now_phunsuong = millis();
      phun_suong = 0;
      bat_quat = 1;
      ledcWrite(suongChannel, duty_suong);
    }
    if ((millis() - now_phunsuong > 2000) && bat_quat == 1) {
      ledcWrite(suongChannel, 0);
      now_quat = millis();
      digitalWrite(quat, 1);
      bat_quat = 0;
    }

    if (millis() - now_quat > 10000 && bat_quat == 0)  // bật quạt 7 giây sau khi phun sương 3 giây
    {
      phun_suong = 1;
    }
  }

  if (batquat_3 == 1) {
    ledcWrite(suongChannel, 0);
    if (PT100_Temperature > 28.3) {
      if (digitalRead(quat) == 0) {
        digitalWrite(quat, 1);
      }
    } else {
      if (digitalRead(quat) == 1) {
        digitalWrite(quat, 0);
      }
    }
  }
  if(millis() - laymau_dat > 3000)
  {
    uint32_t phantramdat = convert_SOIL();
    if(phantramdat <70){
    ledcWrite(suongChannel, 0); // tắt bơm trước khi lấy mẫu mỗi 3 giây
    delay(200);
    uint32_t phantramdat = convert_SOIL();
    //uint32_t phantramdat_codinh = phantramdat;
    }
    if(phantramdat < 70)
    {
      tuoicay = 1; // tuoicay==1 để không gia nhiệt hoặc bật quạt khi đang tưới cây
      batbom = 1; // batbom == 1 để không cho gửi dữ liệu lên dashboard mỗi 1 giây (vì lúc bơm, giá trị độ ẩm đất không đúng)
      tuoi_cay();
    }else{
      if(batquat_2 == 0)
      {
      ledcWrite(suongChannel, 0);
      }
      tuoicay = 0;
      batbom = 0;
    }
    laymau_dat = millis();
  }
  if(phantramdat >= 70 && t < 28.5)
  {
    ledcWrite(suongChannel, 0);
  }
  }

  unsigned long now1 = millis();
  //publish cảm biến lên dashboard
  if (now1 - lastMsg > 1000) {
    lastMsg = now1;
    h = dht.readHumidity();
    t = dht.readTemperature();
    //kiểm tra xem có lỗi trong quá trình đọc nhiệt độ trên DHT11 không
    if (isnan(h) || isnan(t)) {
      error_t_h = 1;
    } else {
      error_t_h = 0;
    }
    if (error_t_h == 0) {
      sprintf(data_temp, "%3.2f", t);
      client.publish("temperature", data_temp);
      Serial.print("Temp: ");
      Serial.print(t);
      Serial.println(" *C");

      sprintf(data_hum, "%3.2f", h);
      client.publish("humidity", data_hum);
      Serial.print("Humi: ");
      Serial.print(data_hum);
      Serial.println(" %RH");
    }
    //publish độ ẩm đất
    if(batbom == 0)
    {
    phantramdat = convert_SOIL();  //data_soil
    sprintf(data_soil, "%3d", phantramdat);
    client.publish("cap/soil", data_soil);
    Serial.print("Phan tram do am dat = ");
    Serial.println(data_soil);
    }

    //publish nhiệt độ trong nhà kính
    readRegister(CS_A);
    convertToTemperature(CS_A);
    sprintf(data_temp_PT100, "%3.2f", PT100_Temperature);
    client.publish("temp/PT100", data_temp_PT100);
    Serial.print("PT100 = ");
    Serial.println(data_temp_PT100);
  }



  //publish cảnh báo lên Node-RED, rồi từ Node-RED đẩy lên điện thoại
  if (((phantramdat >= muccanhbao_soil_nguongtren) || (phantramdat <= muccanhbao_soil_nguongduoi)) && (millis() - time1 > 12000)) {
    ledcWrite(suongChannel, 0); // tắt bơm trước khi lấy mẫu mỗi 3 giây
    delay(200);
    uint32_t phantramdat = convert_SOIL();
  if ((phantramdat >= muccanhbao_soil_nguongtren) || (phantramdat <= muccanhbao_soil_nguongduoi))
  {
    client.publish("warning/soil", "Độ ẩm đất vượt ngưỡng");
  }
    time1 = millis();
  }
  if ((PT100_Temperature >= muccanhbao_nhietdo_nguongtren || PT100_Temperature <= muccanhbao_nhietdo_nguongduoi) && (millis() - time2 > 12000)) {
       readRegister(CS_A);
    convertToTemperature(CS_A);
  if ((PT100_Temperature >= muccanhbao_nhietdo_nguongtren || PT100_Temperature <= muccanhbao_nhietdo_nguongduoi))
  {
    client.publish("warning/temp", "Nhiệt độ nhà kính vượt ngưỡng");
  }
    time2 = millis();
  }

  //cập nhật dữ liệu lên sheet mỗi 10 giây 1 lần
  if (millis() - time_sheet > 10000) {

    client.publish("temperature/table", data_temp);
    //client.publish("humidity/table", data_hum);
    client.publish("soil/table", data_soil);
    client.publish("temp_PT100/table", data_temp_PT100);
    time_sheet = millis();
  }
}

// hàm PID gia nhiệt
void PID_Temp() {

  if (digitalRead(den) == 0 && (millis() - delay_ngat > 15000)) {
    digitalWrite(den, 1);  // công tắc tổng, bật công tắc này để bắt đầu thực hiện PID
  } else {
  }
  unsigned long now = millis();
  if (now - time_hoitiep > 1000) {

    PID_error = setpoint - PT100_Temperature;
    if (PID_error > 1.1)  
    { PID_i = 0; }

    PID_p = kp * PID_error;            //Calculate the P value
    PID_i = PID_i + (ki * PID_error);  //Calculate the I value
    timePrev = Time;                   // the previous time is stored before the actual time read
    Time = millis();                   // actual time read
    elapsedTime = (Time - timePrev) / 1000;
    PID_d = kd * ((PID_error - previous_error) / elapsedTime);  //Calculate the D value
    PID_value = PID_p + PID_i + PID_d;                          //Calculate total PID value

    //We define firing delay range between 0 and 7000. Read above why 7000!!!!!!!
    if (PID_value < 0) { PID_value = 0; }
    if (PID_value > 7000) { PID_value = 7000; }
    previous_error = PID_error;
    Serial.print("PID_Value = ");
    Serial.println(PID_value);
    time_hoitiep = now;
  }
  // Kiểm soát công suất bóng đèn
  if (zero_cross_detected == 1) {
    delayMicroseconds(max_firing_delay - PID_value);
    if ((PT100_Temperature >= setpoint - 0.01) && fix == 0) {
      digitalWrite(firing_pin, 0);
      delay(3000);
      fix = 1;
      fix_oke = 1;
    }
    if (PT100_Temperature >= setpoint && fix_oke == 1) {
      digitalWrite(firing_pin, 0);
    } else {
      digitalWrite(firing_pin, 1);
    }
    delayMicroseconds(100);
    digitalWrite(firing_pin, 0);
    zero_cross_detected = 0;
  }
}

void tuoi_cay()
{
    PID_Nhietdo = 0;
    batquat_1 = 0;
    batquat_2 = 0;
    batquat_3 = 0;
    if(digitalRead(quat) == 1)
    {
    digitalWrite(quat, 0);
    }
    if(digitalRead(den) == 1)
    {
    digitalWrite(den, 0);
    }
    uint32_t phantramdat = convert_SOIL();
    uint32_t phantramdat_codinh = phantramdat;
    sprintf(data_soil, "%3d", phantramdat_codinh);
    client.publish("cap/soil", data_soil);
  
  if(phantramdat_codinh >= 0 && phantramdat_codinh < 30)
  {
    ledcWrite(suongChannel, 255);
    //batbom = 0;
  }
  else if(phantramdat_codinh >= 30 && phantramdat_codinh < 60)
  {
    ledcWrite(suongChannel, 210);
  }
  else if(phantramdat_codinh >= 60 && phantramdat_codinh < 70)
  {
    ledcWrite(suongChannel, 180);
  }
  if(phantramdat_codinh >= 70)
  {
    tuoicay = 0;
  }

}