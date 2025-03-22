#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;

int khanh;
const int IN1 = 8;  //
const int IN2 = 7;  //

const int IN3 = 6;
const int IN4 = 5;

const int ENA = 9;   // banh phai
const int ENB = 10;  // banh trai

unsigned long hientai5 = 0;

unsigned long hientai = 0;
unsigned long hientai1 = 0;
unsigned long hientai2 = 0;
unsigned long hientai3 = 0;
unsigned long hientai4 = 0;
unsigned long thoigian;
int delay_trai = 3000;
int delay_phai = 3000;
int delay_quaydau = 700;
int delay_truoc = 10000;
boolean quayphai45 = 0;
boolean quaytrai45 = 0;
boolean quayphai = 0;
boolean quaytrai = 0;
boolean sau = 0;
boolean thang = 0;
//boolean truoc = 0;


int truoc;
int trai45;
int phai45;
int phai;



void setup() {
  Serial.begin(9600);
  
  // dongco
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //Motor two
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  //vl53
  pinMode(11, OUTPUT);  //trai
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(12, OUTPUT);



  digitalWrite(11, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(12, LOW);
   Wire.begin();
  init_VL53L0X();
  sensor.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();
  sensor4.startContinuous();
 // sensor5.startContinuous();
   delay(2000);
}

void loop() {

  thoigian = millis();

  hientai5 = thoigian;
  if (truoc > 400 && phai > 400 && trai45 > 400 && phai45 > 400) {
    quaydau();
    sau = 1;
    hientai5 = thoigian;
  }
  if (sau == 1) {
    if (thoigian - hientai5 > 400) {
      dung();
      sau = 0;
    }
  }
  phai = sensor.readRangeContinuousMillimeters();
  Serial.print("phai: ");
  Serial.println(phai);

  trai45 = sensor2.readRangeContinuousMillimeters();
  Serial.print("trai45: ");
  Serial.println(trai45);


  truoc = sensor3.readRangeContinuousMillimeters();
  Serial.print("truoc: ");
  Serial.println(truoc);

  phai45 = sensor4.readRangeContinuousMillimeters();
  Serial.print("phai45: ");
  Serial.println(phai45);  //dlphai mau trang = 650-760

  //&& phai > 400 && trai > 400 && trai45 >400 && phai45 >400
  if (truoc > 400)

  {
    dung();
    thang = 0;
  }

  if (truoc < 400) {

    strong1();
    thang = 1;
    hientai2 = thoigian;
  }
  if (thang == 1) {
    if (thoigian - hientai2 >= delay_truoc) {
      dung();
      thang = 0;
    }
  }
  if (phai < 400 && truoc > 400) {
    if (phai != -1 && phai != 8190 && phai != 8191) {
      quayphai2();
      quayphai = 1;
      hientai = thoigian;
    }
  }
  if (quayphai == 1) {
    if (thoigian - hientai >= delay_phai) {
      dung();
      quayphai = 0;
    }
  }

  if (trai45 < 400 && thang == 0 && truoc > 400) {
    quaytrai2();
    quaytrai45 = 1;
    hientai1 = thoigian;
  }
  if (quaytrai45 == 1) {
    if (thoigian - hientai1 >= 4000) {
      dung();
      quaytrai45 = 0;
    }
  }
  if (phai45 < 400 && truoc > 400 && phai > 400 && thang == 0) {
    quayphai1();
    quayphai45 = 1;
    hientai = thoigian;
  }
  if (quayphai45 == 1) {
    if (thoigian - hientai >= 4000) {
      dung();
      quayphai45 = 0;
    }
  }
}
void quaydau() {
  digitalWrite(IN1, LOW);   // high
  digitalWrite(IN2, HIGH);  // low
  digitalWrite(IN3, LOW);   // high
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 220);
  analogWrite(ENB, 240);  //TRAI
}
void quayphai1() {
  digitalWrite(IN1, LOW);   // high
  digitalWrite(IN2, HIGH);  // low
  digitalWrite(IN3, LOW);   // high
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 150);
  analogWrite(ENB, 170);  //TRAI
}
void quaytrai1() {
  digitalWrite(IN1, HIGH);  // high
  digitalWrite(IN2, LOW);   // low
  digitalWrite(IN3, HIGH);  // high
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 170);  //TRAI
}
void quayphai2() {
  digitalWrite(IN1, LOW);   // high
  digitalWrite(IN2, HIGH);  // low
  digitalWrite(IN3, LOW);   // high
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200);
  analogWrite(ENB, 210);  //TRAI
}
void quaytrai2() {
  digitalWrite(IN1, HIGH);  // high
  digitalWrite(IN2, LOW);   // low
  digitalWrite(IN3, HIGH);  // high
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 180);
  analogWrite(ENB, 190);  //TRAI
}
void lui() {
  digitalWrite(IN1, HIGH);  // high
  digitalWrite(IN2, LOW);   // low
  digitalWrite(IN3, LOW);   // high
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 240);
  analogWrite(ENB, 240);  //TRAI
}
void strong1() {
  analogWrite(ENA, 240);
  analogWrite(ENB, 240);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void dung() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void init_VL53L0X() {
  Serial.print("Setting VL53L0X addresses...");
  digitalWrite(11, HIGH);
  delay(1);
  sensor.init(true);
  sensor.setAddress((uint8_t)01);

  digitalWrite(3, HIGH);
  delay(1);
  sensor2.init(true);
  sensor2.setAddress((uint8_t)02);

  digitalWrite(4, HIGH);
  delay(1);
  sensor3.init(true);
  sensor3.setAddress((uint8_t)03);

  digitalWrite(12, HIGH);
  delay(1);
  sensor4.init(true);
  sensor4.setAddress((uint8_t)04);
}