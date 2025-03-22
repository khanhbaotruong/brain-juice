#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;

int khanh;
const int IN1 = 8;  // 
const int IN2 = 7;   // 

const int IN3 = 6;
const int IN4 = 5;

const int ENA = 9; // banh phai
const int ENB = 10; // banh trai

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
boolean quayphai = 0;
boolean quaytrai = 0;
boolean sau = 0;
boolean thang = 0;
//boolean truoc = 0;

int dolinetrai = A2;
int dolinephai = A3;

int truoc;
int trai;
int trai45;
int phai45;
int phai;



void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode (A3, INPUT);
  pinMode (A2,INPUT);
  // dongco
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //Motor two
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  //vl53
  pinMode(2, OUTPUT); //trai
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);



  digitalWrite(2, LOW); 
 digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(12, LOW);
        digitalWrite(13, LOW);

  init_VL53L0X();
  sensor.startContinuous();
  sensor2.startContinuous();
    sensor3.startContinuous();
    sensor4.startContinuous();
    sensor5.startContinuous();
}

void loop() {
    thoigian = millis(); 
     int dltrai = analogRead(A2);
  int dlphai = analogRead(A3);
  Serial.print("doline_trai = ");
  Serial.println(dltrai);
  Serial.print("doline_phai = ");
  Serial.println(dlphai);

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
  Serial.println(phai45);//dlphai mau trang = 650-760
  trai = sensor5.readRangeContinuousMillimeters();
   Serial.print("trai: ");
  Serial.println(trai); // dltrai mau trang = 600-760
// doline
//   if (dlphai < 800) {
//     khanh = 1;
//   }
//   else if (dltrai <800) {
//     khanh = 2;
//   }
//   else if (dlphai < 800 && dltrai < 800) {
//     khanh = 3;
//   }
// switch (khanh)  // < 650 means white line
//   {
//     case 1:
//       luiphai();
//       delay(300);
//       quaytrai1();
//       delay(250);
//       dung();
//       khanh = 0;
//       break;

//     case 2:
//       luitrai();
//       delay(300);
//       quayphai1();
//       delay(250);
//       dung();
//       khanh = 0;
//       break;

//     case 3:
//       lui();
//       delay(300);
//       quaytrai1();
//       delay(450);
//       khanh = 0;
//       break;
//   }


  if(truoc >300 )
 
  {
    dung();
  }
  if (truoc < 300) {

   strong1();
  thang = 1;
  hientai2 = thoigian;
  }
  if(thang == 1)
  {
    if(thoigian - hientai2 >=delay_truoc)
    {
      dung();
      thang = 0;
    }
  }
if (phai < 300 && trai >300 && truoc >300) {
  if(phai != -1 && phai != 8190){
    quayphai1();
    quayphai = 1;
    hientai = thoigian;
  }
}
  if(quayphai == 1)
  {
    if(thoigian - hientai >=delay_phai)
    {
      dung();
      quayphai = 0;
    }
  }

  if (trai < 300 && phai > 300 && truoc >300) {
    quaytrai1();
    quaytrai = 1;
    hientai1 = thoigian;
  }
  if(quaytrai == 1)
  {
    if(thoigian - hientai1 >=delay_trai)
    {
      dung();
      quaytrai = 0;
    }
  }
  if (trai45 < 300 && truoc > 300 && trai >300) {
    quaytrai1();
    quaytrai = 1;
    hientai1 = thoigian;
  }
  if(quaytrai == 1)
  {
    if(thoigian - hientai1 >=4000)
    {
      dung();
      quaytrai = 0;
    }
  }
  if (phai45 < 300 && truoc >300 && phai > 300) {
    quayphai1();
    quayphai = 1;
    hientai = thoigian;
}
  if(quayphai == 1)
  {
    if(thoigian - hientai >=4000)
    {
      dung();
      quayphai = 0;
    }
  }
// if(truoc<100 || trai <100 || phai <100)
// {
// thang();
// delay(2000);
// }
// if(trai45<100 || phai45<100)
// {
// lui();
// delay(2000);
// }
// dung();
// }
}
void quaydau() {
  digitalWrite(IN1, LOW);  // high
  digitalWrite(IN2, HIGH);   // low
  digitalWrite(IN3, LOW);   // high
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);//TRAI
}
void quayphai1() {
  digitalWrite(IN1, LOW);  // high
  digitalWrite(IN2, HIGH);   // low
  digitalWrite(IN3, LOW);   // high
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 230);
  analogWrite(ENB, 230);//TRAI
}
void quaytrai1() {
  digitalWrite(IN1, HIGH);  // high
  digitalWrite(IN2, LOW);   // low
  digitalWrite(IN3, HIGH);   // high
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 230);
  analogWrite(ENB, 230);//TRAI
}
void lui() {
  digitalWrite(IN1, HIGH);  // high
  digitalWrite(IN2, LOW);   // low
  digitalWrite(IN3, LOW);   // high
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 240);
  analogWrite(ENB, 240);//TRAI
}
void strong1() {
    analogWrite(ENA, 240);
  analogWrite(ENB, 240);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}  
void luiphai() {
    analogWrite(ENA, 240);
  analogWrite(ENB, 190);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
} 
void luitrai() {
    analogWrite(ENA, 190);
  analogWrite(ENB, 240);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
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
  digitalWrite(2, HIGH);
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
  
  digitalWrite(13, HIGH);
  delay(1);
  sensor5.init(true);
  sensor5.setAddress((uint8_t)05);
}