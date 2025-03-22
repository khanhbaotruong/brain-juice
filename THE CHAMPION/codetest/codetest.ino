#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;

unsigned long hientai = 0;
unsigned long thoigian;
//boolean truoc = 0;
int delay_truoc = 1500;

int dolinetrai = 11;
int dolinephai = 12;

int truoc;
int trai;
int trai45;
int phai45;
int phai;



#define MotorDirection 8  //dir1, phai
#define MotorSpeed 9   //pwm1
#define tanso 100000

#define MotorDirection2 7 //dir2, trai
#define MotorSpeed2 6    // pwm2



int SpeedVal = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode (A3, INPUT);
  pinMode (A2,INPUT);
  //vl53
  pinMode(2, OUTPUT); //trai
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
    pinMode(10, OUTPUT);



  digitalWrite(2, LOW); 
 digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
        digitalWrite(10, LOW);



  
  init_VL53L0X();
  sensor.startContinuous();
  sensor2.startContinuous();
    sensor3.startContinuous();
    sensor4.startContinuous();
    sensor5.startContinuous();

  //ledcwritephai
  pinMode(MotorDirection, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);

  //ledcwritetrai
  pinMode(MotorDirection2, OUTPUT);
  pinMode(MotorSpeed2, OUTPUT);
}

void loop() {
    thoigian = millis(); 
     int dltrai = analogRead(A3);
  int dlphai = analogRead(A2);
  Serial.print("doline_trai = ");
  Serial.println(dltrai);
  Serial.print("doline_phai = ");
  Serial.println(dlphai);

  trai = sensor.readRangeContinuousMillimeters();
  Serial.print("trai: ");
  Serial.println(trai);
  
  trai45 = sensor2.readRangeContinuousMillimeters();
   Serial.print("trai45: ");
  Serial.println(trai45);
  

  truoc = sensor3.readRangeContinuousMillimeters();
   Serial.print("truoc: ");
  Serial.println(truoc);

  phai45 = sensor4.readRangeContinuousMillimeters();
   Serial.print("phai45: ");
  Serial.println(phai45);
  phai = sensor5.readRangeContinuousMillimeters();
   Serial.print("phai: ");
  Serial.println(phai);
if(trai<100)
{
  thang();
}
else{lui();}
// if (a < 130) {
//    thang();
//   truoc = 1;
//   hientai = thoigian;
//   }
//   if(truoc == 1)
//   {
//     if(thoigian - hientai >=delay_truoc)
//     {
//       dung();
//       truoc = 0;
//     }
//   }

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
void thang() {
  digitalWrite(MotorDirection, LOW); // phai
  digitalWrite(MotorDirection2, HIGH);// trai
  analogWrite(6, 50);
  analogWrite(9, 50);
}
void lui() {
  digitalWrite(MotorDirection, HIGH); // phai
  digitalWrite(MotorDirection2, LOW);// trai
  analogWrite(6, 50);
  analogWrite(9, 50);
}  
void dung() {
  digitalWrite(MotorDirection, LOW);
  digitalWrite(MotorDirection2, HIGH);
  analogWrite(6, 0);
  analogWrite(9, 0);
}
void init_VL53L0X() {
  Serial.print("Setting VL53L0X addresses...");
  digitalWrite(2, HIGH);
  delay(100);
  sensor.init(true);
  delay(100);
  sensor.setAddress((uint8_t)01);
  delay(100);

  digitalWrite(3, HIGH);
  delay(100);
  sensor2.init(true);
  delay(100);
  sensor2.setAddress((uint8_t)02);
  delay(100);

  digitalWrite(4, HIGH);
  delay(100);
  sensor3.init(true);
  delay(100);
  sensor3.setAddress((uint8_t)03);
  delay(100);
  
  digitalWrite(5, HIGH);
  delay(100);
  sensor4.init(true);
  delay(100);
  sensor4.setAddress((uint8_t)04);
  delay(100);
  
  digitalWrite(10, HIGH);
  delay(100);
  sensor5.init(true);
  delay(100);
  sensor5.setAddress((uint8_t)05);
  delay(100);
}