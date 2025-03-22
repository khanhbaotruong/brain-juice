//#include <Adafruit_MAX31865.h>
#include <stdlib.h>
#include <SPI.h>

#define CS_A 5

#define SDI 23
#define SDO 19
#define CLK 18

#define ngatdiem0 22
#define firing_pin 17
#define den 14
unsigned long now1;
uint32_t demngat = 0;
uint32_t max_firing_delay = 7000;
bool zero_cross_detected = 0;
bool fix = 0;
bool fix_oke=0;

//biến PID

float setpoint = 40.0f;
//PID variables
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
//PID constants
int kp = 300;
float ki = 3;
int kd = 400;
float PID_p = 0;
float PID_i = 0;
float PID_d = 0;

//Variables for the PT100 boards
double resistance;
uint8_t reg1, reg2;  //reg1 holds MSB, reg2 holds LSB for RTD
uint16_t fullreg;    //fullreg holds the combined reg1 and reg2
double PT100_Temperature;
//Variables and parameters for the R - T conversion
double Z1, Z2, Z3, Z4, Rt;
double RTDa = 3.9083e-3;
double RTDb = -5.775e-7;
double rpoly = 0;

unsigned long lastMsg = 0;

void IRAM_ATTR ngat() {
  if (digitalRead(ngatdiem0) == 0) {
    zero_cross_detected = 1;
    demngat += 1;
    while (digitalRead(ngatdiem0) == 0);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SPI.begin();
  pinMode(CS_A, OUTPUT);
  pinMode(ngatdiem0, INPUT);
  pinMode(firing_pin, OUTPUT);
  pinMode(den, OUTPUT);
  attachInterrupt(ngatdiem0, ngat, FALLING);
  //digitalWrite(den, 1);
  unsigned long now1 = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long now = millis();
  if(millis() - now1 > 10000)
  {
    digitalWrite(den, 1);
  }

  // if (zero_cross_detected == 1) {
  //   Serial.print("so lan ngat");
  //   Serial.println(demngat);
  //   zero_cross_detected = 0;
  // }

  //PID
  if (now - lastMsg > 500) {
    
    readRegister(CS_A);
    convertToTemperature(CS_A);
    PID_error = setpoint - PT100_Temperature;
    if (PID_error > 3.1)  //integral constant will only affect errors below 30ºC
    { PID_i = 0; }

    PID_p = kp * PID_error;            //Calculate the P value
    PID_i = PID_i + (ki * PID_error);  //Calculate the I value
    timePrev = Time;                   // the previous time is stored before the actual time read
    Time = millis();              // actual time read
    elapsedTime = (Time - timePrev) / 1000;
    PID_d = kd * ((PID_error - previous_error) / elapsedTime);  //Calculate the D value
    PID_value = PID_p + PID_i + PID_d;                          //Calculate total PID value

    //We define firing delay range between 0 and 8300. Read above why 8300!!!!!!!
    if (PID_value < 0) { PID_value = 0; }
    if (PID_value > 8300) { PID_value = 8300; }
    previous_error = PID_error;
    Serial.print("PID_Value = ");
    Serial.println(PID_value);
    lastMsg = now;
  }

// Xuất điện áp hiệu dụng cho bóng đèn gia nhiệt
  if (zero_cross_detected) {
					  delayMicroseconds(max_firing_delay - PID_value);
					  if((PT100_Temperature >= setpoint-0.2)&& fix==0)
			  					  {
			  						digitalWrite(firing_pin,0);
			  						delay(3000);
			  						fix = 1;
			  						fix_oke = 1;
			  					  }
					  if(PT100_Temperature >= setpoint && fix_oke == 1){
			  					  digitalWrite(firing_pin,0);
			  					  }else{digitalWrite(firing_pin,1);}
			  			delayMicroseconds(100);
					  digitalWrite(firing_pin,0);
					  zero_cross_detected = 0;

				  }
}

void convertToTemperature(int8_t CS) {
  Rt = resistance;
  Rt /= 32768;
  Rt *= 417;  //This is now the real resistance in Ohms
  Serial.print("Resistance: ");
  Serial.println(Rt);  //Temperature in Celsius degrees
  Z1 = -RTDa;
  Z2 = RTDa * RTDa - (4 * RTDb);
  Z3 = (4 * RTDb) / 100;
  Z4 = 2 * RTDb;

  PT100_Temperature = Z2 + (Z3 * Rt);
  PT100_Temperature = (sqrt(PT100_Temperature) + Z1) / Z4;

  if (PT100_Temperature >= 0) {
    Serial.print("MAX31865: ");
    Serial.println(PT100_Temperature);  //Temperature in Celsius degrees
    return;                             //exit
  } else {
    Rt /= 100;
    Rt *= 100;  // normalize to 100 ohm

    rpoly = Rt;

    PT100_Temperature = -242.02;
    PT100_Temperature += 2.2228 * rpoly;
    rpoly *= Rt;  // square
    PT100_Temperature += 2.5859e-3 * rpoly;
    rpoly *= Rt;  // ^3
    PT100_Temperature -= 4.8260e-6 * rpoly;
    rpoly *= Rt;  // ^4
    PT100_Temperature -= 2.8183e-8 * rpoly;
    rpoly *= Rt;  // ^5
    PT100_Temperature += 1.5243e-10 * rpoly;

    Serial.print("Temperature: ");
    Serial.println(PT100_Temperature);  //Temperature in Celsius degrees
  }
  //Note: all formulas can be found in the AN-709 application note from Analog Devices
}


void readRegister(int8_t ChipSel) {
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(ChipSel, LOW);

  SPI.transfer(0x80);  //80h = 128 - config register
  SPI.transfer(0xB0);  // A0h = 160 - 1011 0000: set bias high, start 1-shot, 2/4-Wire
  delay(10);           // Wartezeit fuer Conversion
  //Von Schreiben in Lesen muss High-Low gesetzt werden
  digitalWrite(ChipSel, HIGH);
  digitalWrite(ChipSel, LOW);
  //

  SPI.transfer(1);            // Daten sollen ausgelesen werden
  reg1 = SPI.transfer(0xFF);  // 8 Bit vom MSB
  reg2 = SPI.transfer(0xFF);  // 8 Bit vom LSB
  digitalWrite(ChipSel, HIGH);

  fullreg = reg1;        //read MSB
  fullreg <<= 8;         //Shift to the MSB part
  fullreg |= reg2;       //read LSB and combine it with MSB
  fullreg >>= 1;         //Shift D0 out. -> eigentlich nur 15Bit und nicht 16
  resistance = fullreg;  //pass the value to the resistance variable
  //note: this is not yet the resistance of the RTD!

  digitalWrite(ChipSel, LOW);
  SPI.transfer(0x80);  //80h = 128
  //SPI.transfer(0x00); //144 = 10010000
  SPI.endTransaction();
  digitalWrite(ChipSel, HIGH);
}