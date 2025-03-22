/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

//#include <Adafruit_MAX31865.h>
#include <stdlib.h>
#include <SPI.h>
#include "DHT.h"


#define DHTPIN 15
#define DHTTYPE DHT11

#define CS_A 5



#define SDI 23
#define SDO 19
#define CLK 18


//CS => CS //Arduino 10
//MISO => SDO //Arduino 12
//MOSI => SDI //Arduino 11
//SCK => SCK //Arduino 13

//Variables for the PT100 boards
double resistance;
uint8_t reg1, reg2; //reg1 holds MSB, reg2 holds LSB for RTD
uint16_t fullreg; //fullreg holds the combined reg1 and reg2
double temperature;
//Variables and parameters for the R - T conversion
double Z1, Z2, Z3, Z4, Rt;
double RTDa = 3.9083e-3;
double RTDb = -5.775e-7;
double rpoly = 0;

float h;
float t;

unsigned long lastMsg = 0;
DHT dht(DHTPIN, DHTTYPE);
//const int chipSelectPin = 10;

void setup()
{
  SPI.begin();
  Serial.begin(115200); //Start serial
  pinMode(CS_A, OUTPUT); //because CS is manually switched  
  dht.begin();
}

void loop()
{
  //readRegister(CS_A);
 // convertToTemperature(CS_A);
  

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    h = dht.readHumidity();
    t = dht.readTemperature();

    Serial.print("Humi: ");
    Serial.print(h);
    Serial.println(" %RH");

    Serial.print("DHT11: ");
    Serial.print(t);
    Serial.println(" *C");

    readRegister(CS_A);
  convertToTemperature(CS_A);
  }

}


void convertToTemperature(int8_t CS)
{
  Rt = resistance;
  Rt /= 32768;
  Rt *= 417; //This is now the real resistance in Ohms
  Serial.print("Resistance: ");
  Serial.println(Rt); //Temperature in Celsius degrees
  Z1 = -RTDa;
  Z2 = RTDa * RTDa - (4 * RTDb);
  Z3 = (4 * RTDb) / 100;
  Z4 = 2 * RTDb;

  temperature = Z2 + (Z3 * Rt);
  temperature = (sqrt(temperature) + Z1) / Z4;

  if (temperature >= 0)
  {
    Serial.print("MAX31865: ");
    Serial.println(temperature); //Temperature in Celsius degrees
    return; //exit
  }
  else
  {
    Rt /= 100;
    Rt *= 100; // normalize to 100 ohm

    rpoly = Rt;

    temperature = -242.02;
    temperature += 2.2228 * rpoly;
    rpoly *= Rt; // square
    temperature += 2.5859e-3 * rpoly;
    rpoly *= Rt; // ^3
    temperature -= 4.8260e-6 * rpoly;
    rpoly *= Rt; // ^4
    temperature -= 2.8183e-8 * rpoly;
    rpoly *= Rt; // ^5
    temperature += 1.5243e-10 * rpoly;

    Serial.print("Temperature: ");
    Serial.println(temperature); //Temperature in Celsius degrees
  }
  //Note: all formulas can be found in the AN-709 application note from Analog Devices
}


void readRegister(int8_t ChipSel)
{
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(ChipSel, LOW);

  SPI.transfer(0x80); //80h = 128 - config register
  SPI.transfer(0xB0); // A0h = 160 - 1011 0000: set bias high, start 1-shot, 2/4-Wire
  delay(10); // Wartezeit fuer Conversion
  //Von Schreiben in Lesen muss High-Low gesetzt werden
  digitalWrite(ChipSel, HIGH);
  digitalWrite(ChipSel, LOW);
  //
  
  SPI.transfer(1); // Daten sollen ausgelesen werden
  reg1 = SPI.transfer(0xFF); // 8 Bit vom MSB
  reg2 = SPI.transfer(0xFF); // 8 Bit vom LSB
  digitalWrite(ChipSel, HIGH);

  fullreg = reg1; //read MSB
  fullreg <<= 8;  //Shift to the MSB part
  fullreg |= reg2; //read LSB and combine it with MSB
  fullreg >>= 1; //Shift D0 out. -> eigentlich nur 15Bit und nicht 16
  resistance = fullreg; //pass the value to the resistance variable
  //note: this is not yet the resistance of the RTD!

  digitalWrite(ChipSel, LOW);
  SPI.transfer(0x80); //80h = 128
  //SPI.transfer(0x00); //144 = 10010000
  SPI.endTransaction();
  digitalWrite(ChipSel, HIGH);

}