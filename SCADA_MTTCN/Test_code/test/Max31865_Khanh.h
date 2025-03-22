


//Biến cho Max31865-PT100
double resistance;
uint8_t reg1, reg2;  //reg1 holds MSB, reg2 holds LSB for RTD
uint16_t fullreg;    //fullreg holds the combined reg1 and reg2
double PT100_Temperature;
//Variables and parameters for the R - T conversion
double Z1, Z2, Z3, Z4, Rt;
double RTDa = 3.9083e-3;
double RTDb = -5.775e-7;
double rpoly = 0;


void convertToTemperature(int8_t CS) {
  Rt = resistance;
  Rt /= 32768;
  Rt *= 419;  //This is now the real resistance in Ohms
  Serial.print("Resistance: ");
  Serial.println(Rt);  //Temperature in Celsius degrees
  Z1 = -RTDa;
  Z2 = RTDa * RTDa - (4 * RTDb);
  Z3 = (4 * RTDb) / 100;
  Z4 = 2 * RTDb;

  PT100_Temperature = Z2 + (Z3 * Rt);
  PT100_Temperature = (sqrt(PT100_Temperature) + Z1) / Z4;

  if (PT100_Temperature >= 0) {//Temperature in Celsius degrees
    return;                             //exit
  }
  //Note: all formulas can be found in the AN-709 application note from Analog Devices
}


void readRegister(int8_t ChipSel) {
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(ChipSel, LOW);

  SPI.transfer(0x80);  //80h = 128 - config register
  SPI.transfer(0xB1);  // B1h = 177 - 1011 0001: set bias high, start 1-shot, 3-Wire
  delay(10);           // Wartezeit fuer Conversion
  //Von Schreiben in Lesen muss High-Low gesetzt werden
  digitalWrite(ChipSel, HIGH);
  digitalWrite(ChipSel, LOW);
  //

  SPI.transfer(0x01);            // Daten sollen ausgelesen werden
  reg1 = SPI.transfer(0xFF);  // 8 Bit vom MSB 
  reg2 = SPI.transfer(0xFF);  // 8 Bit vom LSB
  digitalWrite(ChipSel, HIGH);

  fullreg = reg1;        //read MSB fullreg = 0000 0000 xxxx xxxx = xxxx xxxx 0000 0000 |= xxxx xxxx xxxx xxxx
  fullreg <<= 8;         //Shift to the MSB part
  fullreg |= reg2;       //read LSB and combine it with MSB
  fullreg >>= 1;         //Shift D0 out. -> eigentlich nur 15Bit und nicht 16
  resistance = fullreg;  //pass the value to the resistance variable
  //note: this is not yet the resistance of the RTD!

  digitalWrite(ChipSel, LOW);
  SPI.transfer(0x80);  //80h = 128
  //SPI.transfer(0x90); //144 = 1001 0001; câu lệnh này có hoặc không đều được.
  SPI.endTransaction();
  digitalWrite(ChipSel, HIGH);
}