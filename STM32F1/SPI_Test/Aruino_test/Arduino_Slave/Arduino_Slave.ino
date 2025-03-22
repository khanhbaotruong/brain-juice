#include <SPI.h>

#define LED 4

bool ledState;

bool received;
volatile byte Slavereceived, Slavesend[3];

uint8_t index;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(MISO, OUTPUT);
  ledState = 1;
  Slavesend[0] = 0x3F;
  Slavesend[1] = 0x4F;
  Slavesend[2] = 0x5F;  //Slave truyền bằng cách ghi data vào thanh ghi SPDR
  SPCR |= _BV(SPE);
  received = false;
  index = 1;
  SPDR = Slavesend[0];
  SPI.attachInterrupt();
}

ISR(SPI_STC_vect) {
  Slavereceived = SPDR;

  if (Slavereceived == 0xF) {
    received = 1;
    SPDR = Slavesend[1];
    //delayMicroseconds(10);
  }
  else if (Slavereceived== 0x1F) {
    received = 1;
    SPDR = Slavesend[2];
    //delayMicroseconds(10);
  }
  else if (Slavereceived == 0x2F) {
    received = 1;
    SPDR = Slavesend[0];
    //delayMicroseconds(10);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  if (received == 1) {

    digitalWrite(LED, ledState);
    ledState = !ledState;

    received = 0;
  }
  Serial.println(Slavereceived);
}
