#include <cvzone.h>
#include <Servo.h>

#define hongngoai 10  // CHO cho servo 6
#define hongngoai2 9
int servo = 6;
int servo2 = 11;
int pos = 0;
int pos1 = 0;
//khai bao cho
int choCounter = 0;     // số lần button được nhấn
int choState = 0;       // trạng thái hiện tại của button
int lastcho_State = 0;  // trạng thái trước đó của button

//khai bao meo
int meoCounter = 0;     // số lần button được nhấn
int meoState = 0;       // trạng thái hiện tại của button
int lastmeo_State = 0;  // trạng thái trước đó của button

//khai bao chim
int chimCounter = 0;  // số lần button được nhấn
int chimCounter_1 = 0;
int chimState = 0;  // trạng thái hiện tại của button
int lastchim_State = 0;

// khai bao bo
int boCounter = 0;  // số lần button được nhấn
int boCounter_1 = 0;
int boState = 0;  // trạng thái hiện tại của button
int lastbo_State = 0;
//
unsigned long hientai = 0;
unsigned long thoigian;

int tre = 2000;

boolean uutien1 = 0;
boolean uutien3 = 0;
boolean uutien2 = 0;
boolean uutien4 = 0;
//
boolean cho = 0;
boolean meo = 0;
boolean chim = 0;
boolean bo = 0;

SerialData serialData(4, 1);  //(numOfValsRec,digitsPerValRec)
int valsRec[4];               // array of int with size numOfValsRec
Servo myservo;
Servo myservo1;

void setup() {


  myservo.attach(servo);
  myservo.write(90);
  myservo1.attach(servo2);
  myservo1.write(90);
  serialData.begin();
  pinMode(hongngoai, INPUT);
  pinMode(hongngoai2, INPUT);
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(8, OUTPUT);
}

void loop() {
  thoigian = millis();
  myservo.attach(servo);
  myservo.write(90);
  myservo1.attach(servo2);
  myservo1.write(90);
  int vatthe1 = digitalRead(hongngoai);
  int vatthe2 = digitalRead(hongngoai2);
  serialData.Get(valsRec);
  //thuattoanuutienchomeo
  if (choCounter - meoCounter >= 1) {
    uutien1 = 1;
  }
  if (meoCounter - choCounter >= 1) {
    uutien1 = 0;
  }
  // phân cấp
  if (((boCounter - choCounter >= 0) || (chimCounter - choCounter >= 0)) || ((boCounter - meoCounter >= 0) || (chimCounter - meoCounter >= 0))) {
    if (((boCounter > choCounter) || (chimCounter > choCounter)) && ((boCounter > meoCounter) || (chimCounter > meoCounter))) {
      uutien3 = 1;
    }
  }

  //thuattoanuutienbochim
  if (boCounter - chimCounter >= 1) {
    uutien2 = 1;
  }
  if (chimCounter - boCounter >= 1) {
    uutien2 = 0;
  }

  //choState=valsRec
  //thuat toan cho
  // 0000
  if (valsRec[0] != lastcho_State) { 
    if (valsRec[0] == 1) {
      choCounter++;
      cho = 1;
      hientai = thoigian;
      // chó tắt mở chân 2
    }
  }
  lastcho_State = valsRec[0];
  if (cho == 1) {
    digitalWrite(2, 1);
    if (thoigian - hientai >= tre) {
      digitalWrite(2, 0);
      cho = 0;
    }
  }
  //
  if (valsRec[1] != lastmeo_State) {
    if (valsRec[1] == 1) {
      meoCounter++;
      meo = 1;
      hientai = thoigian;
    }
  }
  lastmeo_State = valsRec[1];
  if (meo == 1) {
    digitalWrite(3, 1);
    if (thoigian - hientai >= tre) {
      digitalWrite(3, 0);
      meo = 0;
    }
  }
  // thuattoanchim
  if (valsRec[2] != lastchim_State) {
    if (valsRec[2] == 1) {
      chimCounter++;
      chimCounter_1++;
      chim = 1;
      hientai = thoigian;
    }
  }
  lastchim_State = valsRec[2];
  if (chim == 1) {
    digitalWrite(4, 1);
    if (thoigian - hientai >= tre) {
      digitalWrite(4, 0);
      chim = 0;
    }
  }
  //thuat toan bo
  if (valsRec[3] != lastbo_State) {
    if (valsRec[3] == 1) {
      boCounter++;
      boCounter_1++;
      bo = 1;
      hientai = thoigian;
    }
  }
  lastbo_State = valsRec[3];
  if (bo == 1) {
    digitalWrite(8, 1);
    if (thoigian - hientai >= tre) {
      digitalWrite(8, 0);
      bo = 0;
    }
  }
  // cho-meo
  if (((choCounter - 1 >= 0) || (meoCounter - 1 >= 0)) && vatthe1 == 0) {
    if (uutien1 == 1 && uutien3 == 0) {

      quay1();
      quay2();
      delay(10);
      choCounter -= 1;
    }
    if (uutien1 == 0 && uutien3 == 0) {

      quay3();
      quay4();
      delay(10);
      meoCounter -= 1;
    }
  }
  if (uutien3 == 1 && vatthe1 == 0) {
    if (boCounter_1 >= 1) {
      delay(100);
      uutien3 = 0;
      boCounter_1 -= 1;
    }
    if (chimCounter_1 >= 1) {
      delay(100);
      uutien3 = 0;
      chimCounter_1 -= 1;
    }
  }
  //bo-chim
  if (((boCounter - 1 >= 0) || (chimCounter - 1 >= 0)) && vatthe2 == 0) {
    if (uutien2 == 1) {

      quay5();
      quay6();
      delay(30);
      boCounter -= 1;
      uutien3 = 0;
    }
    if (uutien2 == 0) {

      quay7();
      quay8();
      delay(30);
      chimCounter -= 1;
      uutien3 = 0;
    }
  }
}
void quay2() {
  for (int pos = 0; pos <= 90; pos += 30) {
    myservo.write(pos);
    delay(1);
  }
}
void quay1() {
  for (int pos = 90; pos >= 0; pos -= 15) {
    myservo.write(pos);
    delay(30);
  }
}
void quay3() {
  for (int pos1 = 90; pos1 <= 180; pos1 += 15) {
    myservo.write(pos1);
    delay(30);
  }
}
void quay4() {
  for (int pos1 = 180; pos1 >= 90; pos1 -= 30) {
    myservo.write(pos1);
    delay(2);
  }
}
// ham quay cho bo chim
void quay6() {
  for (int pos2 = 0; pos2 <= 90; pos2 += 30) {
    myservo1.write(pos2);
    delay(1);
  }
}
void quay5() {
  for (int pos2 = 90; pos2 >= 0; pos2 -= 15) {
    myservo1.write(pos2);
    delay(30);
  }
}
void quay7() {
  for (int pos3 = 90; pos3 <= 180; pos3 += 15) {
    myservo1.write(pos3);
    delay(30);
  }
}
void quay8() {
  for (int pos3 = 180; pos3 >= 90; pos3 -= 30) {
    myservo1.write(pos3);
    delay(2);
  }
}