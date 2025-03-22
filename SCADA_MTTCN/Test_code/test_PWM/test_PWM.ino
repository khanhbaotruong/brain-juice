// setting PWM properties
const int suongPin = 13;  // GPIO13 - PWM phun sương
const int freq = 15000; // Tần số 15KHz
const int suongChannel = 1; // chọn channel 0
const int resolution = 8; // độ phân giải analog 8 bit, tối đa là 16 bit
unsigned long timemau_soil;
//const int duty_suong = 210;


                                          /*            Thư viện Capacitive_Soil được viết bởi Trương Quang Bảo Khanh                     */

                        /*  Thư viện này được sử dụng với mục đích chuyển đổi tín hiệu analog thành giá trị % độ ẩm đất sử dụng cảm biến Độ ẩm đất điện dung */

                                          /*Các giá trị analog VD: 2751, 2325 v.v... là những giá trị được đo được từ % độ ẩm đất thực tế.*/

                                          /*Các bạn có thể đo các giá trị này bằng cách phơi khô 1 phần đất, sử dụng cân tiểu ly để ước lượng
                                            và suy ra giá trị phần trăm đất với công thức W = (mw x 100)/ms, trong đó mw là khối lượng nước
                                            và ms là khối lượng đất khô.                                                                   */
                                                                                                                        
#define SOIL 33
uint16_t phantramdat = 0;
uint16_t cambiendat;
uint16_t convert_SOIL()
{
  uint16_t per_SOIL = 0;                            //biến này lưu trữ giá trị phần trăm độ ẩm đất
  uint16_t cambiendat = analogRead(SOIL);           //SOIL cần được khai báo là 1 pin cụ thể
  // độ ẩm đất bé hơn 0%
  
  //delay(300);
  if(cambiendat >= 2751)
  {
    per_SOIL = 0;
  }

  // độ ẩm đất đo được từ 0 - 20%
  else if(cambiendat >= 2325 && cambiendat <= 2750)
  {
    per_SOIL = map(cambiendat, 2325, 2750, 20, 0);
  }

  // độ ẩm đất từ 20% - 40%
  else if(cambiendat >= 2020 && cambiendat <= 2324)
  {
    per_SOIL = map(cambiendat, 2020, 2324, 40, 20);
  }

  // độ ẩm đất từ 40% - 60%
  else if(cambiendat >= 1670 && cambiendat <= 2019)
  {
    per_SOIL = map(cambiendat, 1670, 2019, 60, 40);
  }

  // độ ẩm đất từ 60% - 80%
  else if(cambiendat >= 1520 && cambiendat <= 1669)
  {
    per_SOIL = map(cambiendat, 1520, 1669, 80, 60);
  }

  // độ ẩm đất từ 80% - 100%
  else if(cambiendat >= 1415 && cambiendat <= 1519)
  {
    per_SOIL = map(cambiendat, 1415, 1519, 100, 80);
  }

  // độ ẩm đất lớn hơn 100%
  else if(cambiendat <= 1414)
  {
    per_SOIL = 100;
  }
  Serial.print("analog = ");
  Serial.println(cambiendat);
  Serial.print("phantramdat = ");
  Serial.println(phantramdat);
  return per_SOIL;
}
void setup() {
  // put your setup code here, to run once:
  ledcSetup(suongChannel, freq, resolution);
  ledcAttachPin(suongPin, suongChannel);
  pinMode(SOIL, INPUT);
  Serial.begin(115200);
  timemau_soil = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
 // timemau_soil = millis();
  if(millis() - timemau_soil > 2000)
  {
    phantramdat = convert_SOIL();
    timemau_soil = millis();
  }
  //phantramdat = convert_SOIL();
  if(phantramdat >= 0 && phantramdat < 30)
  {
    ledcWrite(suongChannel, 255);
  }
  else if(phantramdat >= 30 && phantramdat < 65)
  {
    ledcWrite(suongChannel, 210);
  }
  else if(phantramdat >= 65 && phantramdat < 80)
  {
    ledcWrite(suongChannel, 180);
  }
  else if(phantramdat >= 80)
  {
    ledcWrite(suongChannel, 0);
  }
  
  // if(phantramdat >= 80 && t < 28.5)
  // {
  //   ledcWrite(suongChannel, 0);
  // }

//}
}
