
#include <SPI.h>
#include <mcp2515.h>
#include <PNGdec.h>
/*Include PNG pic*/
#include "Fan1.h"  // Image is stored here in an 8-bit array
#include "Fan2.h"  // Image is stored here in an 8-bit array

/*BLDC Screen PNG*/
#include "bldc_bw_btn.h"
#include "bldc_fw_btn.h"
#include "bldc_gear0.h"
#include "bldc_gear1.h"
#include "bldc_gear2.h"
#include "bldc_gear3.h"
#include "bldc_gear4.h"
#include "bldc_gear5.h"
#include "bldc_gear6.h"
#include "bldc_gear7.h"
#include "bldc_gear8.h"
#include "bldc_gear9.h"
#include "bldc_gear10.h" // khi stop, ghi lai jpeg va ghi Start/stop PNG
#include "bldc_speed_10.h"
#include "bldc_speed_20.h"
#include "bldc_speed_30.h"
#include "bldc_speed_40.h"
#include "bldc_speed_50.h"
#include "bldc_speed_60.h"
#include "bldc_speed_70.h"
#include "bldc_speed_80.h"
#include "bldc_speed_90.h"
#include "bldc_speed_100.h"
#include "bldc_speed_110.h"
#include "bldc_speed_120.h"
#include "bldc_start_btn.h"
#include "bldc_stop_btn.h"
#include "bldc_kmh_frame.h"

/*Temperature screen PNG*/
#include "temp_auto_btn.h"
#include "temp_manual_btn.h"
#include "temp_fanoff_btn.h"
#include "temp_fanon_btn.h"
#include "temp_low_status.h"
#include "temp_middle_status.h"
#include "temp_high_status.h"
#include "temp_frame_celcius.h"

/*PSI screen PNG*/

PNG png;  // PNG decoder instance

struct can_frame canMsg;
struct can_frame canMsg_temp_rev;
struct can_frame canMsg_bldc_rev;
struct can_frame canMsg_psi_rev;

/*three update value realtime*/
float temperature = 0.0;
float prv_temperature = 0.0;
uint16_t kmh = 0;
uint16_t prv_kmh = 0;
uint8_t psi = 0;
uint8_t bldc_gear1 = 0;  // value range is 0 to 10

/*background switch*/
bool menu_scr = 1;
bool bldc_scr = 0;
bool bldc_scr_comback = 0;
bool temp_scr = 0;
bool temp_scr_comback = 0;
bool psi_scr = 0;
bool bldc_direction = 0;      // 0 mean forward, 1 mean backward

/*button global status*/
bool temp_fan_mode = 0;  //temp_fan_mode = 0 mean auto fan control, temp_fan_mode = 1 mean manual fan control
bool fan_button = 0;

bool bldc_enable = 0;   // 0 mean stop, 1 mean start



struct MCP2515 mcp2515(17);  // CS pin is GPIO 17

/*defined macros for ILI9341*/

/*Menu layout button and data position*/

//Temperature menu button
#define TEMP_BUTTON_X 160
#define TEMP_BUTTON_Y 60
#define TEMP_BUTTON_W 40
#define TEMP_BUTTON_H 60

// Temperature data position
#define TEMP_DATA_X 213
#define TEMP_DATA_Y 82

// Temperature frame position
#define TEMP_FRAM_X 210
#define TEMP_FRAM_Y 80
#define TEMP_FRAM_W 80
#define TEMP_FRAM_H 20

//Speed button - BLDC
#define SPEED_BUTTON_X  100
#define SPEED_BUTTON_Y  80
#define SPEED_BUTTON_W  60
#define SPEED_BUTTON_H  40

// BLDC data position
#define BLDC_DATA_X 6
#define BLDC_DATA_Y 102

// BLDC frame position
#define BLDC_FRAM_X 5
#define BLDC_FRAM_Y 100
#define BLDC_FRAM_W 94
#define BLDC_FRAM_H 20

/*End of menu layout*/


/*BLDC Screen*/

//BLDC Start/Stop button
#define BLDC_START_STOP_X 14
#define BLDC_START_STOP_Y 67
#define BLDC_START_STOP_W 83
#define BLDC_START_STOP_H 36

//BLDC Forward/Backward button
#define BLDC_FW_BW_X  15
#define BLDC_FW_BW_Y  142
#define BLDC_FW_BW_W  83
#define BLDC_FW_BW_H  36

//BLDC Up button
#define BLDC_UP_GEAR_X  251
#define BLDC_UP_GEAR_Y  152
#define BLDC_UP_GEAR_W  49
#define BLDC_UP_GEAR_H  37

//BLDC Down button
#define BLDC_DOWN_GEAR_X  122
#define BLDC_DOWN_GEAR_Y  152
#define BLDC_DOWN_GEAR_W  49
#define BLDC_DOWN_GEAR_H  37

//GEAR PNG Position
#define GEAR_X  172
#define GEAR_Y  152



//Speedometer PNG position
#define SPEEDOMETER_X  102
#define SPEEDOMETER_Y  41

//Km/h frame
#define KMH_FRAM_X  180
#define KMH_FRAM_Y  94

//Km/h data position
#define KMH_DATA_X  181
#define KMH_DATA_Y  95

/*Temperature screen*/

//Auto-Manual Button
#define AUTO_MANU_BTN_X 33
#define AUTO_MANU_BTN_Y 73
#define AUTO_MANU_BTN_W 83
#define AUTO_MANU_BTN_H 36

//Fan ON - OFF button
#define FAN_ON_OFF_X  33
#define FAN_ON_OFF_Y  142
#define FAN_ON_OFF_W  83
#define FAN_ON_OFF_H  36

//Frame temperature status
#define FRAM_TEMP_STATUS_X  142
#define FRAM_TEMP_STATUS_Y  54
#define FRAM_TEMP_STATUS_W  155
#define FRAM_TEMP_STATUS_H  111

//temp screen frame
#define TEMP_SC_FRAM_X  169
#define TEMP_SC_FRAM_Y  100
#define TEMP_SC_FRAM_W  58
#define TEMP_SC_FRAM_H  20

//temp screen data
#define TEMP_SC_DATA_X  171
#define TEMP_SC_DATA_Y  102



//Information and Home button
#define INFO_HOME_X 280
#define INFO_HOME_Y 0
#define INFO_HOME_W 40
#define INFO_HOME_H 40



/* end of define macros for ILI9341*/


#define MAX_RETRIES 3
#define CAN_ACK_ID_TEMP 0x446  // CAN ID for acknowledgment
#define CAN_ACK_ID_BLDC 0x104  // CAN ID for acknowledgment
#define CAN_ACK_ID_PSI 0x103   // CAN ID for acknowledgment

void send_receive_bldc_mess(); 
void tx_can_frame_temp();
void tx_can_frame_bldc();
void tx_can_frame_psi();
void tft_temp_print();
void fan_rotate();
void updateFanAnimation();
void menu_check_button();
void bldc_check_button();
void bldc_png_default_layout(); // used when firt time get into BLDC screen and after press Stop
void temp_png_default_layout();
void temp_check_button();
void psi_check_button();
void UpdateSpeedometerEvery50ms();
void UpdateTemperateStatusEvery50ms();
void bldc_png_update_layout_after_touch();
void temp_png_update_layout_after_touch();
//--------For ILI---/
#include <FS.h>
#include <SD.h>

#include <TFT_eSPI.h>
#include "Free_Fonts.h"  // Include the header file attached to this sketch
TFT_eSPI tft = TFT_eSPI();

// JPEG decoder library
#include <JPEGDecoder.h>

bool pressed = 0;
uint16_t xpos = 0, ypos = 0;  // To store the touch coordinates

//-----------------/

#define MAX_IMAGE_WIDTH 320  // Adjust for your images

int16_t xpos_png = 0;
int16_t ypos_png = 0;

/*delay with millis*/
// Biến thời gian để kiểm soát tần suất cập nhật hình ảnh
unsigned long previousMillis = 0;
const long interval = 150;  // 150ms


void setup() {

  digitalWrite(21, HIGH);  // Touch controller chip select (if used)
  digitalWrite(15, HIGH);  // TFT screen chip select
  digitalWrite(5, HIGH);   // SD card chips select, must use GPIO 5 (ESP32 SS)
  digitalWrite(17, HIGH);  // MCP2515 CS

  Serial.begin(115200);
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  // Prepare CAN message

  //-----ILI---



  tft.begin();

  if (!SD.begin(5, tft.getSPIinstance())) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  Serial.println("initialisation done.");

  // Set the rotation to the orientation you wish to use in your project before calibration
  // (the touch coordinates returned then correspond to that rotation only)
  tft.setRotation(1);

  // Calibrate the touch screen and retrieve the scaling factors
  touch_calibrate();

  tft.fillScreen(TFT_BLACK);

  tft.setRotation(1);  // ngang = láncape
  tft.fillScreen(random(0xFFFF));
  drawSdJpeg("/welcome.jpeg", 0, 0);
  while (!(tft.getTouch(&xpos, &ypos)));
  tft.fillScreen(random(0xFFFF));
  drawSdJpeg("/menu.jpeg", 0, 0);

  //tft.fillRect(TEMP_FRAM_X, TEMP_FRAM_Y, TEMP_FRAM_W, TEMP_FRAM_H, TFT_WHITE); // Tô màu nền trước


}

void loop() {

  while (menu_scr == 1) {
    menu_check_button();
    if(menu_scr == 0)
    {
      break;
    }
    if(bldc_enable == 1)
    {
    send_receive_bldc_mess();
    }

    send_receive_temp_mess();

    fan_rotate();
    Serial.println("MENU");
  }
  while (bldc_scr == 1)
  {
    bldc_check_button();
    if(menu_scr == 1 && bldc_scr == 0)
    {
      prv_kmh = 300;
      bldc_scr_comback = 1;
      break;
    }
    send_receive_temp_mess();
    fan_rotate();
  }
  while(temp_scr == 1)
  {
    temp_check_button();
    if(menu_scr == 1 && temp_scr == 0)
    {
      prv_temperature = 300;
      temp_scr_comback = 1;
      break;
    }
    fan_rotate();
  }

}

/*Send and receive BLDC mess to and from STM32*/
void send_receive_bldc_mess() {

  /*Prepare bldc tx header*/
  tx_can_frame_bldc();

  // Serial.println("ACK transmit BLDC");
  // Serial.print("Data0 = ");
  // Serial.println(canMsg.data[0]);
  // Serial.print("Data1 = ");
  // Serial.println(canMsg.data[1]);
  // Serial.print("Data2 = ");
  // Serial.println(canMsg.data[2]);


  bool messageSent = false;
  int retries = 0;

  while (!messageSent && retries < MAX_RETRIES) {
    if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {

      // Wait for acknowledgment
      unsigned long startTime = millis();
      bool ackReceived = false;

      while (millis() - startTime < 500) {  // Wait up to 500ms for an ACK
        if (mcp2515.readMessage(&canMsg_bldc_rev) == MCP2515::ERROR_OK) {
          if (canMsg_bldc_rev.can_id == CAN_ACK_ID_BLDC) {
            canMsg_bldc_rev.can_id = 0;
            ackReceived = true;
            break;
          }
        }
      }

      if (ackReceived) {
        // Serial.println("ACK received");
        // Serial.print("Data0 = ");
        // Serial.println(canMsg_bldc_rev.data[0]);
        // Serial.print("Data1 = ");
        // Serial.println(canMsg_bldc_rev.data[1]);
        messageSent = true;
      } else {
        // Serial.println("ACK not received, retrying...");
        retries++;
      }
    } else {
      // Serial.println("Error sending message, retrying...");
      retries++;
    }
  }

  if (!messageSent) {
    // Serial.println("Failed to send message after retries");
  }
  tft_bldc_print();
}

/*Send and receive temperate mess to and from Stm32*/
void send_receive_temp_mess() {
  /*Prepare bldc tx header*/
  tx_can_frame_temp();

  // Serial.println("ACK transmit TEMP");
  // Serial.print("Data0 = ");
  // Serial.println(canMsg.data[0]);
  // Serial.print("Data1 = ");
  // Serial.println(canMsg.data[1]);


  bool messageSent = false;
  int retries = 0;

  while (!messageSent && retries < MAX_RETRIES) {
    if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {

      // Wait for acknowledgment
      unsigned long startTime = millis();
      bool ackReceived = false;

      while (millis() - startTime < 500) {  // Wait up to 500ms for an ACK
        if (mcp2515.readMessage(&canMsg_temp_rev) == MCP2515::ERROR_OK) {
          if (canMsg_temp_rev.can_id == CAN_ACK_ID_TEMP) {
            canMsg_temp_rev.can_id = 0;
            ackReceived = true;
            break;
          }
        }
      }

      if (ackReceived) {
        // Serial.println("ACK received");
        // Serial.print("Data0 = ");
        // Serial.println(canMsg_temp_rev.data[0]);
        // Serial.print("Data1 = ");
        // Serial.println(canMsg_temp_rev.data[1]);
        messageSent = true;
      } else {
        // Serial.println("ACK not received, retrying...");
        retries++;
      }
    } else {
      // Serial.println("Error sending message, retrying...");
      retries++;
    }

    
  }

  if (!messageSent) {
    // Serial.println("Failed to send message after retries");
  }
  tft_temp_print();
}

void touch_calibrate() {
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // Calibrate
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(20, 0);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.println("Touch corners as indicated");

  tft.setTextFont(1);
  tft.println();

  tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

  Serial.println();
  Serial.println();
  Serial.println("// Use this calibration code in setup():");
  Serial.print("  uint16_t calData[5] = ");
  Serial.print("{ ");

  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(calData[i]);
    if (i < 4) Serial.print(", ");
  }

  Serial.println(" };");
  Serial.print("  tft.setTouch(calData);");
  Serial.println();
  Serial.println();

  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("Calibration complete!");
  tft.println("Calibration code sent to Serial port.");

  delay(500);
}

//####################################################################################################
// Draw a JPEG on the TFT pulled from SD Card
//####################################################################################################
// xpos, ypos is top left corner of plotted image
void drawSdJpeg(const char *filename, int xpos, int ypos) {

  // Open the named file (the Jpeg decoder library will close it)
  File jpegFile = SD.open(filename, FILE_READ);  // or, file handle reference for SD library

  if (!jpegFile) {
    Serial.print("ERROR: File \"");
    Serial.print(filename);
    Serial.println("\" not found!");
    return;
  }

  Serial.println("===========================");
  Serial.print("Drawing file: ");
  Serial.println(filename);
  Serial.println("===========================");

  // Use one of the following methods to initialise the decoder:
  bool decoded = JpegDec.decodeSdFile(jpegFile);  // Pass the SD file handle to the decoder,
  //bool decoded = JpegDec.decodeSdFile(filename);  // or pass the filename (String or character array)

  if (decoded) {
    // print information about the image to the serial port
    jpegInfo();
    // render the image onto the screen at given coordinates
    jpegRender(xpos, ypos);
  } else {
    Serial.println("Jpeg file format not supported!");
  }
}

//####################################################################################################
// Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void jpegRender(int xpos, int ypos) {

  //jpegInfo(); // Print information from the JPEG file (could comment this line out)

  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  bool swapBytes = tft.getSwapBytes();
  tft.setSwapBytes(true);

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = jpg_min(mcu_w, max_x % mcu_w);
  uint32_t min_h = jpg_min(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // Fetch data from the file, decode and display
  while (JpegDec.read()) {  // While there is more data in the file
    pImg = JpegDec.pImage;  // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)

    // Calculate coordinates of top left corner of current MCU
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;

    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // copy pixels into a contiguous block
    if (win_w != mcu_w) {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++) {
        p += mcu_w;
        for (int w = 0; w < win_w; w++) {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }

    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;

    // draw image MCU block only if it will fit on the screen
    if ((mcu_x + win_w) <= tft.width() && (mcu_y + win_h) <= tft.height())
      tft.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    else if ((mcu_y + win_h) >= tft.height())
      JpegDec.abort();  // Image has run off bottom of screen so abort decoding
  }

  tft.setSwapBytes(swapBytes);

  showTime(millis() - drawTime);  // These lines are for sketch testing only
}

//####################################################################################################
// Print image information to the serial port (optional)
//####################################################################################################
// JpegDec.decodeFile(...) or JpegDec.decodeArray(...) must be called before this info is available!
void jpegInfo() {

  // Print information extracted from the JPEG file
  Serial.println("JPEG image info");
  Serial.println("===============");
  Serial.print("Width      :");
  Serial.println(JpegDec.width);
  Serial.print("Height     :");
  Serial.println(JpegDec.height);
  Serial.print("Components :");
  Serial.println(JpegDec.comps);
  Serial.print("MCU / row  :");
  Serial.println(JpegDec.MCUSPerRow);
  Serial.print("MCU / col  :");
  Serial.println(JpegDec.MCUSPerCol);
  Serial.print("Scan type  :");
  Serial.println(JpegDec.scanType);
  Serial.print("MCU width  :");
  Serial.println(JpegDec.MCUWidth);
  Serial.print("MCU height :");
  Serial.println(JpegDec.MCUHeight);
  Serial.println("===============");
  Serial.println("");
}

//####################################################################################################
// Show the execution time (optional)
//####################################################################################################
// WARNING: for UNO/AVR legacy reasons printing text to the screen with the Mega might not work for
// sketch sizes greater than ~70KBytes because 16-bit address pointers are used in some libraries.

// The Due will work fine with the HX8357_Due library.

void showTime(uint32_t msTime) {
  //tft.setCursor(0, 0);
  //tft.setTextFont(1);
  //tft.setTextSize(2);
  //tft.setTextColor(TFT_WHITE, TFT_BLACK);
  //tft.print(F(" JPEG drawn in "));
  //tft.print(msTime);
  //tft.println(F(" ms "));
  Serial.print(F(" JPEG drawn in "));
  Serial.print(msTime);
  Serial.println(F(" ms "));
}

void tx_can_frame_temp() {
  canMsg.can_id = 0x047;  // CAN ID
  canMsg.can_dlc = 2;     // Data length code (number of bytes)
  if (temp_fan_mode == 1) {
    if (fan_button == 1) {
      canMsg.data[0] = 0x01;  //Manual control ON
      canMsg.data[1] = 0x01;  //Fan ON
    } else {
      canMsg.data[0] = 0x01;  //Manual control ON
      canMsg.data[1] = 0x00;  //Fan OFF 
    }
  } else {
    canMsg.data[0] = 0x00;  //Auto control ON
    canMsg.data[1] = 0x00;  //Fan OFF
  }
}

void tx_can_frame_bldc() {
  canMsg.can_id = 0x037;  // CAN ID
  canMsg.can_dlc = 3;     // Data length code (number of bytes)
  if (bldc_enable == 1) {
    if (bldc_direction == 1) {
      canMsg.data[0] = 0x01;       //start bldc
      canMsg.data[1] = 0x01;       // move forward
      canMsg.data[2] = bldc_gear1;  // gear power speed = bldc_gear1 variable
    } else {
      canMsg.data[0] = 0x01;       //start bldc
      canMsg.data[1] = 0x00;       // move backward
      canMsg.data[2] = bldc_gear1;  // gear power speed = bldc_gear1 variable
    }
  } else {
    canMsg.data[0] = 0x00;  // stop bldc
    canMsg.data[1] = 0x01;  // move forward
    canMsg.data[2] = 0x00;  // gear power speed = 0
  }
}

void tft_temp_print() {
  // Ghép hai byte thành uint16_t
  uint16_t temp = ((uint16_t)canMsg_temp_rev.data[0] << 8) | canMsg_temp_rev.data[1];

  // Chuyển đổi uint16_t về float
  temperature = (float)temp / 100.0;  // Chia ngược lại theo cách nhân ban đầu
  // Serial.print("temperature = ");
  // Serial.println(temperature);
  //drawSdJpeg("/temp_data.jpg", TEMP_FRAM_X, TEMP_FRAM_Y); // -> lam nhu vay thi no se bi giat man hinh

  if(menu_scr == 1)
  {
    tft.fillRect(TEMP_FRAM_X, TEMP_FRAM_Y, TEMP_FRAM_W, TEMP_FRAM_H, TFT_WHITE);  // Tô màu nền trước
    tft.setTextColor(TFT_BLACK);
    tft.setFreeFont(FSB9);  // Select the font
    tft.drawString(String(temperature, 1) + " *C", TEMP_DATA_X, TEMP_DATA_Y, GFXFF);
  }
}

void tft_bldc_print() {
  // Ghép hai byte thành uint16_t
  uint16_t rpm = ((uint16_t)canMsg_bldc_rev.data[0] << 8) | canMsg_bldc_rev.data[1];
  float wheelDiameter = 0.2; // Đường kính bánh xe (m)
  float wheelCircumference = 3.1416 * wheelDiameter; // Chu vi bánh xe (m)
    
    // Công thức chuyển đổi RPM -> km/h
    kmh = (uint16_t)(10*(rpm * wheelCircumference * 60 / 1000));
    if(kmh > 131)
    {
      kmh = 131;
    }
    else if(kmh < 0)
    {
      kmh = 0;
    }
  // Chuyển đổi uint16_t về float
  // Serial.print("rpm = ");
  // Serial.println(rpm);
  // Serial.print("km/h = ");
  // Serial.println(kmh);

  if(menu_scr == 1)
  {
    tft.fillRect(BLDC_FRAM_X, BLDC_FRAM_Y, BLDC_FRAM_W, BLDC_FRAM_H, TFT_WHITE);  // Tô màu nền trước
    tft.setTextColor(TFT_BLACK);
    tft.setFreeFont(FSB9);  // Select the font
    tft.drawString(String(kmh) + " km/h", BLDC_DATA_X, BLDC_DATA_Y, GFXFF);
  }
}

//=========================================v==========================================
//                                      pngDraw
//====================================================================================
// This next function will be called during decoding of the png file to
// render each image line to the TFT.  If you use a different TFT library
// you will need to adapt this function to suit.
// Callback function to draw pixels to the display
void pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WIDTH];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos_png, ypos_png + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}


void displayPNG(const uint8_t *imageData, size_t imageSize, int16_t x, int16_t y) {
  // Gán tọa độ mới vào biến toàn cục
  xpos_png = x;
  ypos_png = y;

  int16_t rc = png.openFLASH(const_cast<uint8_t *>(imageData), imageSize, pngDraw);
  if (rc == PNG_SUCCESS) {
    // Serial.println("Successfully opened PNG file");
    // Serial.printf("Image specs: (%d x %d), %d bpp, pixel type: %d\n",
    //               png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());

    tft.startWrite();
    uint32_t dt = millis();
    rc = png.decode(NULL, 0);
    // Serial.print(millis() - dt);
    // Serial.println(" ms");
    tft.endWrite();
  } else {
    // Serial.println("Failed to open PNG file");
  }
}

void fan_rotate() {
  if (((temp_fan_mode == 1) && (fan_button == 1)) || (temperature >= 33)) {
    updateFanAnimation();  // Gọi hàm cập nhật quạt theo millis()
  } else {
    displayPNG(Fan2, sizeof(Fan2), 2, 202);
  }
}

void updateFanAnimation() {
  static bool fanState = false;  // Trạng thái của quạt

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Cập nhật thời gian mới nhất

    // Hiển thị ảnh quạt theo trạng thái hiện tại
    if (fanState) {
      displayPNG(Fan1, sizeof(Fan1), 2, 202);
    } else {
      displayPNG(Fan2, sizeof(Fan2), 2, 202);
    }

    fanState = !fanState;  // Đảo trạng thái quạt
  }
}
void menu_check_button()
{
  if (tft.getTouch(&xpos, &ypos)) {
    //Infor pressed
    if ((xpos > INFO_HOME_X) && (xpos < (INFO_HOME_X + INFO_HOME_W))) {
      if ((ypos > INFO_HOME_Y) && (ypos <= (INFO_HOME_Y + INFO_HOME_H))) {
        tft.setRotation(1);  // ngang = láncape
        //tft.fillScreen(random(0xFFFF));
        drawSdJpeg("/info.jpeg", 0, 0);
        while (!(tft.getTouch(&xpos, &ypos)));
        drawSdJpeg("/menu.jpeg", 0, 0);
      }
    }
    //Speed pressed
    if ((xpos > SPEED_BUTTON_X) && (xpos < (SPEED_BUTTON_X + SPEED_BUTTON_W))) {
      if ((ypos > SPEED_BUTTON_Y) && (ypos <= (SPEED_BUTTON_Y + SPEED_BUTTON_H))) {
        bldc_scr = 1;
        menu_scr = 0;
        bldc_png_default_layout();
      }
    }
    //Temperature pressed
    if ((xpos > TEMP_BUTTON_X) && (xpos < (TEMP_BUTTON_X + TEMP_BUTTON_W))) {
      if ((ypos > TEMP_BUTTON_Y) && (ypos <= (TEMP_BUTTON_Y + TEMP_BUTTON_H))) {
        temp_scr = 1;
        menu_scr = 0;
        temp_png_default_layout();
      }
    }

  }
}

void bldc_check_button()
{
  if (tft.getTouch(&xpos, &ypos)) {

    //Forward/Backward pressed
    if ((xpos > BLDC_FW_BW_X) && (xpos < (BLDC_FW_BW_X + BLDC_FW_BW_W))) {
      if ((ypos > BLDC_FW_BW_Y) && (ypos <= (BLDC_FW_BW_Y + BLDC_FW_BW_H))) {
        delay(30);
        bldc_direction = !bldc_direction;
      }
    }
    //Start/Stop pressed
    if ((xpos > BLDC_START_STOP_X) && (xpos < (BLDC_START_STOP_X + BLDC_START_STOP_W))) {
      if ((ypos > BLDC_START_STOP_Y) && (ypos <= (BLDC_START_STOP_Y + BLDC_START_STOP_H))) {
        delay(30);
        bldc_enable = !bldc_enable;
      }
    }
    // Up pressed
    if ((xpos > BLDC_UP_GEAR_X) && (xpos < (BLDC_UP_GEAR_X + BLDC_UP_GEAR_W))) {
      if ((ypos > BLDC_UP_GEAR_Y) && (ypos <= (BLDC_UP_GEAR_Y + BLDC_UP_GEAR_H))) {
        delay(30);
        bldc_gear1++;
        if(bldc_gear1 > 10)
        {
          bldc_gear1 = 10;
        }
      }
    }
    // Down pressed
    if ((xpos > BLDC_DOWN_GEAR_X) && (xpos < (BLDC_DOWN_GEAR_X + BLDC_DOWN_GEAR_W))) {
      if ((ypos > BLDC_DOWN_GEAR_Y) && (ypos <= (BLDC_DOWN_GEAR_Y + BLDC_DOWN_GEAR_H))) {
        delay(30);
        if (bldc_gear1 > 0) {
          bldc_gear1--;
        }
      }
    }
    if ((xpos > INFO_HOME_X) && (xpos < (INFO_HOME_X + INFO_HOME_W))) {
      if ((ypos > INFO_HOME_Y) && (ypos <= (INFO_HOME_Y + INFO_HOME_H))) {
        delay(30 );

        bldc_scr = 0;
        menu_scr = 1;
        
      }
    }
    

    //tft.fillCircle(xpos, xpos, 2, TFT_BLACK);
    // Serial.print("x,y = ");
    // Serial.print(xpos);
    // Serial.print(",");
    // Serial.println(ypos);
    
    pressed = 1;
  }
  

  //send can mess to STM32
  send_receive_bldc_mess();
  UpdateSpeedometerEvery50ms();

  //update layout
    bldc_png_update_layout_after_touch();
}

void bldc_png_default_layout()
{
  drawSdJpeg("/BLDC.jpg", 0, 0);
  displayPNG(Stop_button, sizeof(Stop_button), BLDC_START_STOP_X, BLDC_START_STOP_Y);
  //displayPNG(Forward_button, sizeof(Forward_button), BLDC_FW_BW_X, BLDC_FW_BW_Y);
  displayPNG(Backward_button, sizeof(Backward_button), BLDC_FW_BW_X, BLDC_FW_BW_Y);
}

void bldc_png_update_layout_after_touch()
{
  if(pressed == 1 || bldc_scr_comback == 1)
  {
    bldc_scr_comback = 0;
    pressed = 0;
    if(bldc_enable == 1)
    {
      displayPNG(Start_button, sizeof(Start_button), BLDC_START_STOP_X, BLDC_START_STOP_Y); // change to start bldc
    }
    else{
      //bldc_png_default_layout();
      displayPNG(Stop_button, sizeof(Stop_button), BLDC_START_STOP_X, BLDC_START_STOP_Y); // change to stop bldc
    }

    switch (bldc_gear1) {
      case 0:
          displayPNG(N_Gear, sizeof(N_Gear), GEAR_X, GEAR_Y);
          break;
      case 1:
          displayPNG(I_Gear, sizeof(I_Gear), GEAR_X, GEAR_Y);
          break;
      case 2:
          displayPNG(II_Gear, sizeof(II_Gear), GEAR_X, GEAR_Y);
          break;
      case 3:
          displayPNG(III_Gear, sizeof(III_Gear), GEAR_X, GEAR_Y);
          break;
      case 4:
          displayPNG(IV_Gear, sizeof(IV_Gear), GEAR_X, GEAR_Y);
          break;
      case 5:
          displayPNG(V_Gear, sizeof(V_Gear), GEAR_X, GEAR_Y);
          break;
      case 6:
          displayPNG(VI_Gear, sizeof(VI_Gear), GEAR_X, GEAR_Y);
          break;
      case 7:
          displayPNG(VII_Gear, sizeof(VII_Gear), GEAR_X, GEAR_Y);
          break;
      case 8:
          displayPNG(VIII_Gear, sizeof(VIII_Gear), GEAR_X, GEAR_Y);
          break;
      case 9:
          displayPNG(IX_Gear, sizeof(IX_Gear), GEAR_X, GEAR_Y);
          break;
      case 10:
          displayPNG(X_Gear, sizeof(X_Gear), GEAR_X, GEAR_Y);
          break;
    }

    if(bldc_direction == 1)
    {
      displayPNG(Forward_button, sizeof(Forward_button), BLDC_FW_BW_X, BLDC_FW_BW_Y);
    }
    else{
      displayPNG(Backward_button, sizeof(Backward_button), BLDC_FW_BW_X, BLDC_FW_BW_Y);
    }

    if(menu_scr == 1)
    {
      tft.setRotation(1);  // ngang = láncape
      drawSdJpeg("/menu.jpeg", 0, 0);
    } 
  }

}

void UpdateSpeedometerEvery50ms() {
    static uint32_t previousMillis = 0;  // Lưu thời điểm lần cuối kiểm tra
    uint32_t currentMillis = millis();   // Lấy thời gian hiện tại

    if (currentMillis - previousMillis >= 50) {  
        previousMillis = currentMillis;  // Cập nhật thời gian kiểm tra

      if(kmh != prv_kmh)
      {
      
      if (kmh >= 0 && kmh < 10) {
        displayPNG(kmh10, sizeof(kmh10), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 10 && kmh < 20) {
          displayPNG(kmh20, sizeof(kmh20), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 20 && kmh < 30) {
          displayPNG(kmh30, sizeof(kmh30), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 30 && kmh < 40) {
          displayPNG(kmh40, sizeof(kmh40), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 40 && kmh < 50) {
          displayPNG(kmh50, sizeof(kmh50), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 50 && kmh < 60) {
          displayPNG(kmh60, sizeof(kmh60), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 60 && kmh < 70) {
          displayPNG(kmh70, sizeof(kmh70), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 70 && kmh < 80) {
          displayPNG(kmh80, sizeof(kmh80), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 80 && kmh < 90) {
          displayPNG(kmh90, sizeof(kmh90), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 90 && kmh < 100) {
          displayPNG(kmh100, sizeof(kmh100), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 100 && kmh < 110) {
          displayPNG(kmh110, sizeof(kmh110), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 110 && kmh < 120) {
          displayPNG(kmh120, sizeof(kmh120), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else if (kmh >= 120 && kmh < 132) {
          displayPNG(kmh120, sizeof(kmh120), SPEEDOMETER_X, SPEEDOMETER_Y);
      } else {
          
      }

        displayPNG(Framefordata, sizeof(Framefordata), KMH_FRAM_X, KMH_FRAM_Y); // change to start bldc
        tft.setTextColor(TFT_GOLD);
        tft.setFreeFont(FSB18);  // Select the font
        tft.drawString(String(kmh), KMH_DATA_X, KMH_DATA_Y, GFXFF);
        prv_kmh = kmh;
      }
    }
}

/*code for temp screen*/

void temp_check_button()
{
  if (tft.getTouch(&xpos, &ypos)) {

    //Auto/Manual
    if ((xpos > AUTO_MANU_BTN_X) && (xpos < (AUTO_MANU_BTN_X + AUTO_MANU_BTN_W))) {
      if ((ypos > AUTO_MANU_BTN_Y) && (ypos <= (AUTO_MANU_BTN_Y + AUTO_MANU_BTN_H))) {
        delay(40);
        temp_fan_mode = !temp_fan_mode;
      }
    }
    //Start/Stop pressed
    if ((xpos > FAN_ON_OFF_X) && (xpos < (FAN_ON_OFF_X + FAN_ON_OFF_W))) {
      if ((ypos > FAN_ON_OFF_Y) && (ypos <= (FAN_ON_OFF_Y + FAN_ON_OFF_H))) {
        delay(40);
        //change state fan only when manual control is enabled
        if(temp_fan_mode == 1)
        {
          fan_button = !fan_button;
        }
      }
    }
    //Menu pressed
    if ((xpos > INFO_HOME_X) && (xpos < (INFO_HOME_X + INFO_HOME_W))) {
      if ((ypos > INFO_HOME_Y) && (ypos <= (INFO_HOME_Y + INFO_HOME_H))) {
        delay(40);

        temp_scr = 0;
        menu_scr = 1;
        
      }
    }
    

    //tft.fillCircle(xpos, xpos, 2, TFT_BLACK);
    // Serial.print("x,y = ");
    // Serial.print(xpos);
    // Serial.print(",");
    // Serial.println(ypos);
    
    pressed = 1;
  }


  send_receive_temp_mess();
  UpdateTemperateStatusEvery50ms();
  temp_png_update_layout_after_touch();
}

void temp_png_default_layout()
{
  drawSdJpeg("/Temperature.jpg", 0, 0);
  displayPNG(Automode_button, sizeof(Automode_button), AUTO_MANU_BTN_X, AUTO_MANU_BTN_Y);
}

void UpdateTemperateStatusEvery50ms()
{
  static uint32_t previousMillis = 0;  // Lưu thời điểm lần cuối kiểm tra
    uint32_t currentMillis = millis();   // Lấy thời gian hiện tại

    if (currentMillis - previousMillis >= 50) {  
        previousMillis = currentMillis;  // Cập nhật thời gian kiểm tra

      if(temperature != prv_temperature)
      {
      
        if (temperature >= 25 && temperature < 30) {
            displayPNG(Low_Screen, sizeof(Low_Screen), FRAM_TEMP_STATUS_X, FRAM_TEMP_STATUS_Y);
        } else if (temperature >= 30 && temperature < 33) {
            displayPNG(Middle_Screen, sizeof(Middle_Screen), FRAM_TEMP_STATUS_X, FRAM_TEMP_STATUS_Y);
        } else if (temperature >= 33) {
            displayPNG(High_Screen, sizeof(High_Screen), FRAM_TEMP_STATUS_X, FRAM_TEMP_STATUS_Y);
        } else {
            
        }

        displayPNG(framefordata_Temp, sizeof(framefordata_Temp), TEMP_SC_FRAM_X, TEMP_SC_FRAM_Y); 
        tft.setTextColor(TFT_GOLD);
        tft.setFreeFont(FSB9);  // Select the font
        tft.drawString(String(temperature, 1) + " *C", TEMP_SC_DATA_X, TEMP_SC_DATA_Y, GFXFF);
        prv_temperature = temperature;
      }
    }
}

void temp_png_update_layout_after_touch()
{
  if(pressed == 1 || temp_scr_comback == 1)
  {
    temp_scr_comback = 0;
    pressed = 0;
    if(temp_fan_mode == 0)
    {
      displayPNG(Automode_button, sizeof(Automode_button), AUTO_MANU_BTN_X, AUTO_MANU_BTN_Y); // change to auto button
    }
    else{
      displayPNG(Manual_button, sizeof(Manual_button), AUTO_MANU_BTN_X, AUTO_MANU_BTN_Y); // change to manual button

      if(fan_button == 1)
      {
        displayPNG(Fanon_button, sizeof(Fanon_button), FAN_ON_OFF_X, FAN_ON_OFF_Y);
      }
      else{
        displayPNG(Fanoff_button, sizeof(Fanoff_button), FAN_ON_OFF_X, FAN_ON_OFF_Y);
      }
    }


    if(menu_scr == 1)
    {
      tft.setRotation(1);  // ngang = láncape
      drawSdJpeg("/menu.jpeg", 0, 0);
    } 
  }
}