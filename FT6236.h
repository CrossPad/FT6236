/*!
 * FT6236.h
 */

#ifndef FT6236_H
#define FT6236_H

#include "Arduino.h"
#include <Wire.h>

#define FT6236_ADDR 0x38           // I2C address
#define FT6236_G_FT5201ID 0xA8     // FocalTech's panel ID
#define FT6236_REG_NUMTOUCHES 0x02 // Number of touch points

#define FT6236_NUM_X 0x33 // Touch X position
#define FT6236_NUM_Y 0x34 // Touch Y position

#define FT6236_REG_MODE 0x00        // Device mode, either WORKING or FACTORY
#define FT6236_REG_GESTURE 0x01     // Gesture ID
#define FT6236_REG_CALIBRATE 0x02   // Calibrate mode
#define FT6236_REG_P1_XH 0x03       // Data port for first touch point, X high 4 bits
#define FT6236_REG_P1_XL 0x04       // Data port for first touch point, X low 8 bits
#define FT6236_REG_P1_YH 0x05       // Data port for first touch point, Y high 4 bits
#define FT6236_REG_P1_YL 0x06       // Data port for first touch point, Y low 8 bits
#define FT6236_REG_P1_WEIGHT 0x07   // Data port for first touch point, pressure
#define FT6236_REG_P1_MISC 0x08     // Data port for first touch point 7:4 gesture ID, 3:0 reserved
#define FT6236_REG_P2_XH 0x09       // Data port for second touch point, X high 4 bits
#define FT6236_REG_P2_XL 0x0A       // Data port for second touch point, X low 8 bits
#define FT6236_REG_P2_YH 0x0B       // Data port for second touch point, Y high 4 bits
#define FT6236_REG_P2_YL 0x0C       // Data port for second touch point, Y low 8 bits
#define FT6236_REG_P2_WEIGHT 0x0D   // Data port for second touch point, pressure
#define FT6236_REG_P2_MISC 0x0E     // Data port for second touch point 7:4 gesture ID, 3:0 reserved
#define FT6236_REG_FILTERCOEF 0x85  // Filter coefficient
#define FT6236_REG_CONTROL 0x86     // Control register
#define FT6236_REG_TIMEENTERMONI 0x87 // Time enter monitor
#define FT6236_REG_PERIODACTIVE 0x88  // Period active
#define FT6236_REG_PERIODMONITOR 0x89 // report rate in monitor mode
#define FT6236_REG_INTERRUPTMODE 0xA4 // Interrupt mode 0: polling, 1: trigger
#define FT6236_REG_PWR_MODE 0xA5        // power mode
#define FT6236_REG_STATE 0xBC           // Running state 
#define FT6236_REG_WORKMODE 0x00    // Work mode
#define FT6236_REG_FACTORYMODE 0x40 // Factory mode
#define FT6236_REG_THRESHHOLD 0x80  // Threshold for touch detection
#define FT6236_REG_POINTRATE 0x88   // Point rate
#define FT6236_REG_FIRMVERS 0xA6    // Firmware version
#define FT6236_REG_CHIPID 0xA3      // Chip selecting
#define FT6236_REG_VENDID 0xA8      // FocalTech's panel ID

#define FT6236_VENDID 0x11  // FocalTech's panel ID
#define FT6206_CHIPID 0x06  // FT6206 ID
#define FT6236_CHIPID 0x36  // FT6236 ID
#define FT6236U_CHIPID 0x64 // FT6236U ID

#define FT6236_DEFAULT_THRESHOLD 128 // Default threshold for touch detection

class TS_Point
{
public:
  TS_Point(void);
  TS_Point(int16_t x, int16_t y, int16_t z);

  bool operator==(TS_Point);
  bool operator!=(TS_Point);

  int16_t x;
  int16_t y;
  int16_t z;
  uint8_t id;
  uint8_t area;
};

class FT6236
{
public:
  enum TS_Gesture
  {
    No_Gesture = 0x00,
    Move_Up = 0x10,
    Move_Right = 0x14,
    Move_Down = 0x18,
    Move_Left = 0x1C,
    Zoom_In = 0x48,
    Zoom_Out = 0x49
  };
  enum TS_Event
  {
    Touch_Down = 0x00,
    Touch_Up = 0x01,
    Contact = 0x02,
    No_Event = 0x03
  };

  FT6236(void);
  void debug(void);
  boolean begin(uint8_t thresh = FT6236_DEFAULT_THRESHOLD, int8_t sda = -1, int8_t scl = -1);
  uint8_t touched(void);
  TS_Point getPoint(uint8_t n = 0);
  TS_Gesture getGesture(void);
  TS_Event getEvent(void);
  bool setPowerMode(bool mode); // 0: active, 1: monitor
  bool setInterruptMode(bool mode); // 0: polling, 1: trigger
  bool setTouchRate(uint8_t rate);
  bool setFilterCoefficient(uint8_t coef);
  bool setTouchThreshold(uint8_t thresh);
  bool setMonitorTime(uint8_t periodActive, uint8_t periodMonitor, uint8_t enterTime = 0x0A);

  uint8_t gesture;
  uint8_t touches;
  uint8_t event[2];
  uint8_t touchWeight[2];
  uint8_t touchMisc[2];
  uint16_t touchX[2], touchY[2], touchID[2];

private:
  void writeRegister8(uint8_t reg, uint8_t val);
  uint8_t readRegister8(uint8_t reg);

  void readData(void);
};

#endif
