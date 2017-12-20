//  1 = Start Delimiter                                                     [7E]
//  2 = Lenght (MSB)                                                        [00]
//  3 = Lenght (LSB)                                                        [14]
//  4 = API Drame Identifier (16-bit address I/O)                           [83]
//  5 = Sender Address                                                      [XX]
//  6 = Sender Address                                                      [XX]
//  7 = Received signal strength indicator                                  [XX]
//  8 = Option byte (disregard)                                             [00]
//  9 = Number of Samples                                                   [01]
// 10 = Channel Indicator Mask (n/a A5 A4 A3 A2 A1 A0 D8)                   [3E]
// 11 = Channel Indicator Mask (D7 D6 D5 D4 D3 D2 D1 D0)                    [E0]
// 12 = Digital Sample (MSB) - (x  x  x  x  x  x  x  8)                     [00]
// 13 = Digital Sample (LSB) - (7  6  5  4  3  2  1  0)                     [40]
// 14 = Analog Sample (MSB) - Acc Z High byte                               [XX]
// 15 = Analog Sample (LSB) - Acc Z Low  byte                               [XX]
// 16 = Analog Sample (MSB) - Acc Y High byte                               [XX]
// 17 = Analog Sample (LSB) - Acc Y Low  byte                               [XX]
// 18 = Analog Sample (MSB) - Acc X High byte                               [XX]
// 19 = Analog Sample (LSB) - Acc X Low  byte                               [XX]
// 20 = Analog Sample (MSB) - EMG ch1 High byte                             [XX]
// 21 = Analog Sample (LSB) - EMG ch1 Low  byte                             [XX]
// 22 = Analog Sample (MSB) - EMG ch2 High byte                             [XX]
// 23 = Analog Sample (LSB) - EMG ch2 Low  byte                             [XX]
// 24 = Checksum                               [0xFF - (sum(4, ..., 23) & 0xFF)]

#ifndef XBEE_H
#define XBEE_H

#include <Arduino.h>

#define NUMBER_OF_XBEE_DEVICES 2
static const uint16_t DEVICE_ID[] = {0x00E0, 0x00E1};

#define SMOOTHING_FACTOR_ACCELEROMETER 0.15 // 0 -- 1; where smaller values result in smoother averaging
#define SMOOTHING_FACTOR_EMG           0.05 // 0 -- 1; where smaller values result in smoother averaging
#define NUMBER_OF_INIT_XBEE_SAMPLES    100 //uint8_t
#define MEASUREMENTS_UPPER_BOUNDARY    1023

// Calibrated values              {Device0, Device1, ..., DeviceN}
static const float FROM_LOW_X[] = {233.069595, 249.126205};
static const float FROM_LOW_Y[] = {285.760009, 292.928192};
static const float FROM_LOW_Z[] = {230.111206, 175.412002};
static const float FROM_HIGH_X[] = {726.448974, 733.098388};
static const float FROM_HIGH_Y[] = {780.381408, 788.776611};
static const float FROM_HIGH_Z[] = {715.660583, 656.414001};
 // Calibrated at 1g, range of the sensor is -1.5g -- 1.5g
static const float TO_LOW_X[] = {170.66666666, 170.66666666};
static const float TO_LOW_Y[] = {170.66666666, 170.66666666};
static const float TO_LOW_Z[] = {170.66666666, 170.66666666};
static const float TO_HIGH_X[] = {853.33333333, 853.33333333};
static const float TO_HIGH_Y[] = {853.33333333, 853.33333333};
static const float TO_HIGH_Z[] = {853.33333333, 853.33333333};

class XBeeClass{
public:
  XBeeClass(){};
  struct XBeeMeasurementsStruct{
    unsigned char ID[2];
    struct accelerometerStruct{
      float x;
      float y;
      float z;
    } accelerometer;
    struct RPYStruct{
      float roll;
      float pitch;
    } RPY;
    struct EMGStruct{
      uint16_t channel1;
      uint16_t channel2;
    } EMG;
  } XBeeMeasurements[NUMBER_OF_XBEE_DEVICES];
  void begin(HardwareSerial &HWserial, uint32_t baud = 115200);
  void begin(Stream &serial);
  bool read();
  inline void runningAverageReset();
  void accelerometerCalibration(uint8_t deviceIndex, uint16_t numberOfSamples); // one time calibration; ONE DEVICE AT THE TIME (all other devices switched off)!!!
private:
  Stream *XBeeSerial;
  unsigned char packet[20];
  bool isReceived[NUMBER_OF_XBEE_DEVICES];
  float previousAccelerometerX[NUMBER_OF_XBEE_DEVICES], previousAccelerometerY[NUMBER_OF_XBEE_DEVICES], previousAccelerometerZ[NUMBER_OF_XBEE_DEVICES];
  uint16_t previousEMGch1, previousEMGch2;
  inline float runningFilterAccelerometer(uint8_t deviceIndex, float previousMeasurement[], const uint16_t newMeasurement);
  inline uint16_t runningFilterEMG(uint16_t &previousMeasurement, const uint16_t newMeasurement);
  inline float convertToUnits(float accelerometerMeasurement);
  inline float mapFloat(const float x, const float in_min, const float in_max, const float out_min, const float out_max);
  inline float getRoll(float accelerationY, float accelerationZ);
  inline float getPitch(float accelerationX, float accelerationY, float accelerationZ);
  inline void clearRX();
  inline void clearRXAll();
};
extern XBeeClass XBee;

#endif
