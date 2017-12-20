#include <Arduino.h>
#include "XBee.h"
#include "Dynamixel_Serial.h"
#include "CrustCrawler.h"

#define buttonYES 3

//%%%%%%%%%%%%%%%%%%%//
// Tremor supression //
//%%%%%%%%%%%%%%%%%%%//
#define GAIN   2.028454208e+05
#define FILTER_AVERAGER 3
#define NUMBER_OF_CHANNELS_FOR_FILTERING 2
struct boxey{
  int16_t emgData, outputVal;
  float xv[4], yv[4];
} box_channel[NUMBER_OF_CHANNELS_FOR_FILTERING];
void filterloop();


//%%%%%%%%%%//
// SHOULDER //
//%%%%%%%%%%//
#define NUMBER_OF_SHOULDER_CALIBRATION_POINTS 5
#define SUM_CALIBRATION_POSITION 13.0895
#define SUM_CALIBRATION_POSITION_SQUARED 37.0078
static const float calibrationPosition[NUMBER_OF_SHOULDER_CALIBRATION_POINTS] = {3.6647, 3.1416, 2.6185, 2.0939, 1.5708};
struct shoulderEMGCalibrationStruct{
  uint16_t points[NUMBER_OF_SHOULDER_CALIBRATION_POINTS];
  float intercept;
  float slope;
} shoulderEMGCalibration[2];
void calibrationAccelerometerAndShoulderEMG();
void regression(uint8_t index, uint16_t emgDataPoints[NUMBER_OF_SHOULDER_CALIBRATION_POINTS]);
inline int32_t runningFilterPosition(int32_t &previousMeasurement, const int32_t newMeasurement);
#define SMOOTHING_FACTOR_SHOULDER_1 0.1
int32_t previousPositionShoulder1;
void shoulderHAA();


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
// Accelerometer for joint 2 and 3 //
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
struct accelerationCalibrationStruct{
  float roll[NUMBER_OF_SHOULDER_CALIBRATION_POINTS];
  float rollOffset;
} accelerationCalibration[NUMBER_OF_XBEE_DEVICES];
#define POSITION_UNIT_FACTOR 651.739491961316 // (2048/pi)
int32_t armPositions[NUMBER_OF_DYNAMIXEL_DEVICES_ARM] = {2048, 2048+1024, 2048};
void elbow();
void shoulderFE();


//%%%%%%%%%//
// GRIPPER //
//%%%%%%%%%//
struct gripperEMGCalibrationStruct{
  float graspMaxEMG;
  uint16_t gripperNoise;
  uint16_t releaseNoise;
} gripperEMGCalibration;
#define GRIPPER_EMG_TO_PWM_CONVERION 0.0009775171 // 1.0Nm/1024
void calibrationGripperEMG();
void gripper();


uint32_t startTime;
void setup(){
  Serial.begin(115200);
  XBee.begin(Serial2, 115200);
  // XBee.accelerometerCalibration(0, 5000); // one time only

  pinMode(buttonYES, INPUT);
  uint8_t j = 0;
  while (j < NUMBER_OF_INIT_XBEE_SAMPLES){
    if (XBee.read()){
      box_channel[0].emgData = XBee.XBeeMeasurements[1].EMG.channel1;
      box_channel[1].emgData = XBee.XBeeMeasurements[1].EMG.channel2;
      for (size_t i = 0; i < 17; i++){
         filterloop();
      }
      j++;
    }
  }
  CrustCrawler.begin();
  CrustCrawler.setGoalPosition(armPositions);
  CrustCrawler.release();

  calibrationAccelerometerAndShoulderEMG();
  calibrationGripperEMG();
  startTime = millis();
}


void loop(){
  if (XBee.read()){
      box_channel[0].emgData = XBee.XBeeMeasurements[1].EMG.channel1;
      box_channel[1].emgData = XBee.XBeeMeasurements[1].EMG.channel2;
      for (size_t i = 0; i < 6; i++){
         filterloop();
      }

    shoulderHAA(); //EMG
    shoulderFE(); //accelerometer
    elbow(); //accelerometer
    gripper(); //EMG
    CrustCrawler.setGoalPosition(armPositions);
  }
  if (millis() - startTime > 3000){
      CrustCrawler.correctTrajectory();
  }
}


void filterloop(){
  for (size_t i = 0; i < NUMBER_OF_CHANNELS_FOR_FILTERING; i++) {
    int16_t outputValAve = 0;
    box_channel[i].xv[3] = box_channel[i].emgData / GAIN;
    for (size_t j = 0; j < FILTER_AVERAGER; j++) {int32_t armPositions[NUMBER_OF_DYNAMIXEL_DEVICES_ARM] = {2048, 3072, 2048};
        box_channel[i].xv[0] = box_channel[i].xv[1]; box_channel[i].xv[1] = box_channel[i].xv[2]; box_channel[i].xv[2] = box_channel[i].xv[3];
        box_channel[i].yv[0] = box_channel[i].yv[1]; box_channel[i].yv[1] = box_channel[i].yv[2]; box_channel[i].yv[2] = box_channel[i].yv[3];
        box_channel[i].yv[3] = (box_channel[i].xv[0] + box_channel[i].xv[3]) + 3 * (box_channel[i].xv[1] + box_channel[i].xv[2]) + (0.9933538995 * box_channel[i].yv[0]) + (-2.9749051508 * box_channel[i].yv[1]) + (2.9815118124 * box_channel[i].yv[2]);
        box_channel[i].outputVal = box_channel[i].yv[3];
        outputValAve+=box_channel[i].outputVal;
    }
      box_channel[i].outputVal=outputValAve/FILTER_AVERAGER;
  }
}


void shoulderHAA(){ //joint1
  armPositions[0] = runningFilterPosition(previousPositionShoulder1, 2048 + round(2048/3.1416*((XBee.XBeeMeasurements[0].EMG.channel1 - shoulderEMGCalibration[0].intercept) / shoulderEMGCalibration[0].slope + (XBee.XBeeMeasurements[0].EMG.channel2 - shoulderEMGCalibration[1].intercept) / shoulderEMGCalibration[1].slope) / 2.0));
  if (armPositions[0] < 2048/3.1416*calibrationPosition[4]){
    armPositions[0] = 2048/3.1416*calibrationPosition[4];
  } else if (armPositions[0] > 2048/3.1416*calibrationPosition[0]){
    armPositions[0] = 2048/3.1416*calibrationPosition[0];
  }
}

void shoulderFE(){ //joint2
  armPositions[1] = 3072 + ((XBee.XBeeMeasurements[0].RPY.roll - accelerationCalibration[0].rollOffset) * POSITION_UNIT_FACTOR);
}

void elbow(){ //joint3
  armPositions[2] = 2048 - (((XBee.XBeeMeasurements[1].RPY.roll - accelerationCalibration[1].rollOffset) + (XBee.XBeeMeasurements[0].RPY.roll - accelerationCalibration[0].rollOffset)) * POSITION_UNIT_FACTOR);
}

void gripper(){
  if (box_channel[0].emgData < gripperEMGCalibration.gripperNoise && box_channel[1].emgData > gripperEMGCalibration.releaseNoise){
    CrustCrawler.release();
  }
  else{
    CrustCrawler.grasp(gripperEMGCalibration.graspMaxEMG * GRIPPER_EMG_TO_PWM_CONVERION * box_channel[0].emgData);
  }
}


void calibrationAccelerometerAndShoulderEMG(){
  Serial.println("\nPress YES to start the ACCELEROMETER and SHOULDER EMG calibration!");
  while (digitalRead(buttonYES) == LOW){}
  Serial.println("Calibration Started!");
  delay(1000);

  Serial.println("Fully ADDUCT the shoulder horizontally! (30DEG), press YES to continue!");
  while (digitalRead(buttonYES) == LOW){}
  Serial.println("Keep your arm steady!");
  XBee.runningAverageReset();
  shoulderEMGCalibration[0].points[0] = XBee.XBeeMeasurements[0].EMG.channel1;
  shoulderEMGCalibration[1].points[0] = XBee.XBeeMeasurements[0].EMG.channel2;
  accelerationCalibration[0].roll[0] = XBee.XBeeMeasurements[0].RPY.roll;
  accelerationCalibration[1].roll[0] = XBee.XBeeMeasurements[1].RPY.roll;
  Serial.println("Step finished!\n");
  delay(500);

  Serial.println("Move -30DEG (0DEG), press YES to continue!");
  while (digitalRead(buttonYES) == LOW){}
  Serial.println("Keep your arm steady!");
  XBee.runningAverageReset();
  shoulderEMGCalibration[0].points[1] = XBee.XBeeMeasurements[0].EMG.channel1;
  shoulderEMGCalibration[1].points[1] = XBee.XBeeMeasurements[0].EMG.channel2;
  accelerationCalibration[0].roll[1] = XBee.XBeeMeasurements[0].RPY.roll;
  accelerationCalibration[1].roll[1] = XBee.XBeeMeasurements[1].RPY.roll;
  Serial.println("Step finished!\n");
  delay(500);

  Serial.println("Move -30DEG (-30DEG), press YES to continue!");
  while (digitalRead(buttonYES) == LOW){}
  Serial.println("Keep your arm steady!");
  XBee.runningAverageReset();
  shoulderEMGCalibration[0].points[2] = XBee.XBeeMeasurements[0].EMG.channel1;
  shoulderEMGCalibration[1].points[2] = XBee.XBeeMeasurements[0].EMG.channel2;
  accelerationCalibration[0].roll[2] = XBee.XBeeMeasurements[0].RPY.roll;
  accelerationCalibration[1].roll[2] = XBee.XBeeMeasurements[1].RPY.roll;
  Serial.println("Step finished!\n");
  delay(500);

  Serial.println("Move -30DEG (-60DEG), press YES to continue!");
  while (digitalRead(buttonYES) == LOW){}
  Serial.println("Keep your arm steady!");
  XBee.runningAverageReset();
  shoulderEMGCalibration[0].points[3] = XBee.XBeeMeasurements[0].EMG.channel1;
  shoulderEMGCalibration[1].points[3] = XBee.XBeeMeasurements[0].EMG.channel2;
  accelerationCalibration[0].roll[3] = XBee.XBeeMeasurements[0].RPY.roll;
  accelerationCalibration[1].roll[3] = XBee.XBeeMeasurements[1].RPY.roll;
  Serial.println("Step finished!\n");
  delay(500);

  Serial.println("Move -30DEG (-90DEG), press YES to continue!");
  while (digitalRead(buttonYES) == LOW){}
  Serial.println("Keep your arm steady!");
  XBee.runningAverageReset();
  shoulderEMGCalibration[0].points[4] = XBee.XBeeMeasurements[0].EMG.channel1;
  shoulderEMGCalibration[1].points[4] = XBee.XBeeMeasurements[0].EMG.channel2;
  accelerationCalibration[0].roll[4] = XBee.XBeeMeasurements[0].RPY.roll;
  accelerationCalibration[1].roll[4] = XBee.XBeeMeasurements[1].RPY.roll;
  for (uint8_t i = 0; i < 2; i++){
    regression(i, shoulderEMGCalibration[i].points);
  }
  accelerationCalibration[0].rollOffset = (accelerationCalibration[0].roll[0] + accelerationCalibration[0].roll[1] + accelerationCalibration[0].roll[2] + accelerationCalibration[0].roll[3] + accelerationCalibration[0].roll[4]) / NUMBER_OF_SHOULDER_CALIBRATION_POINTS;
  accelerationCalibration[1].rollOffset = (accelerationCalibration[1].roll[0] + accelerationCalibration[1].roll[1] + accelerationCalibration[1].roll[2] + accelerationCalibration[1].roll[3] + accelerationCalibration[1].roll[4]) / NUMBER_OF_SHOULDER_CALIBRATION_POINTS;
  Serial.println("Calibration of ACCELEROMETER and SHOULDER EMG finished!\n");
}

void regression(uint8_t index, uint16_t emgDataPoints[NUMBER_OF_SHOULDER_CALIBRATION_POINTS]){
  uint16_t sumOfPoints = 0, sumOfPositionEMG = 0;
  for (uint8_t i = 0; i < NUMBER_OF_SHOULDER_CALIBRATION_POINTS; i++){
    sumOfPoints += emgDataPoints[i];
    sumOfPositionEMG += calibrationPosition[i] * emgDataPoints[i];
  }
  shoulderEMGCalibration[index].slope = (NUMBER_OF_SHOULDER_CALIBRATION_POINTS * sumOfPositionEMG - SUM_CALIBRATION_POSITION * sumOfPoints) / (NUMBER_OF_SHOULDER_CALIBRATION_POINTS * SUM_CALIBRATION_POSITION_SQUARED - SUM_CALIBRATION_POSITION * SUM_CALIBRATION_POSITION);
  if (shoulderEMGCalibration[index].slope > 0){
    shoulderEMGCalibration[index].intercept = emgDataPoints[0];
  }
  else{
    shoulderEMGCalibration[index].intercept = emgDataPoints[NUMBER_OF_SHOULDER_CALIBRATION_POINTS - 1];
  }
}

void calibrationGripperEMG(){
  Serial.println("Press any button to start the GRIPPER calibration!");
  while (digitalRead(buttonYES) == LOW){}
  Serial.println("Calibration Started!");
  delay(1000);

  Serial.println("Grasp object with reasonable amount of force! Press anything whenever ready.");
  while (digitalRead(buttonYES) == LOW){}
  Serial.println("Keep your arm steady!");
  uint8_t j = 0;
  while (j < NUMBER_OF_INIT_XBEE_SAMPLES){
    if (XBee.read()){
      box_channel[0].emgData = XBee.XBeeMeasurements[1].EMG.channel1;
      box_channel[1].emgData = XBee.XBeeMeasurements[1].EMG.channel2;
      for (size_t i = 0; i < 17; i++){
         filterloop();
      }
      j++;
    }
  }
  gripperEMGCalibration.graspMaxEMG = 1024.0 / box_channel[0].emgData;
  gripperEMGCalibration.releaseNoise = box_channel[1].emgData;
  gripperEMGCalibration.gripperNoise = round(box_channel[0].emgData);
  Serial.println("Step finished!\n");
  delay(500);

  Serial.println("Release object with reasonable amount of force! Press anything whenever ready.");
  while (digitalRead(buttonYES) == LOW){}
  Serial.println("Keep your arm steady!");
  j = 0;
  while (j < NUMBER_OF_INIT_XBEE_SAMPLES){
    if (XBee.read()){
      box_channel[0].emgData = XBee.XBeeMeasurements[1].EMG.channel1;
      box_channel[1].emgData = XBee.XBeeMeasurements[1].EMG.channel2;
      for (size_t i = 0; i < 17; i++){
         filterloop();
      }
      j++;
    }
  }
  gripperEMGCalibration.releaseNoise = round(gripperEMGCalibration.releaseNoise * 0.6 + box_channel[1].emgData * 0.4);
  gripperEMGCalibration.gripperNoise = round((gripperEMGCalibration.gripperNoise * 0.3 + box_channel[0].emgData * 0.7)*1.25);
}

inline int32_t runningFilterPosition(int32_t &previousMeasurement, const int32_t newMeasurement){
  return previousMeasurement = round(SMOOTHING_FACTOR_SHOULDER_1 * newMeasurement + (1.0 - SMOOTHING_FACTOR_SHOULDER_1) * previousMeasurement);
}
