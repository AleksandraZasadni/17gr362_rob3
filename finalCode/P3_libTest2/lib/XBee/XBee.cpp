#include "XBee.h"

void XBeeClass::begin(HardwareSerial &HWserial, uint32_t baud){
  HWserial.begin(baud);
  XBeeSerial = &HWserial;
  clearRX();
  runningAverageReset();
}

void XBeeClass::begin(Stream &serial){
  XBeeSerial = &serial;
  clearRX();
  runningAverageReset();
}

inline void XBeeClass::runningAverageReset(){
  uint8_t i = 0;
  while (i < NUMBER_OF_INIT_XBEE_SAMPLES){
    if (XBeeClass::read()){
      i++;
    }
  }
}

bool XBeeClass::read(){
  clearRX();
  if (XBeeSerial->available() < 24*NUMBER_OF_XBEE_DEVICES){
    return false;
  }
  for (uint8_t j = 0; j < NUMBER_OF_XBEE_DEVICES; j++){
    if (XBeeSerial->read() != 0x7E || XBeeSerial->read() != 0x00 || XBeeSerial->read() != 0x14){
      clearRXAll();
      return false;
    }
    int checksum = 0;
    for (uint8_t i = 0; i < 20; i++){
      checksum += packet[i] = XBeeSerial->read();
    }
    if (XBeeSerial->read() != (0xFF - (checksum & 0xFF))){
      Serial.print("\nXBEE - WRONG CHECKSUM!\n"); // Debug
      clearRXAll();
      return false;
    }
    for (uint8_t i = 0; i < NUMBER_OF_XBEE_DEVICES; i++){
      if (DEVICE_ID[i] == unsigned((packet[1] << 8) | packet[2])){
        if (((packet[10] << 8) | packet[11]) > MEASUREMENTS_UPPER_BOUNDARY ||
            ((packet[12] << 8) | packet[13]) > MEASUREMENTS_UPPER_BOUNDARY ||
            ((packet[14] << 8) | packet[15]) > MEASUREMENTS_UPPER_BOUNDARY ||
            ((packet[16] << 8) | packet[17]) > MEASUREMENTS_UPPER_BOUNDARY ||
            ((packet[18] << 8) | packet[19]) > MEASUREMENTS_UPPER_BOUNDARY){
          clearRXAll();
          return false;
        }
        isReceived[i] = true;
        XBeeMeasurements[i].ID[0] = packet[1];
        XBeeMeasurements[i].ID[1] = packet[2];
        XBeeMeasurements[i].accelerometer.x = convertToUnits(mapFloat(runningFilterAccelerometer(i, previousAccelerometerX, (packet[14] << 8) | packet[15]), FROM_LOW_X[i], FROM_HIGH_X[i], TO_LOW_X[i], TO_HIGH_X[i]));
        XBeeMeasurements[i].accelerometer.y = convertToUnits(mapFloat(runningFilterAccelerometer(i, previousAccelerometerY, (packet[12] << 8) | packet[13]), FROM_LOW_Y[i], FROM_HIGH_Y[i], TO_LOW_Y[i], TO_HIGH_Y[i]));
        XBeeMeasurements[i].accelerometer.z = convertToUnits(mapFloat(runningFilterAccelerometer(i, previousAccelerometerZ, (packet[10] << 8) | packet[11]), FROM_LOW_Z[i], FROM_HIGH_Z[i], TO_LOW_Z[i], TO_HIGH_Z[i]));
        XBeeMeasurements[i].RPY.roll = getRoll(XBeeMeasurements[i].accelerometer.y, XBeeMeasurements[i].accelerometer.z);
        XBeeMeasurements[i].RPY.pitch = getPitch(XBeeMeasurements[i].accelerometer.x, XBeeMeasurements[i].accelerometer.y, XBeeMeasurements[i].accelerometer.z);
        XBeeMeasurements[i].EMG.channel1 = (packet[16] << 8) | packet[17];
        XBeeMeasurements[i].EMG.channel2 = (packet[18] << 8) | packet[19];
      }
    }
    XBeeMeasurements[0].EMG.channel1 = runningFilterEMG(previousEMGch1, XBeeMeasurements[0].EMG.channel1);
    XBeeMeasurements[0].EMG.channel2 = runningFilterEMG(previousEMGch2, XBeeMeasurements[0].EMG.channel2);
  }
  bool allReceived = true, allNotReceived = true;
  for (uint8_t i = 0; i < NUMBER_OF_XBEE_DEVICES; i++){
      isReceived[i] ? allNotReceived = false : allReceived = false;
  }
  if (allReceived && (!allNotReceived)){
    for (uint8_t i = 0; i < NUMBER_OF_XBEE_DEVICES; i++){
      isReceived[i] = false;
    }
    return true;
  }
}

inline void XBeeClass::clearRX(){
  while (XBeeSerial->available() > 0 && XBeeSerial->peek() != 0x7E){
    XBeeSerial->read();
  }
}

inline void XBeeClass::clearRXAll(){
  while (XBeeSerial->read() != -1);
}

void XBeeClass::accelerometerCalibration(uint8_t deviceIndex, uint16_t numberOfSamples){ // ONE DEVICE AT THE TIME (all other devices switched off)!!!
  uint16_t currentX, currentY, currentZ;
  uint64_t maxX = 0, minX = 0, maxY = 0, minY = 0, maxZ = 0, minZ = 0;
  uint16_t i = 0;
  uint8_t sideCounter = 0;
  Serial.print("\n---------------------");
  Serial.print("\nCalibration procedure (works only for orthogonal accelerometer; one device at the time):");
  Serial.print("\n1) Make one of the accelerometer axes collinear with the direction of gravity");
  Serial.print("\n2) When ready and prompted, press any key");
  Serial.print("\n3) Wait until prompted again while keepin the device steady");
  Serial.print("\n4) Repeat for all 6 sides\n");
  while (maxX == 0 || minX == 0 || maxY == 0 || minY == 0 || maxZ == 0 || minZ == 0){
    Serial.print("\nPress anything when ready...");
    while (!Serial.available()){}
    Serial.print("\n3...");
    delay(500);
    Serial.print("\n2...");
    delay(500);
    Serial.print("\n1...");
    delay(500);
    i = 0;
    clearRX();
    while (i < numberOfSamples){
      if (XBee.read()){
        currentX = ((packet[14] << 8) | packet[15]);
        currentY = ((packet[12] << 8) | packet[13]);
        currentZ = ((packet[10] << 8) | packet[11]);

       if ((currentX > 512.0) && ((abs((currentX - 512.0))) > (abs((currentY - 512.0)))) && ((abs((currentX - 512.0))) > (abs((currentZ - 512.0))))){
          maxX += currentX;
        }
        else if ((currentX < 512.0) && ((abs((currentX - 512.0))) > (abs((currentY - 512.0)))) && ((abs((currentX - 512.0))) > (abs((currentZ - 512.0))))){
          minX += currentX;
        }
        else if ((currentY > 512.0) && ((abs((currentY - 512.0))) > (abs((currentX - 512.0)))) && ((abs((currentY - 512.0))) > (abs((currentZ - 512.0))))){
          maxY += currentY;
        }
        else if ((currentY < 512.0) && ((abs((currentY - 512.0))) > (abs((currentX - 512.0)))) && ((abs((currentY - 512.0))) > (abs((currentZ - 512.0))))){
          minY += currentY;
        }
        else if ((currentZ > 512.0) && ((abs((currentZ - 512.0))) > (abs((currentX - 512.0)))) && ((abs((currentZ - 512.0))) > (abs((currentY - 512.0))))){
          maxZ += currentZ;
        }
        else if ((currentZ < 512.0) && ((abs((currentZ - 512.0))) > (abs((currentX - 512.0)))) && ((abs((currentZ - 512.0))) > (abs((currentY - 512.0))))){
          minZ += currentZ;
        }
        else{
          Serial.print("\nSomething went wrong with the calibration! Restart procedure!"); //debug
        }
        if((i++ % (numberOfSamples / 100)) == 0){
          Serial.print("\n("); Serial.print(i); Serial.print("/"); Serial.print(numberOfSamples); Serial.print(")");
        }
      }
    }
    while (Serial.read() != -1);
    Serial.print("\nStep "); Serial.print(++sideCounter); Serial.print("/6 finished!");
    }
    Serial.print("\n\n--------");
    Serial.print("\nResults for device"); Serial.print(deviceIndex); Serial.print(": ");
    Serial.print("\nMinimum values (x, y, z): ");
    Serial.print("\t");
    Serial.print(minX / float(numberOfSamples), 6);
    Serial.print("\t");
    Serial.print(minY / float(numberOfSamples), 6);
    Serial.print("\t");
    Serial.print(minZ / float(numberOfSamples), 6);
    Serial.print("\nMaximum values (x, y, z): ");
    Serial.print("\t");
    Serial.print(maxX / float(numberOfSamples), 6);
    Serial.print("\t");
    Serial.print(maxY / float(numberOfSamples), 6);
    Serial.print("\t");
    Serial.print(maxZ / float(numberOfSamples), 6);
}

inline float XBeeClass::runningFilterAccelerometer(const uint8_t deviceIndex, float previousMeasurement[], const uint16_t newMeasurement){
  return previousMeasurement[deviceIndex] = SMOOTHING_FACTOR_ACCELEROMETER * newMeasurement + (1.0 - SMOOTHING_FACTOR_ACCELEROMETER) * previousMeasurement[deviceIndex];
}

inline uint16_t XBeeClass::runningFilterEMG(uint16_t &previousMeasurement, const uint16_t newMeasurement){
  return previousMeasurement = round(SMOOTHING_FACTOR_EMG * newMeasurement + (1.0 - SMOOTHING_FACTOR_EMG) * previousMeasurement);
}

inline float XBeeClass::mapFloat(const float x, const float in_min, const float in_max, const float out_min, const float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline float XBeeClass::convertToUnits(float accelerometerMeasurement){
  return (accelerometerMeasurement * 0.0029296875) - 1.5; // 0.0029296875 == 3/1024
}

inline float XBeeClass::getRoll(float accelerationY, float accelerationZ){
  return atan2(-accelerationY, accelerationZ); // RAD
  // return (atan2(-accelerationY, accelerationZ) * 180.0) / M_PI; // DEG
}

inline float XBeeClass::getPitch(float accelerationX, float accelerationY, float accelerationZ){
  return atan2(accelerationX, sqrt(accelerationY * accelerationY + accelerationZ * accelerationZ)); // RAD
  // return (atan2(accelerationX, sqrt(accelerationY * accelerationY + accelerationZ * accelerationZ)) * 180.0) / M_PI; // DEG
}

XBeeClass XBee;
