// Dynamixel Protocol 2.0
//   Instruction/Status packet
//     http://support.robotis.com/en/product/actuator/dynamixel_pro/communication/instruction_status_packet.htm
//   mx-106
//     http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-106(2.0).htm
//   mx-64
//     http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-64(2.0).htm
//   mx-28
//     http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-28(2.0).htm

#ifndef DYNAMIXEL_SERIAL_H
#define DYNAMIXEL_SERIAL_H

#include <Arduino.h>
#include "Dynamixel_Defines.h"

#define DIRECTION_PIN               2 // Pin to control the direction of serial communication

#define NUMBER_OF_DYNAMIXEL_DEVICES 5 // number of Dynamixel actuators connected in series
#define NUMBER_OF_DYNAMIXEL_DEVICES_ARM 3 // base, shoulder, elbow
#define BASE_ID                     0
#define SHOULDER_ID                 1
#define ELBOW_ID                    2
#define GRIPPER_LEFT_ID             3
#define GRIPPER_RIGHT_ID            4
static const unsigned char armIDs[] = {BASE_ID, SHOULDER_ID, ELBOW_ID};
static const unsigned char gripperIDs[] = {GRIPPER_LEFT_ID, GRIPPER_RIGHT_ID};
static const unsigned char allIDs[] = {BASE_ID, SHOULDER_ID, ELBOW_ID, GRIPPER_LEFT_ID, GRIPPER_RIGHT_ID};
#define STATUS_PACKET_TIMEOUT       5 //ms
#define STATUS_PACKET_DELAY       550 //us (micros)

class DynamixelClass{
public:
  DynamixelClass(){};

private:
  Stream *DynamixelSerial;
  struct statusPacketStruct{
    unsigned char ID;
    unsigned char error;
    unsigned char parameter[4]; // 4: maximum size for mx-28/mx-64/mx-106 status packet parameters
  };

// ################################### SETUP ###################################
public:
  void begin(HardwareSerial &HWserial, uint32_t baud = 57600);
  void begin(Stream &serial);

// ############################## PACKET HANDLING ##############################
private:
  void transmitInstructionPacket(const unsigned char ID, const unsigned char instructionSubPacket[], const uint8_t parameterLength);
  statusPacketStruct readStatusPacket();
  uint16_t updateCRC(const unsigned char *dataBlock, const uint8_t dataBlockLength);
  inline void clearRX();

// ################################ INSTRUCTION ################################
private:
  statusPacketStruct readInstruction(const unsigned char ID, const uint16_t address, const uint8_t dataLength);
  bool writeInstruction(const unsigned char ID, const uint16_t address, const uint8_t dataLength, const unsigned char data[]);
  bool writeEEPROM(const unsigned char ID, const uint16_t address, const uint8_t dataLength, const unsigned char data[]);
  bool regWriteInstruction(const unsigned char ID, const uint16_t address, const uint8_t dataLength, const unsigned char data[]);
  statusPacketStruct* syncReadInstruction(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address, const uint8_t dataLength);
  void syncWriteInstruction(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address, const uint8_t dataLength, const unsigned char data[]);
  void syncWriteEEPROM(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address, const uint8_t dataLength, const unsigned char data[]);
  statusPacketStruct* bulkReadInstruction(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address[], const uint8_t dataLength[]);
  void bulkWriteInstruction(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address[], const uint8_t dataLength[], const unsigned char data[]);
public:
  statusPacketStruct pingInstruction(const unsigned char ID);
  bool actionInstruction(const unsigned char ID);
  // bool resetInstruction(const unsigned char ID, const unsigned char option); // DO NOT USE!!!
  bool rebootInstruction(const unsigned char ID);

// ################################## GETTERS ##################################
public:
  uint8_t getBaudrate(const unsigned char ID);
  uint8_t getOperatingMode(const unsigned char ID);
  bool getTorqueEnable(const unsigned char ID);
  bool getRegisteredInstruction(const unsigned char ID);
  uint16_t getVelocityIGain(const unsigned char ID);
  uint16_t getVelocityPGain(const unsigned char ID);
  uint16_t getPositionDGain(const unsigned char ID);
  uint16_t getPositionIGain(const unsigned char ID);
  uint16_t getPositionPGain(const unsigned char ID);
  uint16_t getFeedforward2ndGain(const unsigned char ID);
  uint16_t getFeedforward1stGain(const unsigned char ID);
  int16_t getGoalPWM(const unsigned char ID);
  int16_t getGoalCurrent(const unsigned char ID);
  int32_t getGoalVelocity(const unsigned char ID);
  uint32_t getProfileAcceleration(const unsigned char ID);
  uint32_t getProfileVelocity(const unsigned char ID);
  int32_t getGoalPosition(const unsigned char ID);
  uint16_t getRealtimeTick(const unsigned char ID);
  bool getMoving(const unsigned char ID);
  int16_t getPresentPWM(const unsigned char ID);
  int16_t getPresentCurrent(const unsigned char ID);
  int32_t getPresentVelocity(const unsigned char ID);
  int32_t getPresentPosition(const unsigned char ID);
  int32_t getVelocityTrajectory(const unsigned char ID);
  int32_t getPositionTrajectory(const unsigned char ID);

// ############################### SYNC  GETTERS ###############################
public:
  bool* getRegisteredInstructionSync(const unsigned char ID[], const uint8_t numberOfIDs);
  bool* getMovingSync(const unsigned char ID[], const uint8_t numberOfIDs);
  int16_t* getPresentPWMSync(const unsigned char ID[], const uint8_t numberOfIDs);
  int16_t* getPresentCurrentSync(const unsigned char ID[], const uint8_t numberOfIDs);
  int32_t* getPresentVelocitySync(const unsigned char ID[], const uint8_t numberOfIDs);
  int32_t* getPresentPositionSync(const unsigned char ID[], const uint8_t numberOfIDs);
  int32_t* getVelocityTrajectorySync(const unsigned char ID[], const uint8_t numberOfIDs);
  int32_t* getPositionTrajectorySync(const unsigned char ID[], const uint8_t numberOfIDs);

// ############################### BULK  GETTERS ###############################
public:
int32_t* bulkRead(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address[], const uint8_t dataLength[]); // most generic and not efficient

// ################################## SETTERS ##################################
public:
  void setDriveMode(const unsigned char ID, const uint8_t mode);
  void setOperatingMode(const unsigned char ID, const uint8_t mode);
  void setHomingOffset(const unsigned char ID, const int32_t offset);
  void setTorqueEnable(const unsigned char ID, const bool set);
  void setVelocityIGain(const unsigned char ID, const uint16_t i);
  void setVelocityPGain(const unsigned char ID, const uint16_t p);
  void setPositionDGain(const unsigned char ID, const uint16_t d);
  void setPositionIGain(const unsigned char ID, const uint16_t i);
  void setPositionPGain(const unsigned char ID, const uint16_t p);
  void setFeedforward2ndGain(const unsigned char ID, const uint16_t ff2);
  void setFeedforward1stGain(const unsigned char ID, const uint16_t ff1);
  void setGoalPWM(const unsigned char ID, const int16_t PWM);
  void setGoalCurrent(const unsigned char ID, const int16_t current);
  void setGoalVelocity(const unsigned char ID, const int32_t velocity);
  void setProfileAcceleration(const unsigned char ID, const uint32_t profileAcceleration);
  void setProfileVelocity(const unsigned char ID, const uint32_t profileVelocity);
  void setGoalPosition(const unsigned char ID, const int32_t position);

// ############################### SYNC  SETTERS ###############################
public:
  void setReturnDelayTimeSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint8_t delay[]);
  void setOperatingModeSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint8_t mode[]);
  void setHomingOffsetSync(const unsigned char ID[], const uint8_t numberOfIDs, const int32_t offset[]);
  void setMovingThresholdSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t threshold[]);
  void setPWMLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t limit[]);
  void setCurrentLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t limit[]);
  void setAccelerationLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t limit[]);
  void setVelocityLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t limit[]);
  void setMaxPositionLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t limit[]);
  void setMinPositionLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t limit[]);
  void setTorqueEnableSync(const unsigned char ID[], const uint8_t numberOfIDs, const bool enable[]);
  void setVelocityIGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t I[]);
  void setVelocityPGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t P[]);
  void setPositionDGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t D[]);
  void setPositionIGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t I[]);
  void setPositionPGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t P[]);
  void setFeedforward2ndGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t ff2[]);
  void setFeedforward1stGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t ff1[]);
  void setGoalPWMSync(const unsigned char ID[], const uint8_t numberOfIDs, const int16_t PWM[]);
  void setGoalCurrentSync(const unsigned char ID[], const uint8_t numberOfIDs, const int16_t current[]);
  void setGoalVelocitySync(const unsigned char ID[], const uint8_t numberOfIDs, const int32_t velocity[]);
  void setProfileAccelerationSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t profileAcceleration[]);
  void setProfileVelocitySync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t profileVelocity[]);
  void setGoalPositionSync(const unsigned char ID[], const uint8_t numberOfIDs, const int32_t position[]);


// ############################### BULK  SETTERS ###############################
public:
  void bulkWrite(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address[], const uint8_t dataLength[], int32_t value[]);
};

extern DynamixelClass Dynamixel;

#endif
