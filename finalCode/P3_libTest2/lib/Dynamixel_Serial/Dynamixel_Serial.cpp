#include "Dynamixel_Serial.h"

// #############################################################################
// ################################### SETUP ###################################
// #############################################################################
void DynamixelClass::begin(HardwareSerial &HWserial, uint32_t baud){
  HWserial.begin(baud);
  DynamixelSerial = &HWserial;
  pinMode(DIRECTION_PIN, OUTPUT);
}

void DynamixelClass::begin(Stream &serial){
  DynamixelSerial = &serial;
  pinMode(DIRECTION_PIN, OUTPUT);
}


// #############################################################################
// ############################## PACKET HANDLING ##############################
// #############################################################################
void DynamixelClass::transmitInstructionPacket(const unsigned char ID, const unsigned char instructionSubPacket[], const uint8_t parameterLength){ // "instructionSubPacket[]" - Instruction, Parameter1...ParameterN; where "parameterLength" is N
  clearRX();
  digitalWrite(DIRECTION_PIN, HIGH); // Allow transmitting
  unsigned char packet[parameterLength + 10]; // Include 3 Header, 1 Reserved, 1 ID, 2 Packet Length, 1 Instruction, N Parameters, 2 CRC
  packet[0] = 0xFF; // Header 1
  packet[1] = 0xFF; // Header 2
  packet[2] = 0xFD; // Header 3
  packet[3] = 0x00; // Reserved
  packet[4] = ID; // ID
  packet[5] = (parameterLength + 3) & 0xFF; // LEN_L
  packet[6] = ((parameterLength + 3) >> 8) & 0xFF; // LEN_H
  for (uint8_t i = 0; i < parameterLength + 1; i++){ // +1 for Instruction
    packet[i + 7] = instructionSubPacket[i]; // Instruction and Parameters (starting from 7th)
  }
  uint16_t CRC = updateCRC(packet, parameterLength + 8); // Calculate CRC (lenght excludes 2 CRC)
  packet[parameterLength + 8] = CRC & 0xFF; // CRC_L
  packet[parameterLength + 9] = (CRC >> 8) & 0xFF; // CRC_H
  for(int i = 0; i < parameterLength+10; i++){
    DynamixelSerial->write(packet[i]); // Send Packet
  }
  DynamixelSerial->flush();
  digitalWrite(DIRECTION_PIN, LOW); // Allow receiving
}

DynamixelClass::statusPacketStruct DynamixelClass::readStatusPacket(){
  delayMicroseconds(STATUS_PACKET_DELAY);
  static unsigned long timeoutCounter = STATUS_PACKET_TIMEOUT + millis();
  while (DynamixelSerial->available() < 9){ // All bytes up until ERROR
    if (millis() > timeoutCounter){
      DynamixelClass::statusPacketStruct status;
      status.error = TIMEOUT_ERROR;
      Serial.print("\nTimeout Error!!!"); // Debug
      return status;
    }
  }
  while (DynamixelSerial->peek() != 0xFF){
    if (DynamixelSerial->available() < 9){
      DynamixelClass::statusPacketStruct status;
      status.error = UNKNOWN_ERROR;
      Serial.print("\nNot enough bytes in the RX buffer!!!"); // Debug
      return status;
    }
    DynamixelSerial->read();
  }
  if (DynamixelSerial->read() == 0xFF && DynamixelSerial->read() == 0xFF && DynamixelSerial->read() == 0xFD && DynamixelSerial->read() == 0x00){
    unsigned char statusSubPacket[5]; // ID, LEN_L, LEN_H, 0x55 (Instruction), Error
    for (uint8_t i = 0; i < 5; i++){
      statusSubPacket[i] = DynamixelSerial->read();
    }
    if (statusSubPacket[4] != 0){
      DynamixelClass::statusPacketStruct status;
      status.error = statusSubPacket[4];
      Serial.print("\nError recieved!!! - "); Serial.print(statusSubPacket[4], BIN); // Debug
      return status;
    }
    if (statusSubPacket[3] != 0x55){
      DynamixelClass::statusPacketStruct status;
      status.error = UNKNOWN_ERROR;
      Serial.print("\nInstruction in the received packet is not 0x55!!!"); // Debug
      return status;
    }
    uint16_t packetLength = statusSubPacket[1] | (statusSubPacket[2] << 8);
    timeoutCounter = STATUS_PACKET_TIMEOUT + millis();
    while (DynamixelSerial->available() < packetLength - 3){ // Waiting for the rest of the packet
      if (millis() > timeoutCounter){
        DynamixelClass::statusPacketStruct status;
        status.error = TIMEOUT_ERROR;
        Serial.print("\nTimeout Error!!!"); // Debug
        return status;
      }
    }
    unsigned char statusPacket[packetLength + 7]; // 7: everything before packetLength
    statusPacket[0] = 0xFF; // Header 1
    statusPacket[1] = 0xFF; // Header 2
    statusPacket[2] = 0xFD; // Header 3
    statusPacket[3] = 0x00; // Reserved
    for (uint8_t i = 0; i < 5; i++){
      statusPacket[i + 4] = statusSubPacket[i]; // Copy up until ERROR byte (including ERROR byte)
    }
    for (uint8_t i = 0; i < packetLength - 2; i++){
      statusPacket[i + 9] = DynamixelSerial->read(); // Copy the rest
    }
    if (updateCRC(statusPacket, packetLength + 5) != (statusPacket[packetLength + 5] | (statusPacket[packetLength + 6] << 8))){
      DynamixelClass::statusPacketStruct status;
      status.error = CRC_ERROR;
      Serial.print("\nCRC Error in the received packet!!!"); // Debug
      return status;
    }
    DynamixelClass::statusPacketStruct status;
    status.ID = statusSubPacket[0];
    status.error = NO_ERROR;
    for (uint8_t i = 0; i < packetLength - 4; i++){ // 4: Instruction, Error, 2 CRC
      status.parameter[i] = statusPacket[i + 9];
    }
    return status;
  }
}

uint16_t DynamixelClass::updateCRC(const unsigned char *dataBlock, const uint8_t dataBlockLength) //CRC-16 (IBM/ANSI)
{
  uint16_t CRC = 0;
  uint16_t i;
  uint16_t CRC_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };
  for (uint8_t j = 0; j < dataBlockLength; j++){
    i = ((unsigned short)(CRC >> 8) ^ dataBlock[j]) & 0xFF;
    CRC = (CRC << 8) ^ CRC_table[i];
  }
  return CRC;
}

inline void DynamixelClass::clearRX(){
  while (DynamixelSerial->read() != -1);
}


// #############################################################################
// ################################ INSTRUCTION ################################
// #############################################################################
DynamixelClass::statusPacketStruct DynamixelClass::pingInstruction(const unsigned char ID){
  unsigned char pingSubPacket[] = {PING_INSTRUCTION};
  transmitInstructionPacket(ID, pingSubPacket, 0);
  return readStatusPacket();
}

DynamixelClass::statusPacketStruct DynamixelClass::readInstruction(const unsigned char ID, const uint16_t address, const uint8_t dataLength){
  unsigned char readSubPacket[dataLength + 3]; // +3 for Instruction and Address
  readSubPacket[0] = READ_INSTRUCTION; // Instruction
  readSubPacket[1] = address & 0xFF; // ADDRESS_L
  readSubPacket[2] = (address >> 8) & 0xFF; // ADDRESS_H
  readSubPacket[3] = dataLength & 0xFF; // DATA_LENGTH_L
  readSubPacket[4] = (dataLength >> 8) & 0xFF; // DATA_LENGTH_H
  transmitInstructionPacket(ID, readSubPacket, 4);
  return readStatusPacket();
}

bool DynamixelClass::writeInstruction(const unsigned char ID, const uint16_t address, const uint8_t dataLength, const unsigned char data[]){
  unsigned char writeSubPacket[dataLength + 3]; // +3 for Instruction and Address
  writeSubPacket[0] = WRITE_INSTRUCTION; // Instruction
  writeSubPacket[1] = address & 0xFF; // ADDRESS_L
  writeSubPacket[2] = (address >> 8) & 0xFF; // ADDRESS_H
  for (uint8_t i = 0; i < dataLength; i++){
    writeSubPacket[i + 3] = data[i]; // Data
  }
  transmitInstructionPacket(ID, writeSubPacket, dataLength + 2);
  return !readStatusPacket().error;
}

bool DynamixelClass::writeEEPROM(const unsigned char ID, const uint16_t address, const uint8_t dataLength, const unsigned char data[]){
  DynamixelClass::setTorqueEnable(ID, 0);
  if (DynamixelClass::writeInstruction(ID, address, dataLength, data)){
    DynamixelClass::setTorqueEnable(ID, 1);
    return true;
  }
  else{
    DynamixelClass::setTorqueEnable(ID, 1);
    return false;
  }
}

bool DynamixelClass::regWriteInstruction(const unsigned char ID, const uint16_t address, const uint8_t dataLength, const unsigned char data[]){
  unsigned char regWriteSubPacket[dataLength + 3]; // +3 for Instruction and Address
  regWriteSubPacket[0] = REG_WRITE_INSTRUCTION; // Instruction
  regWriteSubPacket[1] = address & 0xFF; // ADDRESS_L
  regWriteSubPacket[2] = (address >> 8) & 0xFF; // ADDRESS_H
  for (uint8_t i = 0; i < dataLength; i++){
    regWriteSubPacket[i + 3] = data[i]; // Data
  }
  transmitInstructionPacket(ID, regWriteSubPacket, dataLength + 2);
  return !readStatusPacket().error;
}

bool DynamixelClass::actionInstruction(const unsigned char ID){
  unsigned char actionSubPacket[] = {ACTION_INSTRUCTION};
  transmitInstructionPacket(ID, actionSubPacket, 0);
  return !readStatusPacket().error;
}

// bool DynamixelClass::resetInstruction(const unsigned char ID, const unsigned char option){ // DO NOT USE!!!
//   // Options  - 0xFF : Reset all values
//   //          - 0x01 : Reset all values except ID
//   //          - 0x02 : Reset all values except ID and Baudrate
//   unsigned char resetPacket[] = {RESET_INSTRUCTION, option};
//   transmitInstructionPacket(ID, resetPacket, 1);
//   return !readStatusPacket().error;
// }

bool DynamixelClass::rebootInstruction(const unsigned char ID){
  unsigned char rebootSubPacket[] = {REBOOT_INSTRUCTION};
  transmitInstructionPacket(ID, rebootSubPacket, 0);
  return !readStatusPacket().error;
}

DynamixelClass::statusPacketStruct* DynamixelClass::syncReadInstruction(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address, const uint8_t dataLength){
  unsigned char syncReadSubPacket[numberOfIDs + 5]; // +5 for Instruction, Address and Data Length
  syncReadSubPacket[0] = SYNC_READ_INSTRUCTION; // Instruction
  syncReadSubPacket[1] = address & 0xFF; // ADDRESS_L
  syncReadSubPacket[2] = (address >> 8) & 0xFF; // ADDRESS_H
  syncReadSubPacket[3] = dataLength & 0xFF; // DATA_LENGTH_L
  syncReadSubPacket[4] = (dataLength >> 8) & 0xFF; // DATA_LENGTH_H
  for (uint8_t i = 0; i < numberOfIDs; i++){
    syncReadSubPacket[i + 5] = ID[i];
  }
  transmitInstructionPacket(BROADCAST_ID, syncReadSubPacket, numberOfIDs + 4);
  static statusPacketStruct returnPacket[NUMBER_OF_DYNAMIXEL_DEVICES];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    returnPacket[i] = readStatusPacket();
  }
  return returnPacket;
}

void DynamixelClass::syncWriteInstruction(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address, const uint8_t dataLength, const unsigned char data[]){ //data[] - for all servos in order as they are in ID[], dataLength - individual
  unsigned char syncWriteSubPacket[(numberOfIDs * (dataLength + 1)) + 5]; // +5 for Instruction, Address and Data Length; +1 for ID
  syncWriteSubPacket[0] = SYNC_WRITE_INSTRUCTION; // Instruction
  syncWriteSubPacket[1] = address & 0xFF; // ADDRESS_L
  syncWriteSubPacket[2] = (address >> 8) & 0xFF; // ADDRESS_H
  syncWriteSubPacket[3] = dataLength & 0xFF; // DATA_LENGTH_L
  syncWriteSubPacket[4] = (dataLength >> 8) & 0xFF; // DATA_LENGTH_H
  for (uint8_t i = 0; i < numberOfIDs; i++){
    syncWriteSubPacket[(i * (dataLength + 1)) + 5] = ID[i];
    for (uint8_t j = 0; j < dataLength; j++){
      syncWriteSubPacket[(i * (dataLength + 1)) + j + 6] = data[(i * dataLength) + j];
    }
  }
  transmitInstructionPacket(BROADCAST_ID, syncWriteSubPacket, (numberOfIDs * (dataLength + 1)) + 4);
}

void DynamixelClass::syncWriteEEPROM(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address, const uint8_t dataLength, const unsigned char data[]){ //data[] - for all servos in order as they are in ID[], dataLength - individual
  bool enable[numberOfIDs];
  for (int i = 0; i < numberOfIDs; i++){
    enable[i] = 0;
  }
  setTorqueEnableSync(ID, numberOfIDs, enable);
  syncWriteInstruction(ID, numberOfIDs, address, dataLength, data);
  for (int i = 0; i < numberOfIDs; i++){
    enable[i] = 1;
  }
  setTorqueEnableSync(ID, numberOfIDs, enable);
}

DynamixelClass::statusPacketStruct* DynamixelClass::bulkReadInstruction(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address[], const uint8_t dataLength[]){
  unsigned char bulkReadSubPacket[numberOfIDs * 5 + 1]; // +1 for ID
  bulkReadSubPacket[0] = BULK_READ_INSTRUCTION; // Instruction
  for (uint8_t i = 0; i < numberOfIDs; i++){
    bulkReadSubPacket[5 * i + 1] = ID[i];
    bulkReadSubPacket[5 * i + 2] = address[i] & 0xFF; // ADDRESS_L
    bulkReadSubPacket[5 * i + 3] = (address[i] >> 8) & 0xFF; // ADDRESS_H
    bulkReadSubPacket[5 * i + 4] = dataLength[i] & 0xFF; // DATA_LENGTH_L
    bulkReadSubPacket[5 * (i + 1)] = (dataLength[i] >> 8) & 0xFF; // DATA_LENGTH_H
  }
  transmitInstructionPacket(BROADCAST_ID, bulkReadSubPacket, numberOfIDs * 5);
  static statusPacketStruct returnPacket[NUMBER_OF_DYNAMIXEL_DEVICES];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    returnPacket[i] = readStatusPacket();
  }
  return returnPacket;
}

void DynamixelClass::bulkWriteInstruction(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address[], const uint8_t dataLength[], const unsigned char data[]){
  uint8_t dataLengths = 0;
  for (uint8_t i = 0; i < numberOfIDs; i++){
    dataLengths += dataLength[i];
  }
  unsigned char bulkWriteSubPacket[numberOfIDs * 5 + dataLengths + 1]; // 5 for Instruction, Address and Data Length; +1 for ID
  dataLengths = 0;
  bulkWriteSubPacket[0] = BULK_WRITE_INSTRUCTION; // Instruction
  for (uint8_t i = 0; i < numberOfIDs; i++){
    bulkWriteSubPacket[5 * i + dataLengths + 1] = ID[i];
    bulkWriteSubPacket[5 * i + dataLengths + 2] = address[i] & 0xFF; // ADDRESS_L
    bulkWriteSubPacket[5 * i + dataLengths + 3] = (address[i] >> 8) & 0xFF; // ADDRESS_H
    bulkWriteSubPacket[5 * i + dataLengths + 4] = dataLength[i] & 0xFF; // DATA_LENGTH_L
    bulkWriteSubPacket[5 * (i + 1) + dataLengths] = (dataLength[i] >> 8) & 0xFF; // DATA_LENGTH_H
    for (uint8_t j = 0; j < dataLength[i]; j++){
      bulkWriteSubPacket[5 * i + dataLengths + 6 + j] = data[dataLengths + j];
    }
    dataLengths += dataLength[i];
  }
  transmitInstructionPacket(BROADCAST_ID, bulkWriteSubPacket, numberOfIDs * 5 + dataLengths);
}


// #############################################################################
// ################################## GETTERS ##################################
// #############################################################################
uint8_t DynamixelClass::getBaudrate(const unsigned char ID){
  return DynamixelClass::readInstruction(ID, BAUDRATE_ADDRESS, BAUDRATE_SIZE).parameter[0];
}

uint8_t DynamixelClass::getOperatingMode(const unsigned char ID){
  return DynamixelClass::readInstruction(ID, OPERATING_MODE_ADDRESS, OPERATING_MODE_SIZE).parameter[0];
}

bool DynamixelClass::getTorqueEnable(const unsigned char ID){
  return DynamixelClass::readInstruction(ID, TORQUE_ENABLE_ADDRESS, TORQUE_ENABLE_SIZE).parameter[0];
}

bool DynamixelClass::getRegisteredInstruction(const unsigned char ID){
  return DynamixelClass::readInstruction(ID, REGISTERED_INSTRUCTION_ADDRESS, REGISTERED_INSTRUCTION_SIZE).parameter[0];
}

uint16_t DynamixelClass::getVelocityIGain(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, VELOCITY_I_GAIN_ADDRESS, VELOCITY_I_GAIN_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

uint16_t DynamixelClass::getVelocityPGain(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, VELOCITY_P_GAIN_ADDRESS, VELOCITY_P_GAIN_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

uint16_t DynamixelClass::getPositionDGain(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, POSITION_D_GAIN_ADDRESS, POSITION_D_GAIN_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

uint16_t DynamixelClass::getPositionIGain(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, POSITION_I_GAIN_ADDRESS, POSITION_I_GAIN_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

uint16_t DynamixelClass::getPositionPGain(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, POSITION_P_GAIN_ADDRESS, POSITION_P_GAIN_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

uint16_t DynamixelClass::getFeedforward2ndGain(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, FEEDFORWARD_2ND_GAIN_ADDRESS, FEEDFORWARD_2ND_GAIN_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

uint16_t DynamixelClass::getFeedforward1stGain(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, FEEDFORWARD_1ST_GAIN_ADDRESS, FEEDFORWARD_1ST_GAIN_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

int16_t DynamixelClass::getGoalPWM(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, GOAL_PWM_ADDRESS, GOAL_CURRENT_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

int16_t DynamixelClass::getGoalCurrent(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, GOAL_CURRENT_ADDRESS, GOAL_CURRENT_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

int32_t DynamixelClass::getGoalVelocity(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, GOAL_VELOCITY_ADDRESS, GOAL_POSITION_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8) | (status.parameter[2] << 16) | (status.parameter[3] << 24);
}

uint32_t DynamixelClass::getProfileAcceleration(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, PROFILE_ACCELERATION_ADDRESS, PROFILE_ACCELERATION_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8) | (status.parameter[2] << 16) | (status.parameter[3] << 24);
}

uint32_t DynamixelClass::getProfileVelocity(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, PROFILE_VELOCITY_ADDRESS, PROFILE_VELOCITY_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8) | (status.parameter[2] << 16) | (status.parameter[3] << 24);
}

int32_t DynamixelClass::getGoalPosition(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, GOAL_POSITION_ADDRESS, GOAL_POSITION_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8) | (status.parameter[2] << 16) | (status.parameter[3] << 24);
}

uint16_t DynamixelClass::getRealtimeTick(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, REALTIME_TICK_ADDRESS, REALTIME_TICK_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

bool DynamixelClass::getMoving(const unsigned char ID){
  return DynamixelClass::readInstruction(ID, MOVING_ADDRESS, MOVING_SIZE).parameter[0];
}

int16_t DynamixelClass::getPresentPWM(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, PRESENT_PWM_ADDRESS, PRESENT_PWM_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

int16_t DynamixelClass::getPresentCurrent(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, PRESENT_CURRENT_ADDRESS, PRESENT_CURRENT_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8);
}

int32_t DynamixelClass::getPresentVelocity(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, PRESENT_VELOCITY_ADDRESS, PRESENT_VELOCITY_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8) | (status.parameter[2] << 16) | (status.parameter[3] << 24);
}

int32_t DynamixelClass::getPresentPosition(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, PRESENT_POSITION_ADDRESS, PRESENT_POSITION_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8) | (status.parameter[2] << 16) | (status.parameter[3] << 24);
}

int32_t DynamixelClass::getVelocityTrajectory(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, VELOCITY_TRAJECTORY_ADDRESS, VELOCITY_TRAJECTORY_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8) | (status.parameter[2] << 16) | (status.parameter[3] << 24);
}

int32_t DynamixelClass::getPositionTrajectory(const unsigned char ID){
  DynamixelClass::statusPacketStruct status = DynamixelClass::readInstruction(ID, POSITION_TRAJECTORY_ADDRESS, POSITION_TRAJECTORY_SIZE);
  return status.parameter[0] | (status.parameter[1] << 8) | (status.parameter[2] << 16) | (status.parameter[3] << 24);
}


// #############################################################################
// ############################### SYNC  GETTERS ###############################
// #############################################################################
bool* DynamixelClass::getRegisteredInstructionSync(const unsigned char ID[], const uint8_t numberOfIDs){
  statusPacketStruct *returnPacket = syncReadInstruction(ID, numberOfIDs, REGISTERED_INSTRUCTION_ADDRESS, REGISTERED_INSTRUCTION_SIZE);
  static bool isInstructionRegistered[NUMBER_OF_DYNAMIXEL_DEVICES];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    for (uint8_t j = 0; j < numberOfIDs; j++){
      if (returnPacket[j].ID == ID[i] && returnPacket[j].error == 0){
        isInstructionRegistered[i] = returnPacket[j].parameter[0];
      }
    }
  }
  return isInstructionRegistered;
}

bool* DynamixelClass::getMovingSync(const unsigned char ID[], const uint8_t numberOfIDs){
  statusPacketStruct *returnPacket = syncReadInstruction(ID, numberOfIDs, MOVING_ADDRESS, MOVING_SIZE);
  static bool isMoving[NUMBER_OF_DYNAMIXEL_DEVICES];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    for (uint8_t j = 0; j < numberOfIDs; j++){
      if (returnPacket[j].ID == ID[i] && returnPacket[j].error == 0){
        isMoving[i] = returnPacket[j].parameter[0];
      }
    }
  }
  return isMoving;
}

int16_t* DynamixelClass::getPresentPWMSync(const unsigned char ID[], const uint8_t numberOfIDs){
  statusPacketStruct *returnPacket = syncReadInstruction(ID, numberOfIDs, PRESENT_PWM_ADDRESS, PRESENT_PWM_SIZE);
  static int16_t presentPWM[NUMBER_OF_DYNAMIXEL_DEVICES];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    for (uint8_t j = 0; j < numberOfIDs; j++){
      if (returnPacket[j].ID == ID[i] && returnPacket[j].error == 0){
        presentPWM[i] = returnPacket[j].parameter[0] | (returnPacket[j].parameter[1] << 8);
      }
    }
  }
  return presentPWM;
}

int16_t* DynamixelClass::getPresentCurrentSync(const unsigned char ID[], const uint8_t numberOfIDs){
  statusPacketStruct *returnPacket = syncReadInstruction(ID, numberOfIDs, PRESENT_CURRENT_ADDRESS, PRESENT_CURRENT_SIZE);
  static int16_t presentCurrent[NUMBER_OF_DYNAMIXEL_DEVICES];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    for (uint8_t j = 0; j < numberOfIDs; j++){
      if (returnPacket[j].ID == ID[i] && returnPacket[j].error == 0){
        presentCurrent[i] = returnPacket[j].parameter[0] | (returnPacket[j].parameter[1] << 8);
      }
    }
  }
  return presentCurrent;
}

int32_t* DynamixelClass::getPresentVelocitySync(const unsigned char ID[], const uint8_t numberOfIDs){
  statusPacketStruct *returnPacket = syncReadInstruction(ID, numberOfIDs, PRESENT_VELOCITY_ADDRESS, PRESENT_VELOCITY_SIZE);
  static int32_t presentVelocity[NUMBER_OF_DYNAMIXEL_DEVICES];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    for (uint8_t j = 0; j < numberOfIDs; j++){
      if (returnPacket[j].ID == ID[i] && returnPacket[j].error == 0){
        presentVelocity[i] = returnPacket[j].parameter[0] | (returnPacket[j].parameter[1] << 8) | (returnPacket[j].parameter[2] << 16) | (returnPacket[j].parameter[3] << 24);
      }
    }
  }
  return presentVelocity;
}

int32_t* DynamixelClass::getPresentPositionSync(const unsigned char ID[], const uint8_t numberOfIDs){
  statusPacketStruct *returnPacket = syncReadInstruction(ID, numberOfIDs, PRESENT_POSITION_ADDRESS, PRESENT_POSITION_SIZE);
  static int32_t presentPosition[NUMBER_OF_DYNAMIXEL_DEVICES];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    for (uint8_t j = 0; j < numberOfIDs; j++){
      if (returnPacket[j].ID == ID[i] && returnPacket[j].error == 0){
        presentPosition[i] = returnPacket[j].parameter[0] | (returnPacket[j].parameter[1] << 8) | (returnPacket[j].parameter[2] << 16) | (returnPacket[j].parameter[3] << 24);
      }
    }
  }
  return presentPosition;
}

int32_t* DynamixelClass::getVelocityTrajectorySync(const unsigned char ID[], const uint8_t numberOfIDs){
  statusPacketStruct *returnPacket = syncReadInstruction(ID, numberOfIDs, VELOCITY_TRAJECTORY_ADDRESS, VELOCITY_TRAJECTORY_SIZE);
  static int32_t velocityTrajectory[NUMBER_OF_DYNAMIXEL_DEVICES];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    for (uint8_t j = 0; j < numberOfIDs; j++){
      if (returnPacket[j].ID == ID[i] && returnPacket[j].error == 0){
        velocityTrajectory[i] = returnPacket[j].parameter[0] | (returnPacket[j].parameter[1] << 8) | (returnPacket[j].parameter[2] << 16) | (returnPacket[j].parameter[3] << 24);
      }
    }
  }
  return velocityTrajectory;
}

int32_t* DynamixelClass::getPositionTrajectorySync(const unsigned char ID[], const uint8_t numberOfIDs){
  statusPacketStruct *returnPacket = syncReadInstruction(ID, numberOfIDs, POSITION_TRAJECTORY_ADDRESS, POSITION_TRAJECTORY_SIZE);
  static int32_t positionTrajectory[NUMBER_OF_DYNAMIXEL_DEVICES];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    for (uint8_t j = 0; j < numberOfIDs; j++){
      if (returnPacket[j].ID == ID[i] && returnPacket[j].error == 0){
        positionTrajectory[i] = returnPacket[j].parameter[0] | (returnPacket[j].parameter[1] << 8) | (returnPacket[j].parameter[2] << 16) | (returnPacket[j].parameter[3] << 24);
      }
    }
  }
  return positionTrajectory;
}


// #############################################################################
// ############################### BULK  GETTERS ###############################
// #############################################################################
int32_t* DynamixelClass::bulkRead(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address[], const uint8_t dataLength[]){ // most generic and not efficient
  statusPacketStruct *returnPacket = bulkReadInstruction(ID, numberOfIDs, address, dataLength);
  static int32_t data[NUMBER_OF_DYNAMIXEL_DEVICES];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    for (uint8_t j = 0; j < numberOfIDs; j++){
      if (returnPacket[j].ID == ID[i] && returnPacket[j].error == 0){
        switch (dataLength[i]){
          case 1:
          data[i] = returnPacket[j].parameter[0];
          break;
          case 2:
          data[i] = returnPacket[j].parameter[0] | (returnPacket[j].parameter[1] << 8);
          break;
          case 4:
          data[i] = returnPacket[j].parameter[0] | (returnPacket[j].parameter[1] << 8) | (returnPacket[j].parameter[2] << 16) | (returnPacket[j].parameter[3] << 24);
          break;
        }
      }
    }
  }
  return data;
}

// #############################################################################
// ################################## SETTERS ##################################
// #############################################################################
void DynamixelClass::setDriveMode(const unsigned char ID, const uint8_t mode){
  // 0 Normal mode
  // 1 Reverse mode
  unsigned char data[] = {mode};
  writeEEPROM(ID, DRIVE_MODE_ADDRESS, DRIVE_MODE_SIZE, data);
}

void DynamixelClass::setOperatingMode(const unsigned char ID, const uint8_t mode){
  // 0 Current Control Mode
  // 1 Velocity Control Mode (0° ~ 360°)
  // 3 Position Control Mode
  // 4 Extended Position Control Mode (Multi-turn)
  // 5 Current-based Position Control Mode
  // 16 PWM Control Mode (Voltage Control Mode)
  unsigned char data[] = {mode};
  writeEEPROM(ID, OPERATING_MODE_ADDRESS, OPERATING_MODE_SIZE, data);
}

void DynamixelClass::setHomingOffset(const unsigned char ID, const int32_t offset){
  unsigned char data[] = {
    offset & 0xFF,
    (offset >> 8) & 0xFF,
    (offset >> 16) & 0xFF,
    (offset >> 24) & 0xFF};
  writeEEPROM(ID, HOMING_OFFSET_ADDRESS, HOMING_OFFSET_SIZE, data);
}

void DynamixelClass::setTorqueEnable(const unsigned char ID, const bool enable){
  unsigned char data[] = {enable};
  writeInstruction(ID, TORQUE_ENABLE_ADDRESS, TORQUE_ENABLE_SIZE, data);
}

void DynamixelClass::setVelocityIGain(const unsigned char ID, const uint16_t i){
  unsigned char data[] = {
    i & 0xFF,
    (i >> 8) & 0xFF};
  writeInstruction(ID, VELOCITY_I_GAIN_ADDRESS, VELOCITY_I_GAIN_SIZE, data);
}

void DynamixelClass::setVelocityPGain(const unsigned char ID, const uint16_t p){
  unsigned char data[] = {
    p & 0xFF,
    (p >> 8) & 0xFF};
  writeInstruction(ID, VELOCITY_P_GAIN_ADDRESS, VELOCITY_P_GAIN_SIZE, data);
}

void DynamixelClass::setPositionDGain(const unsigned char ID, const uint16_t d){
  unsigned char data[] = {
    d & 0xFF,
    (d >> 8) & 0xFF};
  writeInstruction(ID, POSITION_D_GAIN_ADDRESS, POSITION_D_GAIN_SIZE, data);
}

void DynamixelClass::setPositionIGain(const unsigned char ID, const uint16_t i){
  unsigned char data[] = {
    i & 0xFF,
    (i >> 8) & 0xFF};
  writeInstruction(ID, POSITION_I_GAIN_ADDRESS, POSITION_I_GAIN_SIZE, data);
}

void DynamixelClass::setPositionPGain(const unsigned char ID, const uint16_t p){
  unsigned char data[] = {
    p & 0xFF,
    (p >> 8) & 0xFF};
  writeInstruction(ID, POSITION_P_GAIN_ADDRESS, POSITION_P_GAIN_SIZE, data);
}

void DynamixelClass::setFeedforward2ndGain(const unsigned char ID, const uint16_t ff2){
  unsigned char data[] = {
    ff2 & 0xFF,
    (ff2 >> 8) & 0xFF};
  writeInstruction(ID, FEEDFORWARD_2ND_GAIN_ADDRESS, FEEDFORWARD_2ND_GAIN_SIZE, data);
}

void DynamixelClass::setFeedforward1stGain(const unsigned char ID, const uint16_t ff1){
  unsigned char data[] = {
    ff1 & 0xFF,
    (ff1 >> 8) & 0xFF};
  writeInstruction(ID, FEEDFORWARD_1ST_GAIN_ADDRESS, FEEDFORWARD_1ST_GAIN_SIZE, data);
}

void DynamixelClass::setGoalPWM(const unsigned char ID, const int16_t PWM){
  unsigned char data[] = {
    PWM & 0xFF,
    (PWM >> 8) & 0xFF};
  writeInstruction(ID, GOAL_PWM_ADDRESS, GOAL_PWM_SIZE, data);
}

void DynamixelClass::setGoalCurrent(const unsigned char ID, const int16_t current){
  unsigned char data[] = {
    current & 0xFF,
    (current >> 8) & 0xFF};
  writeInstruction(ID, GOAL_CURRENT_ADDRESS, GOAL_CURRENT_SIZE, data);
}

void DynamixelClass::setGoalVelocity(const unsigned char ID, const int32_t velocity){
  unsigned char data[] = {
    velocity & 0xFF,
    (velocity >> 8) & 0xFF,
    (velocity >> 16) & 0xFF,
    (velocity >> 24) & 0xFF};
  writeInstruction(ID, GOAL_VELOCITY_ADDRESS, GOAL_VELOCITY_SIZE, data);

}

void DynamixelClass::setProfileAcceleration(const unsigned char ID, const uint32_t profileAcceleration){
  unsigned char data[] = {
    profileAcceleration & 0xFF,
    (profileAcceleration >> 8) & 0xFF,
    (profileAcceleration >> 16) & 0xFF,
    (profileAcceleration >> 24) & 0xFF};
  writeInstruction(ID, PROFILE_ACCELERATION_ADDRESS, PROFILE_ACCELERATION_SIZE, data);
}

void DynamixelClass::setProfileVelocity(const unsigned char ID, const uint32_t profileVelocity){
  unsigned char data[] = {
    profileVelocity & 0xFF,
    (profileVelocity >> 8) & 0xFF,
    (profileVelocity >> 16) & 0xFF,
    (profileVelocity >> 24) & 0xFF};
  writeInstruction(ID, PROFILE_VELOCITY_ADDRESS, PROFILE_VELOCITY_SIZE, data);
}

void DynamixelClass::setGoalPosition(const unsigned char ID, const int32_t position){
  unsigned char data[] = {
    position & 0xFF,
    (position >> 8) & 0xFF,
    (position >> 16) & 0xFF,
    (position >> 24) & 0xFF};
  writeInstruction(ID, GOAL_POSITION_ADDRESS, GOAL_POSITION_SIZE, data);
}


// #############################################################################
// ############################### SYNC  SETTERS ###############################
// #############################################################################
void DynamixelClass::setReturnDelayTimeSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint8_t delay[]){
  unsigned char data[numberOfIDs];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[i] = delay[i];
  }
  syncWriteEEPROM(ID, numberOfIDs, RETURN_DELAY_TIME_ADDRESS, RETURN_DELAY_TIME_SIZE, data);
}

void DynamixelClass::setOperatingModeSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint8_t mode[]){
  unsigned char data[numberOfIDs];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[i] = mode[i];
  }
  syncWriteEEPROM(ID, numberOfIDs, OPERATING_MODE_ADDRESS, OPERATING_MODE_SIZE, data);
}

void DynamixelClass::setHomingOffsetSync(const unsigned char ID[], const uint8_t numberOfIDs, const int32_t offset[]){
  unsigned char data[numberOfIDs * 4];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[4 * i] = offset[i] & 0xFF;
    data[4 * i + 1] = (offset[i] >> 8) & 0xFF;
    data[4 * i + 2] = (offset[i] >> 16) & 0xFF;
    data[4 * i + 3] = (offset[i] >> 24) & 0xFF;
  }
  syncWriteEEPROM(ID, numberOfIDs, HOMING_OFFSET_ADDRESS, HOMING_OFFSET_SIZE, data);
}

void DynamixelClass::setMovingThresholdSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t threshold[]){
  unsigned char data[numberOfIDs * 4];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[4 * i] = threshold[i] & 0xFF;
    data[4 * i + 1] = (threshold[i] >> 8) & 0xFF;
    data[4 * i + 2] = (threshold[i] >> 16) & 0xFF;
    data[4 * i + 3] = (threshold[i] >> 24) & 0xFF;
  }
  syncWriteEEPROM(ID, numberOfIDs, MOVING_THRESHOLD_ADDRESS, MOVING_THRESHOLD_SIZE, data);
}

void DynamixelClass::setPWMLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t limit[]){
  unsigned char data[numberOfIDs * 2];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[2 * i] = limit[i] & 0xFF;
    data[2 * i + 1] = (limit[i] >> 8) & 0xFF;
  }
  syncWriteEEPROM(ID, numberOfIDs, PWM_LIMIT_ADDRESS, PWM_LIMIT_SIZE, data);
}

void DynamixelClass::setCurrentLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t limit[]){
  unsigned char data[numberOfIDs * 2];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[2 * i] = limit[i] & 0xFF;
    data[2 * i + 1] = (limit[i] >> 8) & 0xFF;
  }
  syncWriteEEPROM(ID, numberOfIDs, CURRENT_LIMIT_ADDRESS, CURRENT_LIMIT_SIZE, data);
}

void DynamixelClass::setAccelerationLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t limit[]){
  unsigned char data[numberOfIDs * 4];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[4 * i] = limit[i] & 0xFF;
    data[4 * i + 1] = (limit[i] >> 8) & 0xFF;
    data[4 * i + 2] = (limit[i] >> 16) & 0xFF;
    data[4 * i + 3] = (limit[i] >> 24) & 0xFF;
  }
  syncWriteEEPROM(ID, numberOfIDs, ACCELERATION_LIMIT_ADDRESS, ACCELERATION_LIMIT_SIZE, data);
}

void DynamixelClass::setVelocityLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t limit[]){
  unsigned char data[numberOfIDs * 4];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[4 * i] = limit[i] & 0xFF;
    data[4 * i + 1] = (limit[i] >> 8) & 0xFF;
    data[4 * i + 2] = (limit[i] >> 16) & 0xFF;
    data[4 * i + 3] = (limit[i] >> 24) & 0xFF;
  }
  syncWriteEEPROM(ID, numberOfIDs, VELOCITY_LIMIT_ADDRESS, VELOCITY_LIMIT_SIZE, data);
}

void DynamixelClass::setMaxPositionLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t limit[]){
  unsigned char data[numberOfIDs * 4];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[4 * i] = limit[i] & 0xFF;
    data[4 * i + 1] = (limit[i] >> 8) & 0xFF;
    data[4 * i + 2] = (limit[i] >> 16) & 0xFF;
    data[4 * i + 3] = (limit[i] >> 24) & 0xFF;
  }
  syncWriteEEPROM(ID, numberOfIDs, MAX_POSITION_LIMIT_ADDRESS, MAX_POSITION_LIMIT_SIZE, data);
}

void DynamixelClass::setMinPositionLimitSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t limit[]){
  unsigned char data[numberOfIDs * 4];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[4 * i] = limit[i] & 0xFF;
    data[4 * i + 1] = (limit[i] >> 8) & 0xFF;
    data[4 * i + 2] = (limit[i] >> 16) & 0xFF;
    data[4 * i + 3] = (limit[i] >> 24) & 0xFF;
  }
  syncWriteEEPROM(ID, numberOfIDs, MIN_POSITION_LIMIT_ADDRESS, MIN_POSITION_LIMIT_SIZE, data);
}

void DynamixelClass::setTorqueEnableSync(const unsigned char ID[], const uint8_t numberOfIDs, const bool enable[]){
  unsigned char data[numberOfIDs];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[i] = enable[i];
  }
  syncWriteInstruction(ID, numberOfIDs, TORQUE_ENABLE_ADDRESS, TORQUE_ENABLE_SIZE, data);
}

void DynamixelClass::setVelocityIGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t I[]){
  unsigned char data[numberOfIDs * 2];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[2 * i] = I[i] & 0xFF;
    data[2 * i + 1] = (I[i] >> 8) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, VELOCITY_I_GAIN_ADDRESS, VELOCITY_I_GAIN_SIZE, data);
}

void DynamixelClass::setVelocityPGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t P[]){
  unsigned char data[numberOfIDs * 2];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[2 * i] = P[i] & 0xFF;
    data[2 * i + 1] = (P[i] >> 8) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, VELOCITY_P_GAIN_ADDRESS, VELOCITY_P_GAIN_SIZE, data);
}

void DynamixelClass::setPositionDGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t D[]){
  unsigned char data[numberOfIDs * 2];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[2 * i] = D[i] & 0xFF;
    data[2 * i + 1] = (D[i] >> 8) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, POSITION_D_GAIN_ADDRESS, POSITION_D_GAIN_SIZE, data);
}

void DynamixelClass::setPositionIGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t I[]){
  unsigned char data[numberOfIDs * 2];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[2 * i] = I[i] & 0xFF;
    data[2 * i + 1] = (I[i] >> 8) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, POSITION_I_GAIN_ADDRESS, POSITION_I_GAIN_SIZE, data);
}

void DynamixelClass::setPositionPGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t P[]){
  unsigned char data[numberOfIDs * 2];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[2 * i] = P[i] & 0xFF;
    data[2 * i + 1] = (P[i] >> 8) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, POSITION_P_GAIN_ADDRESS, POSITION_P_GAIN_SIZE, data);
}

void DynamixelClass::setFeedforward2ndGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t ff2[]){
  unsigned char data[numberOfIDs * 2];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[2 * i] = ff2[i] & 0xFF;
    data[2 * i + 1] = (ff2[i] >> 8) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, FEEDFORWARD_2ND_GAIN_ADDRESS, FEEDFORWARD_2ND_GAIN_SIZE, data);
}

void DynamixelClass::setFeedforward1stGainSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t ff1[]){
  unsigned char data[numberOfIDs * 2];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[2 * i] = ff1[i] & 0xFF;
    data[2 * i + 1] = (ff1[i] >> 8) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, FEEDFORWARD_1ST_GAIN_ADDRESS, FEEDFORWARD_1ST_GAIN_SIZE, data);
}

void DynamixelClass::setGoalPWMSync(const unsigned char ID[], const uint8_t numberOfIDs, const int16_t PWM[]){
  unsigned char data[numberOfIDs * 2];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[2 * i] = PWM[i] & 0xFF;
    data[2 * i + 1] = (PWM[i] >> 8) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, GOAL_PWM_ADDRESS, GOAL_PWM_SIZE, data);
}

void DynamixelClass::setGoalCurrentSync(const unsigned char ID[], const uint8_t numberOfIDs, const int16_t current[]){
  unsigned char data[numberOfIDs * 2];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[2 * i] = current[i] & 0xFF;
    data[2 * i + 1] = (current[i] >> 8) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, GOAL_CURRENT_ADDRESS, GOAL_CURRENT_SIZE, data);
}

void DynamixelClass::setGoalVelocitySync(const unsigned char ID[], const uint8_t numberOfIDs, const int32_t velocity[]){
  unsigned char data[numberOfIDs * 4];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[4 * i] = velocity[i] & 0xFF;
    data[4 * i + 1] = (velocity[i] >> 8) & 0xFF;
    data[4 * i + 2] = (velocity[i] >> 16) & 0xFF;
    data[4 * i + 3] = (velocity[i] >> 24) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, GOAL_VELOCITY_ADDRESS, GOAL_VELOCITY_SIZE, data);
}

void DynamixelClass::setProfileAccelerationSync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t profileAcceleration[]){
  unsigned char data[numberOfIDs * 4];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[4 * i] = profileAcceleration[i] & 0xFF;
    data[4 * i + 1] = (profileAcceleration[i] >> 8) & 0xFF;
    data[4 * i + 2] = (profileAcceleration[i] >> 16) & 0xFF;
    data[4 * i + 3] = (profileAcceleration[i] >> 24) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, PROFILE_ACCELERATION_ADDRESS, PROFILE_ACCELERATION_SIZE, data);
}

void DynamixelClass::setProfileVelocitySync(const unsigned char ID[], const uint8_t numberOfIDs, const uint32_t profileVelocity[]){
  unsigned char data[numberOfIDs * 4];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[4 * i] = profileVelocity[i] & 0xFF;
    data[4 * i + 1] = (profileVelocity[i] >> 8) & 0xFF;
    data[4 * i + 2] = (profileVelocity[i] >> 16) & 0xFF;
    data[4 * i + 3] = (profileVelocity[i] >> 24) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, PROFILE_VELOCITY_ADDRESS, PROFILE_VELOCITY_SIZE, data);
}

void DynamixelClass::setGoalPositionSync(const unsigned char ID[], const uint8_t numberOfIDs, const int32_t position[]){
  unsigned char data[numberOfIDs * 4];
  for (uint8_t i = 0; i < numberOfIDs; i++){
    data[4 * i] = position[i] & 0xFF;
    data[4 * i + 1] = (position[i] >> 8) & 0xFF;
    data[4 * i + 2] = (position[i] >> 16) & 0xFF;
    data[4 * i + 3] = (position[i] >> 24) & 0xFF;
  }
  syncWriteInstruction(ID, numberOfIDs, GOAL_POSITION_ADDRESS, GOAL_POSITION_SIZE, data);
}


// #############################################################################
// ############################### BULK  SETTERS ###############################
// #############################################################################
void DynamixelClass::bulkWrite(const unsigned char ID[], const uint8_t numberOfIDs, const uint16_t address[], const uint8_t dataLength[], int32_t value[]){
  uint8_t dataLengths = 0;
  for (uint8_t i = 0; i < numberOfIDs; i++){
    dataLengths += dataLength[i];
  }
  unsigned char data[dataLengths];
  dataLengths = 0;
  for (uint8_t i = 0; i < numberOfIDs; i++){
    switch (dataLength[i]){
      case 1:
      data[dataLengths] = value[i] & 0xFF;
      break;
      case 2:
      data[dataLengths] = value[i] & 0xFF;
      data[dataLengths + 1] = (value[i] >> 8) & 0xFF;
      break;
      case 4:
      data[dataLengths] = value[i] & 0xFF;
      data[dataLengths + 1] = (value[i] >> 8) & 0xFF;
      data[dataLengths + 2] = (value[i] >> 16) & 0xFF;
      data[dataLengths + 3] = (value[i] >> 24) & 0xFF;
      break;
    }
    dataLengths += dataLength[i];
  }
  bulkWriteInstruction(ID, numberOfIDs, address, dataLength, data);
}



DynamixelClass Dynamixel;
