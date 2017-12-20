#ifndef DYNAMIXEL_DEFINES_H
#define DYNAMIXEL_DEFINES_H

#define BROADCAST_ID                      0xFE

//Instructions:
  #define PING_INSTRUCTION                  0x01
  #define READ_INSTRUCTION                  0x02
  #define WRITE_INSTRUCTION                 0x03
  #define REG_WRITE_INSTRUCTION             0x04
  #define ACTION_INSTRUCTION                0x05
  #define RESET_INSTRUCTION                 0x06
  #define REBOOT_INSTRUCTION                0x08
  #define STATUS_INSTRUCTION                0x55
  #define SYNC_READ_INSTRUCTION             0x82
  #define SYNC_WRITE_INSTRUCTION            0x83
  #define BULK_READ_INSTRUCTION             0x92
  #define BULK_WRITE_INSTRUCTION            0x93

//Errors:
  #define NO_ERROR                          B00000000
  #define ALERT                             B10000000 // problem in the device - can be B1XXXXXXX as a combination with another error
  #define RESULT_FAIL                       B00000001
  #define INSTRUCTION_ERROR                 B00000010
  #define CRC_ERROR                         B00000011
  #define DATA_RANGE_ERROR                  B00000100
  #define DATA_LENGTH_ERROR                 B00000101
  #define DATA_LIMIT_ERROR                  B00000110
  #define ACCESS_ERROR                      B00000111
  #define TIMEOUT_ERROR                     B00001000
  #define UNKNOWN_ERROR                     B00001001

//ADDRESS
  //EEPROM
    #define MODEL_NUMBER_ADDRESS            0x00
    #define MODEL_INFORMATION_ADDRESS       0x02
    #define VERSION_OF_FIRMWARE_ADDRESS     0x06
    #define ID_ADDRESS                      0x07
    #define BAUDRATE_ADDRESS                0x08
    #define RETURN_DELAY_TIME_ADDRESS       0x09
    #define DRIVE_MODE_ADDRESS              0x0A
    #define OPERATING_MODE_ADDRESS          0x0B
    #define SECONDARY_ID_ADDRESS            0x0C
    #define PROTOCOL_VERSION_ADDRESS        0x0D
    #define HOMING_OFFSET_ADDRESS           0x14
    #define MOVING_THRESHOLD_ADDRESS        0x18
    #define TEMPERATURE_LIMIT_ADDRESS       0x1F
    #define MAX_VOLTAGE_LIMIT_ADDRESS       0x20
    #define MIN_VOLTAGE_LIMIT_ADDRESS       0x22
    #define PWM_LIMIT_ADDRESS               0x24
    #define CURRENT_LIMIT_ADDRESS           0x26
    #define ACCELERATION_LIMIT_ADDRESS      0x28
    #define VELOCITY_LIMIT_ADDRESS          0x2C
    #define MAX_POSITION_LIMIT_ADDRESS      0x30
    #define MIN_POSITION_LIMIT_ADDRESS      0x34
    #define SHUTDOWN_ADDRESS                0x3F

  //RAM
    #define TORQUE_ENABLE_ADDRESS           0x40
    #define LED_ADDRESS                     0x41
    #define STATUS_RETURN_LEVEL_ADDRESS     0x44
    #define REGISTERED_INSTRUCTION_ADDRESS  0x45
    #define HARDWARE_ERROR_STATUS_ADDRESS   0x46
    #define VELOCITY_I_GAIN_ADDRESS         0x4C
    #define VELOCITY_P_GAIN_ADDRESS         0x4E
    #define POSITION_D_GAIN_ADDRESS         0x50
    #define POSITION_I_GAIN_ADDRESS         0x52
    #define POSITION_P_GAIN_ADDRESS         0x54
    #define FEEDFORWARD_2ND_GAIN_ADDRESS    0x58
    #define FEEDFORWARD_1ST_GAIN_ADDRESS    0x5A
    #define BUS_WATCHDOG_ADDRESS            0x62
    #define GOAL_PWM_ADDRESS                0x64
    #define GOAL_CURRENT_ADDRESS            0x66
    #define GOAL_VELOCITY_ADDRESS           0x68
    #define PROFILE_ACCELERATION_ADDRESS    0x6C
    #define PROFILE_VELOCITY_ADDRESS        0x70
    #define GOAL_POSITION_ADDRESS           0x74
    #define REALTIME_TICK_ADDRESS           0x78
    #define MOVING_ADDRESS                  0x7A
    #define MOVING_STATUS_ADDRESS           0x7B
    #define PRESENT_PWM_ADDRESS             0x7C
    #define PRESENT_CURRENT_ADDRESS         0x7E
    #define PRESENT_VELOCITY_ADDRESS        0x80
    #define PRESENT_POSITION_ADDRESS        0x84
    #define VELOCITY_TRAJECTORY_ADDRESS     0x88
    #define POSITION_TRAJECTORY_ADDRESS     0x8C
    #define PRESENT_INPUT_VOLTAGE_ADDRESS   0x90
    #define PRESENT_TEMPERATURE_ADDRESS     0x92

//SIZE (byte)
  //EEPROM
    #define MODEL_NUMBER_SIZE               2
    #define MODEL_INFORMATION_SIZE          4
    #define VERSION_OF_FIRMWARE_SIZE        1
    #define ID_SIZE                         1
    #define BAUDRATE_SIZE                   1
    #define RETURN_DELAY_TIME_SIZE          1
    #define DRIVE_MODE_SIZE                 1
    #define OPERATING_MODE_SIZE             1
    #define SECONDARY_ID_SIZE               1
    #define PROTOCOL_VERSION_SIZE           1
    #define HOMING_OFFSET_SIZE              4
    #define MOVING_THRESHOLD_SIZE           4
    #define TEMPERATURE_LIMIT_SIZE          1
    #define MAX_VOLTAGE_LIMIT_SIZE          2
    #define MIN_VOLTAGE_LIMIT_SIZE          2
    #define PWM_LIMIT_SIZE                  2
    #define CURRENT_LIMIT_SIZE              2
    #define ACCELERATION_LIMIT_SIZE         4
    #define VELOCITY_LIMIT_SIZE             4
    #define MAX_POSITION_LIMIT_SIZE         4
    #define MIN_POSITION_LIMIT_SIZE         4
    #define SHUTDOWN_SIZE                   1

  //RAM
    #define TORQUE_ENABLE_SIZE              1
    #define LED_SIZE                        1
    #define STATUS_RETURN_LEVEL_SIZE        1
    #define REGISTERED_INSTRUCTION_SIZE     1
    #define HARDWARE_ERROR_STATUS_SIZE      1
    #define VELOCITY_I_GAIN_SIZE            2
    #define VELOCITY_P_GAIN_SIZE            2
    #define POSITION_D_GAIN_SIZE            2
    #define POSITION_I_GAIN_SIZE            2
    #define POSITION_P_GAIN_SIZE            2
    #define FEEDFORWARD_2ND_GAIN_SIZE       2
    #define FEEDFORWARD_1ST_GAIN_SIZE       2
    #define BUS_WATCHDOG_SIZE               1
    #define GOAL_PWM_SIZE                   2
    #define GOAL_CURRENT_SIZE               2
    #define GOAL_VELOCITY_SIZE              4
    #define PROFILE_ACCELERATION_SIZE       4
    #define PROFILE_VELOCITY_SIZE           4
    #define GOAL_POSITION_SIZE              4
    #define REALTIME_TICK_SIZE              2
    #define MOVING_SIZE                     1
    #define MOVING_STATUS_SIZE              1
    #define PRESENT_PWM_SIZE                2
    #define PRESENT_CURRENT_SIZE            2
    #define PRESENT_VELOCITY_SIZE           4
    #define PRESENT_POSITION_SIZE           4
    #define VELOCITY_TRAJECTORY_SIZE        4
    #define POSITION_TRAJECTORY_SIZE        4
    #define PRESENT_INPUT_VOLTAGE_SIZE      2
    #define PRESENT_TEMPERATURE_SIZE        1

#endif
