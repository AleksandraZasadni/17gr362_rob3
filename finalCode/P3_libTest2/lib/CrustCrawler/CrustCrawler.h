#ifndef CRUSTCRAWLER_H
#define CRUSTCRAWLER_H

#include <Arduino.h>
#include "Dynamixel_Serial.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%//
// INITIAL DYNAMIXEL SETUP //
//%%%%%%%%%%%%%%%%%%%%%%%%%//
static const uint8_t OPERATING_MODES[] = {16, 16, 16, 16, 16};
static const bool TORQUE_ENABLED[]     = {1, 1, 1, 1, 1};
static const uint8_t RETURN_DELAY[]    = {125, 125, 125, 125, 125};

#define GRAVITY_ACCELERATION 9.82201
#define MX_64_TORQUE_TO_PWM 147.5 // 885/6
#define MX_106_TORQUE_TO_PWM 105.357143 // 885/8.4
#define MX_28_TORQUE_TO_PWM 354 // 885/2.5
static const uint16_t PWM_LIMITS[]                = {300, 350, 300, 885, 885};

static const float PROPORTIONAL_GAIN[]            = {2.5, 5.5, 4.0, 2.0, 2.0};
static const float DERIVATIVE_GAIN[]              = {0, 0.0075, 0.0065, 0, 0};
static const float INTEGRAL_GAIN[]                = {0, 0, 0, 0, 0};
static const float VISCOUS_FRICTION_COEFFICIENT[] = {85.0*0.05, 95.0*0.05, 92.5*0.05}; // value in PWM
static const float COULOMB_FRICTION_COEFFICIENT[] = {30.0*0.10, 102.5*0.10, 45.0*0.10}; // value in PWM

#define MOVING_VELOCITY_TRESHOLD 0.5
#define GRIPPER_RELEASE_POSITION 1536
static const uint32_t MIN_POSITION_LIMITS[] = {0, 885, 755, 770, 770};
static const uint32_t MAX_POSITION_LIMITS[] = {4095, 3330, 3330, 2090, 2090};


class CrustCrawlerClass{
public:
  CrustCrawlerClass(){};
  void begin();
  void setGoalPosition(const int32_t displacement[]);
  void correctTrajectory();
  void grasp(const float torque);
  void release();

private:
  void CrustCrawlerClass::DynamixelSetup();
  struct compensatorStruct{ // dynamic compensation
    float M;
    float V;
    float G;
    float F;
  } compensator[NUMBER_OF_DYNAMIXEL_DEVICES_ARM];
  double lastTime;
  int32_t targetDisplacement[NUMBER_OF_DYNAMIXEL_DEVICES], previousDisplacement[NUMBER_OF_DYNAMIXEL_DEVICES], previousDisplacementError[NUMBER_OF_DYNAMIXEL_DEVICES];
  int16_t gripperPWM;
  void calculateCompensator(const int32_t displacement[], const float velocity[]);
  inline float mapThetas(const int32_t x, const int32_t in_min, const int32_t in_max, const float out_min, const float out_max);
  inline int8_t sgn(const float value);
  inline double microsInSeconds();
};

extern CrustCrawlerClass CrustCrawler;

#endif
