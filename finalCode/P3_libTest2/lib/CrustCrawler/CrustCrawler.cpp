#include "CrustCrawler.h"

void CrustCrawlerClass::DynamixelSetup(){
  Dynamixel.begin(Serial1, 1000000);
  Dynamixel.setReturnDelayTimeSync(allIDs, 5, RETURN_DELAY);
  Dynamixel.setDriveMode(3, 0);
  Dynamixel.setDriveMode(4, 1);
  Dynamixel.setOperatingModeSync(allIDs, 5, OPERATING_MODES);
  Dynamixel.setPWMLimitSync(allIDs, 5, PWM_LIMITS);
  Dynamixel.setTorqueEnableSync(allIDs, 5, TORQUE_ENABLED);
}

void CrustCrawlerClass::begin(){
  DynamixelSetup();
  targetDisplacement[3] = targetDisplacement[4] = GRIPPER_RELEASE_POSITION;
  lastTime = microsInSeconds();
  int32_t* temporaryPointer = Dynamixel.getPresentPositionSync(allIDs, NUMBER_OF_DYNAMIXEL_DEVICES); //TODO: find direct conversion from pointer to array located at different memory location
  for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXEL_DEVICES_ARM; i++){
    previousDisplacement[i] = temporaryPointer[i];
  }
}

void CrustCrawlerClass::setGoalPosition(const int32_t displacement[]){
  lastTime = microsInSeconds();
  int32_t* presentDisplacement = Dynamixel.getPresentPositionSync(armIDs, NUMBER_OF_DYNAMIXEL_DEVICES_ARM);
  for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXEL_DEVICES_ARM; i++){
    if (displacement[i] >= MIN_POSITION_LIMITS[i] && displacement[i] <= MAX_POSITION_LIMITS[i]){
      targetDisplacement[i] = displacement[i];
      previousDisplacementError[i] = targetDisplacement[i] - previousDisplacement[i];
    }
    previousDisplacement[i] = presentDisplacement[i];
  }
}

void CrustCrawlerClass::grasp(const float torque){
  float newPWM = MX_28_TORQUE_TO_PWM * torque;
  if (abs(gripperPWM - newPWM) > 4){
    gripperPWM = newPWM;
  }
}

void CrustCrawlerClass::release(){
  gripperPWM = 0;
  int32_t* presentDisplacement = Dynamixel.getPresentPositionSync(gripperIDs, 2);
  previousDisplacement[3] = presentDisplacement[0];
  previousDisplacementError[3] = targetDisplacement[3] - previousDisplacement[3];
  previousDisplacement[4] = presentDisplacement[1];
  previousDisplacementError[4] = targetDisplacement[4] - previousDisplacement[4];
}

void CrustCrawlerClass::correctTrajectory(){
  int32_t* presentDisplacement;
  int32_t presentDisplacementError[NUMBER_OF_DYNAMIXEL_DEVICES];
  float displacementIntegral[NUMBER_OF_DYNAMIXEL_DEVICES], displacementDerivative[NUMBER_OF_DYNAMIXEL_DEVICES], presentVelocity[NUMBER_OF_DYNAMIXEL_DEVICES_ARM];
  int16_t jointPWM[NUMBER_OF_DYNAMIXEL_DEVICES];
  double presentTime = microsInSeconds();

  if (gripperPWM == 0){
    presentDisplacement = Dynamixel.getPresentPositionSync(allIDs, NUMBER_OF_DYNAMIXEL_DEVICES);
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXEL_DEVICES_ARM; i++){
      presentVelocity[i] = (presentDisplacement[i] - previousDisplacement[i]) / (presentTime - lastTime); // * 0.0682666666666 for RPM
    }
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXEL_DEVICES; i++){
      presentDisplacementError[i] = targetDisplacement[i] - presentDisplacement[i];
      displacementIntegral[i] = ((presentDisplacementError[i] + previousDisplacementError[i]) / 2) * (presentTime - lastTime);
      displacementDerivative[i] = (presentDisplacementError[i] - previousDisplacementError[i]) / (presentTime - lastTime); // * 0.0682666666666 for RPM
      previousDisplacement[i] = presentDisplacement[i];
      previousDisplacementError[i] = presentDisplacementError[i];
    }
    jointPWM[3] = round(PROPORTIONAL_GAIN[3] * presentDisplacementError[3] + INTEGRAL_GAIN[3] * displacementIntegral[3] + DERIVATIVE_GAIN[3] * displacementDerivative[3]);
    jointPWM[4] = round(PROPORTIONAL_GAIN[4] * presentDisplacementError[4] + INTEGRAL_GAIN[4] * displacementIntegral[4] + DERIVATIVE_GAIN[4] * displacementDerivative[4]);
  }
  else
  if (gripperPWM > 0){
    presentDisplacement = Dynamixel.getPresentPositionSync(armIDs, NUMBER_OF_DYNAMIXEL_DEVICES_ARM);
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXEL_DEVICES_ARM; i++){
      presentVelocity[i] = (presentDisplacement[i] - previousDisplacement[i]) / (presentTime - lastTime); // * 0.0682666666666 for RPM
      presentDisplacementError[i] = targetDisplacement[i] - presentDisplacement[i];
      displacementIntegral[i] = ((presentDisplacementError[i] + previousDisplacementError[i]) / 2.0) * (presentTime - lastTime);
      displacementDerivative[i] = (presentDisplacementError[i] - previousDisplacementError[i]) / (presentTime - lastTime); // * 0.0682666666666 for RPM
      previousDisplacement[i] = presentDisplacement[i];
      previousDisplacementError[i] = presentDisplacementError[i];
    }
    jointPWM[3] = jointPWM[4] = gripperPWM;
  }
  lastTime = presentTime;

  calculateCompensator(presentDisplacement, presentVelocity);
  for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXEL_DEVICES_ARM; i++){
    // jointPWM[i] = round(compensator[i].G + compensator[i].F); //only compensator (excluding M)
    // jointPWM[i] = round(PROPORTIONAL_GAIN[i] * presentDisplacementError[i] + INTEGRAL_GAIN[i] * displacementIntegral[i] + DERIVATIVE_GAIN[i] * displacementDerivative[i]); //error Driven (no M)
    jointPWM[i] = round(PROPORTIONAL_GAIN[i] * presentDisplacementError[i] + INTEGRAL_GAIN[i] * displacementIntegral[i] + DERIVATIVE_GAIN[i] * displacementDerivative[i] + compensator[i].G + compensator[i].F); //all (no M)
    // jointPWM[i] = round(MASS_MATRIX_GAIN[i] * compensator[i].M * (PROPORTIONAL_GAIN[i] * presentDisplacementError[i] + INTEGRAL_GAIN[i] * displacementIntegral[i] + DERIVATIVE_GAIN[i] * displacementDerivative[i]) + compensator[i].G + compensator[i].F); //all
  }

  for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXEL_DEVICES; i++){
    if (jointPWM[i] > (signed)PWM_LIMITS[i]){
      jointPWM[i] = PWM_LIMITS[i];
    } else if (jointPWM[i] < -((signed)PWM_LIMITS[i])){
      jointPWM[i] = -((signed)PWM_LIMITS[i]);
    }
  }
  Dynamixel.setGoalPWMSync(allIDs, NUMBER_OF_DYNAMIXEL_DEVICES, jointPWM);
}

void CrustCrawlerClass::calculateCompensator(const int32_t displacement[], const float velocity[]){
  float theta[NUMBER_OF_DYNAMIXEL_DEVICES_ARM], theta_d[NUMBER_OF_DYNAMIXEL_DEVICES_ARM];
  for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXEL_DEVICES_ARM; i++){
    theta[i] = mapThetas(displacement[i], 0, 4095, -3.1415926535897, 3.1415926535897);
    theta_d[i] = mapThetas(velocity[i], -2048, 2048, -3.1415926535897, 3.1415926535897);
  }

  compensator[0].G = 0;
  compensator[1].G = MX_106_TORQUE_TO_PWM * (-0.0401888*GRAVITY_ACCELERATION*sin(theta[1] + theta[2])-0.10171*GRAVITY_ACCELERATION*sin(theta[1]));
  compensator[2].G = MX_64_TORQUE_TO_PWM * (-0.0401888*GRAVITY_ACCELERATION*sin(theta[1] + theta[2]));

  for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXEL_DEVICES_ARM; i++){
    compensator[i].F = sgn(theta_d[i])*(VISCOUS_FRICTION_COEFFICIENT[i] * abs(theta_d[i]) + COULOMB_FRICTION_COEFFICIENT[i]); //in PWM
  }

  // compensator[0].M = MX_64_TORQUE_TO_PWM * (0.0249399-0.0000286118*sin(2.0*theta[1])-0.00309537*cos(2.0*(theta[1] + theta[2]))-0.0215198*cos(theta[1])*cos(theta[1])-0.0000176105*sin(2.0*(theta[1] + theta[2]))+0.0000323888*(sin(theta[2])-sin(2.0*theta[1] + theta[2]))+0.00883349*(cos(theta[2])-cos(2.0*theta[1] + theta[2])));
  // compensator[1].M = MX_106_TORQUE_TO_PWM * (0.0278138+0.017667*cos(theta[2])+0.0000647775*sin(theta[2]));
  // compensator[2].M = MX_64_TORQUE_TO_PWM * 0.00626001;

  // compensator[0].V = MX_64_TORQUE_TO_PWM * ((theta_d[1]*theta_d[1])*(+4.29566e-7*cos(theta[1])-0.000519874*sin(theta[1]))+theta_d[1]*theta_d[2]*(0.00000181252*cos(theta[1]+theta[2])-0.000494322*sin(theta[1]+theta[2]))+theta_d[0]*theta_d[1]*((0.0215198*sin(2.0*theta[1])-0.0000572236*cos(2.0*theta[1]))+(0.017667*sin(2.0*theta[1]+theta[2])-0.0000647775*cos(2.0*theta[1]+theta[2])))+theta_d[0]*theta_d[2]*((0.0000323888*(cos(theta[2])-cos(2.0*theta[1]+theta[2])))+(0.00883349*(sin(2.0*theta[1]+theta[2])-sin(theta[2]))))+theta_d[0]*(theta_d[1]+theta_d[2])*(0.00619074*sin(2.0*(theta[1]+theta[2]))-0.0000352211*cos(2.0*(theta[1]+theta[2])))+((theta_d[1]*theta_d[1])+(theta_d[2]*theta_d[2]))*(9.0626e-7*cos(theta[1]+theta[2])-0.000247161*sin(theta[1]+theta[2])));
  // compensator[1].V = MX_106_TORQUE_TO_PWM * ((theta_d[2]*theta_d[2])*(0.0000323888*cos(theta[2])-0.00883349*sin(theta[2]))+theta_d[1]*theta_d[2]*(0.0000647775*cos(theta[2])-0.017667*sin(theta[2]))-(theta_d[0]*theta_d[0])*(0.00309537*sin(2.0*theta[1]+2.0*theta[2])+0.00883349*sin(2.0*theta[1]+theta[2])+0.0000286118*(2.0*sin(theta[1])*sin(theta[1])-1.0)+0.0000176105*(2.0*sin(theta[1]+theta[2])*sin(theta[1]+theta[2])-1.0)+0.0107599*sin(2.0*theta[1])+0.0000323888*(2.0*sin(theta[1]+0.5*theta[2])*sin(theta[1]+0.5*theta[2])-1.0)));
  // compensator[2].V = MX_64_TORQUE_TO_PWM * ((theta_d[1]*theta_d[1])*(0.00883349*sin(theta[2])-0.0000323888*cos(theta[2]))+(theta_d[0]*theta_d[0])*(0.0000161944*(cos(2.0*theta[1]+theta[2])-cos(theta[2]))+0.0000176105*cos(2.0*(theta[1]+theta[2]))+0.00441675*sin(theta[2])-0.00441675*sin(2.0*theta[1]+theta[2])-0.00309537*sin(2.0*(theta[1]+theta[2]))));
}

inline float CrustCrawlerClass::mapThetas(const int32_t x, const int32_t in_min, const int32_t in_max, const float out_min, const float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline int8_t CrustCrawlerClass::sgn(const float value){
  if (value < -MOVING_VELOCITY_TRESHOLD){
    return -1;
  }
  else if (value > MOVING_VELOCITY_TRESHOLD){
    return 1;
  }
  else{
    return 0;
  }
}

inline double CrustCrawlerClass::microsInSeconds(){
  return micros() / 1000000.0;
}


CrustCrawlerClass CrustCrawler;
