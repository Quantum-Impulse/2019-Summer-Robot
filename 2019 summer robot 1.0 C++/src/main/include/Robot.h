/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>

#include "controller.hpp"

#include "rev/CANSparkMax.h"

class Robot : public frc::TimedRobot {

/* Creates the controllers objects Driver and Operator
The Drivers ID is set to 0 while the Operator is set to 1*/
 FRC5572Controller Driver { 0 }, Operator { 1 };

/* ID Numbers for Compressors, Speedcontrollers,
 DoubleSoleniods*/
static const int 
leftLeadDeviceID = 1, 
leftFollowDeviceID = 2,
rightLeadDeviceID = 3,
rightFollowDeviceID = 4,
PCM1 = 5,
PCM2 = 6,
TopL1 = 1,
TopL2 = 1,
TopR1 = 1,
TopR2 = 1,
BotL1 = 1,
BotL2 = 1,
BotR1 = 1,
BotR2 = 1;

/*The NeoSparks SpeedController instantiation with their CAN ID
 and the type of motor its wired with respectively*/
rev::CANSparkMax m_leftTopMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightTopMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_leftBottomMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightBottomMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

/*instantiation of the double Solenoids with 
their PCM, forwardChannel, reverseChannel  */
frc::DoubleSolenoid TopL{PCM1, TopL1, TopL2};
frc::DoubleSolenoid TopR{PCM1, TopR1, TopR2};
frc::DoubleSolenoid BotL{PCM1, BotL1, BotL2};
frc::DoubleSolenoid BotL{PCM2, BotL1, BotL2};

/*instantiation of the compressor with its CAN ID*/ 
frc::Compressor *compresser = new frc::Compressor(0);

/* declaration of varibles that will be used to assign to a 
controller botton method*/
 bool LB1, RB1, X1, Y1, A1, B1,   LB2, RB2, X2, Y2, A2, B2; 

 /* declaration of varibles that will be used to assign to a 
controller botton method*/
 double RT1, LT1, XL1, YL1, XR1, YR1,   RT2, LT2, XL2, YL2, XR2, YR2;

 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  
  /* This is the refresh rate on the robot code. 
  It was changed from .02 to .05 for buffer issues.
  (to be able to process bigger programs) 
  
  to avoid hogging CPU cycles      */
  static constexpr double kDefaultPeriod = 0.05;

  /* use the varible kDefaultPeriod  */ 
  //TimedRobot(kDefaultPeriod);

};




