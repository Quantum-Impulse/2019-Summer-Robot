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
#include "AHRS.h"

#include "controller.hpp"
#include "DriveTrain.hpp"

#include "rev/CANSparkMax.h"

class Robot : public frc::TimedRobot {

/* Creates the controllers objects Driver and Operator
The Drivers ID is set to 0 while the Operator is set to 1*/
 FRC5572Controller* Driver = new FRC5572Controller (0); 
 FRC5572Controller* Operator = new FRC5572Controller(1);

/*Creates the navx-XMP object*/
 AHRS *ahrs = new AHRS(SPI::Port::kMXP);

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
TopL2 = 2,
TopR1 = 3,
TopR2 = 4,
BotL1 = 5,
BotL2 = 6,
BotR1 = 7,
BotR2 = 8;

/*The NeoSparks SpeedController instantiation with their CAN ID
 and the type of motor its wired with respectively*/
rev::CANSparkMax *m_leftTopMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
rev::CANSparkMax *m_rightTopMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
rev::CANSparkMax *m_leftBottomMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
rev::CANSparkMax *m_rightBottomMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

/*instantiation of the double Solenoids with 
their PCM, forwardChannel, reverseChannel  */
frc::DoubleSolenoid *TopL = new frc::DoubleSolenoid(PCM1, TopL1, TopL2);
frc::DoubleSolenoid *TopR = new frc::DoubleSolenoid(PCM1, TopR1, TopR2);
frc::DoubleSolenoid *BotL = new frc::DoubleSolenoid(PCM2, BotL1, BotL2);
frc::DoubleSolenoid *BotR = new frc::DoubleSolenoid(PCM2, BotR1, BotR2);

/*instantiation of the compressor with its CAN ID*/ 
Compressor *compressor = new Compressor(0);

/* declaration of varibles that will be used to assign to a 
controller botton method*/
 bool LB1, RB1, X1, Y1, A1, B1,  LB2, RB2, X2, Y2, A2, B2; 

 /* declaration of varibles that will be used to assign to a 
controller botton method*/
 double RT1, LT1, XL1, YL1, XR1, YR1,   RT2, LT2, XL2, YL2, XR2, YR2;

DriveTrain *driveTrain = new DriveTrain(m_leftTopMotor, m_rightTopMotor, m_leftBottomMotor, m_rightBottomMotor, Driver, ahrs, TopL, TopR, BotL, BotR);

 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  
};




