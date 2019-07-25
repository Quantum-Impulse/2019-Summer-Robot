#include <iostream>
#include <frc/Drive/DifferentialDrive.h>
#include "controller.hpp"
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include <frc/Drive/DifferentialDrive.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/DoubleSolenoid.h>

#include "DriveTrain.hpp"

DriveTrain::DriveTrain(rev::CANSparkMax* TopleftMotor ,
          rev::CANSparkMax* TopRightMotor, 
          rev::CANSparkMax* BottomLeftMotor, 
          rev::CANSparkMax* BottomRightMotor, 
          FRC5572Controller* Driver,
          AHRS *ahrs,
          frc::DoubleSolenoid* TopL,
          frc::DoubleSolenoid* TopR,
          frc::DoubleSolenoid* BotL,
          frc::DoubleSolenoid* BotR){
            this->LeftMotors = new frc::SpeedControllerGroup{ *TopleftMotor, *BottomLeftMotor};
            this->RightMotors = new frc::SpeedControllerGroup{ *TopRightMotor, *BottomRightMotor};
            this->Driver = Driver;
            this->ahrs = ahrs; 
            this-> MRobotDrive = new frc::MecanumDrive{ *TopleftMotor, *BottomLeftMotor, *TopRightMotor, *BottomRightMotor};
            this->TopL = TopL;
            this->TopR = TopR;
            this->BotL = BotL;
            this->BotR = BotR;
}

DriveTrain::~DriveTrain(){
    delete LeftMotors, RightMotors, Driver, ahrs, 
        MRobotDrive, TopL, TopR, BotL, BotR;
}

void DriveTrain::Drive(){
    if(Driver->toggleA){
        PRetract();
        LeftMotors->Set(Driver->L().second);
        RightMotors->Set(Driver->R().second);
    }
    else{
        POut();
        MRobotDrive->DriveCartesian(Driver->L().second, Driver->L().first, Driver->R().first, ahrs->GetYaw());
    }
}

void DriveTrain::POut(){
    TopL->Set(frc::DoubleSolenoid::Value::kForward);
    TopR->Set(frc::DoubleSolenoid::Value::kForward);
    BotL->Set(frc::DoubleSolenoid::Value::kForward);
    BotR->Set(frc::DoubleSolenoid::Value::kForward);
}

void DriveTrain::PRetract(){
    TopL->Set(frc::DoubleSolenoid::Value::kReverse);
    TopR->Set(frc::DoubleSolenoid::Value::kReverse);
    BotL->Set(frc::DoubleSolenoid::Value::kReverse);
    BotR->Set(frc::DoubleSolenoid::Value::kReverse);
}

