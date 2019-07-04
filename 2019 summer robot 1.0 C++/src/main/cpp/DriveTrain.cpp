#include <iostream>
#include <frc/Drive/DifferentialDrive.h>
#include "controller.hpp"
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include <frc/Drive/DifferentialDrive.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/SpeedControllerGroup.h>

#include "DriveTrain.hpp"

DriveTrain::DriveTrain(rev::CANSparkMax* TopleftMotor ,
          rev::CANSparkMax* TopRightMotor, 
          rev::CANSparkMax* BottomLeftMotor, 
          rev::CANSparkMax* BottomRightMotor, 
          FRC5572Controller* Driver,
          AHRS *ahrs) {
            this->LeftMotors = new frc::SpeedControllerGroup{ *TopleftMotor, *BottomLeftMotor};
            this->RightMotors = new frc::SpeedControllerGroup{ *TopRightMotor, *BottomRightMotor};
            this->Driver = Driver; 
            this-> MRobotDrive = new frc::MecanumDrive{ *TopleftMotor, *BottomLeftMotor, *TopRightMotor, *BottomRightMotor};           
            this->TRobotDrive = new frc::DifferentialDrive{ *LeftMotors, *RightMotors};

};

void DriveTrain::Drive(){
    

};
