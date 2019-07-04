#ifndef DRIVE_TRAIN_HPP
#define DRIVE_TRAIN_HPP

#include <iostream>
#include <frc/Drive/DifferentialDrive.h>
#include "controller.hpp"
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include <frc/Drive/DifferentialDrive.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/SpeedControllerGroup.h>


/**
 * @class	    DriveTrain
 * @description Makes the main.cpp(Robot.cpp) more readable 
 * @notes	    This class should be updated for other versions as needed.
 */
  class DriveTrain{
public:

/**
	 * @constructor DriveTrain
	 * @description Construct the DriveTrain class.
	 * @param 	  TopleftMotor     -- CanSparkMax Speed controller (TopleftMotor) 
	 * @param	    TopRightMotor    -- CanSparkMax Speed controller (TopRightMotor)
	 * @param	    BottomLeftMotor  -- CanSparkMax Speed controller (BottomLeftMotor)
	 * @param 	  BottomRightMotor -- CanSparkMax Speed controller (BottomRightMotor)
   * @param     MFdriver         -- Navx XMP (Connect through the ...)
*/
DriveTrain(rev::CANSparkMax* TopleftMotor ,
          rev::CANSparkMax* TopRightMotor, 
          rev::CANSparkMax* BottomLeftMotor, 
          rev::CANSparkMax* BottomRightMotor, 
          FRC5572Controller* Driver,
          AHRS *ahrs);


void Drive();



private:
  frc::DifferentialDrive* TRobotDrive;
  frc::MecanumDrive* MRobotDrive;
  frc::SpeedControllerGroup* LeftMotors;
  frc::SpeedControllerGroup* RightMotors;
  FRC5572Controller* Driver;
  
  bool Switch = false, Crabing = true;

  enum State{TankDrive, Mecanum };
  
  };  


#endif