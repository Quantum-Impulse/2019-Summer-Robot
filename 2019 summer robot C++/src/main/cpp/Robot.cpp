/*----------------------------------------------------------------------------*/
/* FRC Team --> 5572                                                                            */
/* Enrique`s Translation of java code from 2019 FRC game season               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>
#include <CameraServer.h>
#include "WPILib.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

// USB Camera Libraries
#include <cscore_oo.h>

//Quick camera thread, Using cs(opencv) to make camera setting changeable 
static void VisionThread()
  {
      frc::CameraServer *cameraServer;
      cs::UsbCamera camera = cameraServer->GetInstance()->StartAutomaticCapture();
      camera.SetResolution(240, 120);
      camera.SetFPS(15);
      cs::CvSink cvSink = cameraServer->GetInstance()->GetVideo();
      cs::CvSource outputStreamStd = cameraServer->GetInstance()->PutVideo("Gray", 240, 120);
      cv::Mat source;
      cv::Mat output;
      while(true) {
          cvSink.GrabFrame(source);
          cvtColor(source, output, cv::COLOR_BGR2GRAY);
          outputStreamStd.PutFrame(output);
      }
  }

void Robot::RobotInit() {
  compresser->Start();
  std::thread visionThread(VisionThread);
  visionThread.detach();
}

void Robot::RobotPeriodic() {
  /* This program utilizes try/catch statements throughout
  for less complicated error handling.*/
  
  try
  { /* Renaming of the buttons for easier reading of the code and computation, 
    all button usages are defined here */
    
 /* Driver Varible botton mapping   */
    LT1 = Driver.LT();
    LB1 = Driver.LB();
    RT1 = Driver.RT();
    RB1 = Driver.RB();
    X1  = Driver.X();
    Y1  = Driver.Y();
    A1  = Driver.A();
    XL1 = Driver.L().first;
    YL1 = Driver.L().second;
    XR1 = Driver.R().first;
    YR1 = Driver.R().second;

 /* Opartor Varible botton mappings */
    LT2 = Operator.LT();
    LB2 = Operator.LB();
    RT2 = Operator.RT();
    RB2 = Operator.RB();
    X2  = Operator.X();
    Y2  = Operator.Y();
    A2  = Operator.A();
    XL2 = Operator.L().first;
    YL2 = Operator.L().second;
    XR2 = Operator.R().first;
    YR2 = Operator.R().second;

  }catch(const std::exception& UpdateBottons){
    std::cerr << UpdateBottons.what() << " Update Botton Error" << '\n';
  }
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
