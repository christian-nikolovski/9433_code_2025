// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <math.h>
#include "Robot.h"
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <thread>
#include "Drive.h"
#include "Auto.h"
#include "Arm.h"
#include <frc/filter/SlewRateLimiter.h>
#include <thread>
#include <chrono>
#include "cameraserver/CameraServer.h"
#include <frc/AnalogGyro.h>
#include <frc/TimedRobot.h>


#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
	std::cout << "-- LTBT Robot Program Start --" << std::endl;

	cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
	ahrs->Reset();
	camera.SetResolution(640, 480);
	cs::CvSink cvSink = frc::CameraServer::GetVideo();
	cs::CvSource outputStream = frc::CameraServer::PutVideo("Video", 640, 480);
	
	// frc::CameraServer::StartAutomaticCapture();

	// cs::CvSink cvSink = frc::CameraServer::GetVideo();

	// cs::CvSource outputStream = frc::CameraServer::PutVideo("Blur", 640, 480);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
	// Auto newAuto;

	//newAuto.TimedAutoArmBendTwo(2000, -0.4);

	//newAuto.TimedAutoIntake(2000, 0.5);

	//newAuto.ArmStop();

	//newAuto.TimedAutoMecDrive(10000, 0.0, -0.25, 0.0);
    using namespace std::this_thread;
	using namespace std::chrono;
	// set motors fo move backwards
	backL.Set(autoSpeed);
	backR.Set(autoSpeed);
	frontL.Set(autoSpeed);
	frontR.Set(autoSpeed);
	// sleep for 5 seconds
	sleep_for(milliseconds(5000));
	// set motors to stop
	backL.Set(0);
	backR.Set(0);
	frontL.Set(0);
	frontR.Set(0);
}

void Robot::AutonomousPeriodic() 
{
	// Auto newAuto;
	
	
	// if (latch == true) 
	// {	
	// 	newAuto.TimedAutoMecDrive(1000, 0, 0.1, 0, 0);

	// 	latch = false;

	// }

	// std::cout << "GetDisX:" << ahrs->GetDisplacementX() << "\n";
	// std::cout << "GetDisY:" << ahrs->GetDisplacementY() << "\n";
	// std::cout << "GetRads:" << ahrs->GetAngle() * (M_PI / 180) << "\n"; 
}

void Robot::TeleopInit() 
{
}

void Robot::TeleopPeriodic() 
{
	// Note for Teleop, DONT USE FOR LOOPS OR WHILE LOOPS! (unless it is really fast / no sleep_for() in the loop) 
	// it will stay at the loop until it is completed (essentially freezing your robot).
	// Teleop is already called every cycle, so use this to your advantage.

	using namespace frc;
	

	double joyXPower = controller2.GetRawAxis(1) * fabs(controller2.GetRawAxis(1)); 
	double joyYPower = controller2.GetRawAxis(0) * fabs(controller2.GetRawAxis(0)) * -1;
	// double joyXPower = controller2.GetRawAxis(0); 
	// double joyYPower = controller2.GetRawAxis(1);
	double joyZPower = controller2.GetRawAxis(4);

	// double joyYPower = controller2.GetRawAxis(1);
	// double joyZPower = controller2.GetRawAxis(4);
	// double joyXPower = controller2.GetRawAxis(0); 


	//std::cout << "GetAngle:" << ahrs->GetAngle() << "\n";
	// std::cout << "GetRads:" << ahrs->GetAngle() * (M_PI / 180) << "\n"; 
	// std::cout << "GetRadsSin:" << sin(ahrs->GetAngle() * (M_PI / 180)) << "\n"; 
	// std::cout << "GetRadsCos:" << cos(ahrs->GetAngle() * (M_PI / 180)) << "\n"; 
	// std::cout << "GetRadsTan:" << tan(ahrs->GetAngle() * (M_PI / 180)) << "\n"; 
	
	double YawRads = ahrs->GetAngle() * (M_PI / 180);

	// double fieldVelocityHeading;
	// if (joyXPower > 0) {
	// fieldVelocityHeading = std::atan(joyXPower / joyYPower);
	// }
	// else if (joyXPower < 0 && joyYPower > 0) {
	//   fieldVelocityHeading = std::atan(joyXPower / joyYPower) + M_PI;
	// }
	// else (
	//   fieldVelocityHeading = std::atan(joyXPower / joyYPower) - M_PI
	// );

	// double robotVelocityHeading = YawRads + fieldVelocityHeading;

	// double RobotSpeed = sqrt(pow(joyXPower, 2) + pow(joyYPower, 2));


	// double x_rotated = cos(robotVelocityHeading) * RobotSpeed;
	// double y_rotated = sin(robotVelocityHeading) * RobotSpeed;
	float temp = joyYPower * sin(YawRads) + joyXPower * cos(YawRads);
	float tempX = -joyYPower * cos(YawRads) + joyXPower * sin(YawRads);
	joyYPower = temp;
	joyXPower = tempX;
	// float sinSign = (sin(YawRads) > 0.01) ? 1 : -1;
	// std::cout << "sin:" << sin(YawRads) << "\n";
	// std::cout << "cos:" << cos(YawRads) << "\n";
	// std::cout << "tan:" << tan(YawRads) << "\n";
	float x_rotated = joyXPower;
	float y_rotated = joyYPower;

	double motors [4] = {0,0,0,0};

		// if going left, spin left wheels outer from eachother, spin right inner
		/// apply strafe
		motors[0] += (x_rotated);
		motors[1] += (-x_rotated);

		motors[2] += (-x_rotated);
		motors[3] += (x_rotated);

		/// apply forward and backwards
		// left
		motors[0] += (y_rotated);
		motors[1] += (y_rotated);

		// right
		motors[2] += (-y_rotated);
		motors[3] += (-y_rotated);

		/// apply rotation
		// left
		motors[0] -= (joyZPower);
		motors[1] -= (joyZPower);

		// right
		motors[2] -= (joyZPower);
		motors[3] -= (joyZPower);


	frontL.Set(motors[0]);
	backL.Set(motors[1]);

	backR.Set(motors[2]);
	frontR.Set(motors[3]);

	// Create new arm object
	double _leftJoy = -controller.GetRawAxis(1); 

    double leftJoy = filter.Calculate(_leftJoy); 

	double _rightJoy = controller.GetRawAxis(5);
	int _bButton = controller.GetRawButton(2);


    double leftPower = leftJoy;
    double rightPower = _rightJoy * fabs(_rightJoy);

	if (_rightJoy >= 0.1){
		arm.Set(_rightJoy*_rightJoy*0.1);
	}
	else if (_rightJoy <= -0.1){
		arm.Set(_rightJoy*_rightJoy*-0.4);
	}
	else {
		arm.Set(0);
	}

	

	int _yButton = controller.GetRawButton(4);
	int _aButton = controller.GetRawButton(1);

	if (_yButton)
	{
		climber.Set(0.4);
	}
	else if (_aButton)
	{
		climber.Set(-0.25);
	}
	else
	{
		climber.Set(0);
	}
    

    if (_leftJoy >= 0.1)
    {
        armSpeed = _leftJoy*_leftJoy*1;
		intake.Set(armSpeed);
     
    }
    else if (_leftJoy <= -0.1)
    {
        armSpeed = _leftJoy*_leftJoy*0.6;
		intake.Set(-armSpeed);
       
    }
    else
    {
        intake.Set(0);
     
    }

}


void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif
