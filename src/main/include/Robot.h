// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/motorcontrol/Spark.h>
#include <ctre/Phoenix.h>

#include <frc/Joystick.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/AnalogGyro.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/SPI.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  
 private:

	// static constexpr SPI::Port kGyroPort = 0;
	frc::AnalogGyro m_gyro{0};
	double kP = 0.005;

	frc::Joystick joystick{1};

	frc::Joystick controller{5};
 
	// Left
	WPI_VictorSPX frontL {2};
	WPI_VictorSPX backL {1};


	// Right
	WPI_VictorSPX frontR {4};
	WPI_VictorSPX backR {3};

	frc::MecanumDrive mec_drive{frontL, backL, frontR, backR};

	WPI_VictorSPX bendOne {10};
	WPI_VictorSPX bendTwo {11};
	WPI_VictorSPX intake1 {12};
	WPI_VictorSPX intake2 {13};


	int _leftTrigger = controller.GetRawAxis(2);
	int _rightTrigger = controller.GetRawAxis(3);

	int _xButton = controller.GetRawButton(3);
	int _yButton = controller.GetRawButton(4);


	
	double maxSpeed = 0.65;
	double speed = 0.3;
	double autoSpeed = -0.5; 

	frc::SlewRateLimiter<units::scalar> filter{0.9 / 1_s};	
	
};
