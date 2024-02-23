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
#include "AHRS.h"
#include <iostream>
#include <rev/CANSparkMax.h>

class Robot : public frc::TimedRobot {
 AHRS *ahrs;  
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

  Robot() { 
	try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
        	ahrs = new AHRS(frc::I2C::Port::kMXP);
        } 
		catch (std::exception& ex ) {
		 	std::cout << "error could not find AHRS!!" << "\n";
        }
		
  } 

  
  
 private:

	// static constexpr SPI::Port kGyroPort = 0;


	frc::Joystick joystick{1};

	frc::Joystick controller{2};
 
	// Left
	WPI_VictorSPX frontL {1};
	WPI_VictorSPX backL {0};


	// Right
	WPI_VictorSPX frontR {3};
	WPI_VictorSPX backR {2};

	frc::MecanumDrive mec_drive{frontL, backL, frontR, backR};

	WPI_VictorSPX climber {4};
	// WPI_VictorSPX bendTwo {11};
	// WPI_VictorSPX intake1 {12};
	// WPI_VictorSPX intake2 {13};

	//frc::Spark shooter{5};
	rev::CANSparkMax intake{5, rev::CANSparkMax::MotorType::kBrushless};

	WPI_VictorSPX arm{6};


	int _leftTrigger = controller.GetRawAxis(2);
	int _rightTrigger = controller.GetRawAxis(3);

	int _xButton = controller.GetRawButton(3);


	
	double maxSpeed = 0.9;
	double speed = 0.7;
	double autoSpeed = -0.5; 

	frc::SlewRateLimiter<units::scalar> filter{0.9 / 1_s};	


	
};
