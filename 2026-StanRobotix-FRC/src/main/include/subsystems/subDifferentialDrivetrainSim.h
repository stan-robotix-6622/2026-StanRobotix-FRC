// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>
#include <frc/RobotController.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <ctre/phoenix/motorcontrol/can/WPI_BaseMotorController.h>
#include <frc/AnalogGyro.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/drive/DifferentialDrive.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveKinematics.h>




class SubDifferentialDrivetrainSim : public frc2::SubsystemBase {
 public:
  SubDifferentialDrivetrainSim();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic();
  void Drive(double	xSpeed, double ySpeed);


 private:

 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::WPI_BaseMotorController m_leftMotor{0 , "T"};
  ctre::phoenix::motorcontrol::can::WPI_BaseMotorController m_rightMotor{1, "T"};

  frc::AnalogGyro m_gyro{1};
  frc::sim::AnalogGyroSim m_gyroSim{m_gyro};

  // These represent our regular encoder objects, which we would
  // create to use on a real robot. 
  frc::Encoder m_leftEncoder{0, 1};
  frc::Encoder m_rightEncoder{2, 3};
  // These are our EncoderSim objects, which we will only use in
  // simulation. However, you do not need to comment out these
  // declarations when you are deploying code to the roboRIO.
  frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
  frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
  
// Create the simulation model of our drivetrain.

  frc::DifferentialDriveOdometry m_odometry{
      m_gyro.GetRotation2d(), units::meter_t{m_leftEncoder.GetDistance()},
      units::meter_t{m_rightEncoder.GetDistance()}};
frc::Field2d m_field;
frc::sim::DifferentialDrivetrainSim m_driveSim{
  frc::DCMotor::CIM(2), // 2 CIM motors on each side of the drivetrain.
  7.29,               // 7.29:1 gearing reduction.
  7.5_kg_sq_m,        // MOI of 7.5 kg m^2 (from CAD model).
  60_kg,              // The mass of the robot is 60 kg.
  3_in,               // The robot uses 3" radius wheels.
  0.7112_m,     // The track width is 0.7112 meters.
  // The standard deviations for measurement noise:
  // x and y:          0.001 m
  // heading:          0.001 rad
  // l and r velocity: 0.1   m/s
  // l and r position: 0.005 m
  {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}};

  frc::DifferentialDrive mRobotDrive {m_leftMotor, m_rightMotor};
 

};
