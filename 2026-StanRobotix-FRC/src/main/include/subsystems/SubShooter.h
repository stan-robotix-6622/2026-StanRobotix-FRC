// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <rev/SparkRelativeEncoder.h>

#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "Constants.h"

#include <rev/SparkBase.h>


class subShooter : public frc2::SubsystemBase {
 public:
  subShooter();

  void setVelocity(units::turns_per_second_t nextVelocity);
  void setVoltage(units::volt_t iVoltage);
  units::turns_per_second_t getVelocity();
  rev::REVLibError Configure();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::SimpleMotorFeedforward<units::turns> m_feedforward{0_V, 4_V / 31.7_tps};

  rev::spark::SparkMax * mShooterController;
  frc::PIDController * mPIDcontroller;
  rev::spark::SparkRelativeEncoder * mRelativeEncoder;
  rev::spark::SparkBaseConfig * mSparkConfig;

};