// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkSim.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/system/plant/DCMotor.h>

#include "Constants.h"


class subShooter : public frc2::SubsystemBase {
 public:
  subShooter();

  void setVelocity(units::turns_per_second currentVelocity, units::turns_per_second nextVelocity);
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  
  //frc::PWMSparkMax mShooterControllerSim{subShooterConstants::kCANid};
  frc::DCMotor SparkMaxGearbox = frc::DCMotor::NEO(1);
  rev::spark::SparkSim SparkMaxSim{mShooterController, &SparkMaxGearbox};
  frc::PIDController *mPIDcontroller;
  rev::spark::SparkMax * mShooterController;

  frc::SimpleMotorFeedforward<units::turns_per_second> m_feedforward{0_V, 3_V / 1_tps};

};