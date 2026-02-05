// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkSim.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include "Constants.h"


class subShooter : public frc2::SubsystemBase {
 public:
  subShooter();

  void setVelocity(units::volt_t velocity);
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  
  frc::PWMSparkMax mShooterControllerSim{subShooterConstants::kCANid};
  frc::PIDController *mPIDcontroller;
  rev::spark::SparkMax * mShooterController;
};