// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ArmFeedforward.h>
#include <rev/SparkMax.h>
#include "Constants.h"


class SubPivotIntake : public frc2::SubsystemBase {
 public:
  SubPivotIntake();

  void Stop();

  void KeepPosition();

  void SetVoltage(double iVoltage);

  double GetAngle();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::spark::SparkMax * mPivotMotor = nullptr;
  frc::ArmFeedforward * mFeedForward = nullptr;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
