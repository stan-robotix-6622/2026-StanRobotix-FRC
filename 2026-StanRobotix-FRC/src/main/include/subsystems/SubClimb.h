// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <rev/SparkMaxConfig.h>

class SubClimb : public frc2::SubsystemBase {
 public:
  SubClimb();
  void SetSpeed(double iSpeed);
  void StopMotor();
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::spark::SparkMax * mSparkMax1;
  rev::spark::SparkMax * mSparkMax2;
  rev::spark::SparkMaxConfig * mSparkMaxConfig1;
  rev::spark::SparkMaxConfig * mSparkMaxConfig2;
};
