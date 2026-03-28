// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

class SubIndexer : public frc2::SubsystemBase
{
public:
  SubIndexer();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void setVoltage(units::volt_t iOutput);
  rev::REVLibError Configure();
  frc2::CommandPtr getIndexCommand();

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::spark::SparkMax* mIndexerController;
  rev::spark::SparkBaseConfig* mSparkConfigIndexer;
};
