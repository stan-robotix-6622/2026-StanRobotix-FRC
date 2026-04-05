// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <wpi/sendable/SendableBuilder.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkMax.h>

class SubIntake : public frc2::SubsystemBase {
 public:
	SubIntake();
	void Stop();

  void SetVoltage(double);
  frc2::CommandPtr getIntakeCommand();
  void SetSpeed(double);
  bool isIntakeOn();

  void Periodic() override;

  void InitSendable(wpi::SendableBuilder &builder) override;

private:
  rev::spark::SparkMax* mIntakeMotor;

  rev::spark::SparkRelativeEncoder* mEncoder;
};
