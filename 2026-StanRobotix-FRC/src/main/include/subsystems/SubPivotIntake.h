// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ArmFeedforward.h>
#include <wpi/sendable/SendableBuilder.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

class SubPivotIntake : public frc2::SubsystemBase {
 public:
	SubPivotIntake();

	void Stop();

	void KeepPosition();

  void SetVoltage(units::volt_t iVoltage);

  void SetVelocity(units::radians_per_second_t iVelocity);

  units::radian_t GetAngle();

  void Periodic() override;

  void InitSendable(wpi::SendableBuilder &builder) override;

private:
  rev::spark::SparkMax* mPivotMotor;
  rev::spark::SparkRelativeEncoder* mEncoder;
  frc::ArmFeedforward* mFeedForward;
};
