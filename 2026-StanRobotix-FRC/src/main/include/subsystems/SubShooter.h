// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <wpi/sendable/SendableBuilder.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "ShooterLookupTable.h"

class SubShooter : public frc2::SubsystemBase
{
public:
  SubShooter();

  void setDesiredVelocity(units::turns_per_second_t iNextVelocity);
  void setVoltage(units::volt_t iVoltage);
  units::turns_per_second_t getVelocity();

  // The first index of the array is the result of the Leader's configuration and
  // the second is the result of the Follower's configuration
  std::array<rev::REVLibError, 2> Configure();

  void Periodic() override;

  void InitSendable(wpi::SendableBuilder &builder) override;

  bool atDesiredVelocity();

private:
  frc::SimpleMotorFeedforward<units::turns>* mFeedforward;

  rev::spark::SparkMax* mLeaderShooterController;
  rev::spark::SparkMax* mFollowerShooterController;
  frc::PIDController* mPIDcontroller;
  rev::spark::SparkRelativeEncoder* mRelativeEncoder;
  rev::spark::SparkClosedLoopController* mClossedLoopController;

  units::turns_per_second_t mDesiredVelocity;
};