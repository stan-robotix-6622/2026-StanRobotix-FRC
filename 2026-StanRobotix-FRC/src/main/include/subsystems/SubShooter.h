// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/sendable/SendableBuilder.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

class SubShooter : public frc2::SubsystemBase
{
public:
  SubShooter();

  void setVelocity(units::turns_per_second_t iNextVelocity);
  void setVoltage(units::volt_t iVoltage);
  units::turns_per_second_t getVelocity();

  // The first index of the array is the result of the Leader's configuration and
  // the second is the result of the Follower's configuration
  void Configure();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void InitSendable(wpi::SendableBuilder &builder) override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::SimpleMotorFeedforward<units::turns>* mFeedforward;

  rev::spark::SparkMax* mLeaderShooterController;
  rev::spark::SparkMax* mFollowerShooterController;
  frc::PIDController* mPIDcontroller;
  rev::spark::SparkRelativeEncoder* mRelativeEncoder;
  rev::spark::SparkClosedLoopController* mClossedLoopController;
};