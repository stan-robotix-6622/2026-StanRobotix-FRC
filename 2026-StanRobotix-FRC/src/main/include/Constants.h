// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/voltage.h>

#include <numbers>

#include <rev/SparkBase.h> // Include Spark variable types
#include <rev/config/SparkBaseConfig.h> // For the spark IdleMode

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace IntakeConstants {
  constexpr int kMotorid = 10;
  constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;
  constexpr bool kInverted = true;
  constexpr double kSpeed = 0.8; // a modifier (valeur temporaire)
}

namespace PivotConstants{
  constexpr int kMotorPivotid1 = 9;
  // constexpr int kMotorPivotid2 = 10;
  constexpr double kGearRatio = 16;
  constexpr double kOffset = -6.357143878936768;
  constexpr double kP = 1.0; // en attendant
  constexpr double kI = 0.0; // en attendant
  constexpr double kD = 0.0; // en attendant
  constexpr units::volt_t kG = -0.70_V;
  constexpr double setpointUp = -std::numbers::pi / 2;
  constexpr double setpointDown = 0;
  constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;
  constexpr bool kInverted = true;
}
