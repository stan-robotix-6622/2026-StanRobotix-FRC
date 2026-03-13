// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/geometry/Rotation2d.h>

#include <array>
#include <units/time.h>
#include <units/angle.h>

#include "subsystems/SubDrivetrain.h"

using volts_per_second = units::unit_t<units::detail::unit_multiply<units::voltage::volts, units::inverse<units::time::seconds>>, double, units::linear_scale>;

// Made based on the drive commands found at
// https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/spark_swerve/src/main/java/frc/robot/commands/DriveCommands.java
class DriveCommands {
 public:
  static frc2::CommandPtr getFeedforwardCharacterizationCommand(SubDrivetrain* iDrivetrain);
  static frc2::CommandPtr getWheelRadiusCharacterizationCommand(SubDrivetrain* iDrivetrain);

  struct WheelRadiusCharacterizationState {
    std::array<units::radian_t, 4> positions;
    frc::Rotation2d lastAngle = 0.0_rad;
    units::radian_t gyroDelta = 0.0_rad;
  };
};
