// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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
    constexpr int kMotorid = 0;
    constexpr double kSpeed = 0.8;
}

namespace XboxConstants {
    constexpr int kXboxPort = 0;
}

namespace PivotConstants{
  constexpr int kMotorPivotid1 = 0;
  constexpr int kMotorPivotid2 = 0;
  constexpr double kSpeedPivot = 0.8;
  constexpr double kOffset = 0; // en attendant
  constexpr double kP = 0.00001; // en attendant
  constexpr double kI = 0; // en attendant
  constexpr double kD = 0; // en attendant
  constexpr double setpointUp = 1; // en attendant
  constexpr double setpointDown = 1; // en attendant
}