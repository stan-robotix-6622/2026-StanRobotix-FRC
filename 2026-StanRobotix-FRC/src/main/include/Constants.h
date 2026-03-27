// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/SparkMax.h>

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

namespace ClimbConstants {
    constexpr rev::spark::SparkLowLevel::MotorType kMotorTypeLeader = rev::spark::SparkLowLevel::MotorType::kBrushless;
    constexpr rev::spark::SparkLowLevel::MotorType kMotorTypeFollower = rev::spark::SparkLowLevel::MotorType::kBrushless;
    constexpr rev::spark::SparkMaxConfig::IdleMode kIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;
    constexpr double kP = 0.008;
    constexpr double kI = 0;
    constexpr double kD = 0;
    constexpr double kLiftP = 0.008;
    constexpr double kLiftI = 0;
    constexpr double kLiftD = 0;
    constexpr double kSetpointUp = 0;
    constexpr double kSetpointDown = 16;
    constexpr double kLimitReverse = 0;
    constexpr double kLimitForward = 16;
    constexpr bool kInverted = false;
    constexpr bool kInverseFollowerMotor = false;
}

namespace XboxControllerConstants {
    constexpr int deviceIDXboxController = 0;
}

namespace CANid 
{
    constexpr int kMotorClimbLeaderID = 14;
    constexpr int kMotorClimbFollowerID = 15;
}
