// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

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

namespace subShooterConstants {

    constexpr units::volt_t kVoltage = 1_V; //its a placeholder

    constexpr int kCANid = 1;//its a placeholder too :)
}

namespace subIndexConstants {
    
    constexpr units::volt_t kVoltage = 2_V; //placeholder
    constexpr int kCANid = 69; //PLACEHOLDEERR
}