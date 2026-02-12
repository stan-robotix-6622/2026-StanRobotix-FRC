// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// #include "units/angular_acceleration.h"
// #include "units/angle.h"
// #include "units/acceleration.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

//  typedef units::compound_unit<units::volt_t, units::inverse<units::turns_per_second_squared>> kAunit; // V / turn / s^2
//  typedef units::compound_unit<units::volt_t, units::inverse<units::turns_per_second>> kVunit; // V / turn / s

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace subShooterConstants {

    constexpr units::volt_t kVoltage = 1_V; //its a placeholder
    // constexpr kVunit kV = 1_V / 1_tps; //placeholder

    constexpr int kCANid = 11;

    constexpr rev::ResetMode kReset = kResetSafeParameters;

    constexpr rev::PersistMode kPersist = kPersistParameters;
    constexpr double kVitesseVoulue = 10;

}

namespace SubFeederConstants {
    
    constexpr units::volt_t kVoltage = 2_V; //placeholder :)
    constexpr int kCANid = 12;
}

namespace PIDConstants {

    constexpr double kP = 0.00001; //T'is be a placeholder :)
    constexpr double kI = 0;
    constexpr double kD = 0;

    constexpr double setpoint = 1; //its NOT a placeholder :)
}