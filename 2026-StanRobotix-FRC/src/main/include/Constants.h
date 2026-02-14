// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/angle.h"
#include "units/velocity.h"
#include "units/acceleration.h"
#include "units/angular_velocity.h"
#include "units/angular_acceleration.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

typedef units::unit_t<units::compound_unit<units::volts, units::inverse<units::turns_per_second_squared>>, double, units::linear_scale> kAunit; // V / turn / s^2
typedef units::unit_t<units::compound_unit<units::volts, units::inverse<units::turns_per_second>>, double, units::linear_scale> kVunit;         // V / turn / s

namespace OperatorConstants {
    inline constexpr int kDriverControllerPort = 0;
} // namespace OperatorConstants

namespace ShooterConstants {
    constexpr int kCANid = 11;

    constexpr units::volt_t kS = 0_V;
    constexpr kVunit kV = 4_V / 31.7_tps;
    constexpr kAunit kA = 0_V / 1_tr_per_s_sq;

    constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
    constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;

    constexpr units::turns_per_second_t kVitesseVoulue = 10_tps;

    namespace PIDConstants {
        constexpr double kP = 1; // T'is be a placeholder :)
        constexpr double kI = 0;
        constexpr double kD = 0;

        constexpr units::turns_per_second_t setpoint = 10_tps; // its NOT(it actually is) a placeholder :)
    }
}

namespace FeederConstants {
    constexpr int kCANid = 12;

    constexpr units::volt_t kDesiredVoltage = 2_V; // placeholder :)
}
