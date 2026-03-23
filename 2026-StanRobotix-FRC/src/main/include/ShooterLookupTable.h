#pragma once

#include <array>

#include <units/length.h>
#include <units/time.h>
#include <units/angular_velocity.h>

namespace Shooter {
  struct Status
  {
    units::meter_t distanceToTarget;
    units::turns_per_second_t shooterVelocity;
    units::second_t timeOfFlight;
  };

  constexpr std::array<Shooter::Status, 3> LookupTable = {
    Shooter::Status{1_m, 30_tps, 1.5_s},
    Shooter::Status{1.5_m, 50_tps, 2.5_s},
    Shooter::Status{2_m, 70_tps, 3.5_s}
  };
}