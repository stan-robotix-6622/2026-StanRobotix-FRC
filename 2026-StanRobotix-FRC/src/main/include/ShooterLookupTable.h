#pragma once

#include <array>

#include <units/length.h>
#include <units/time.h>
#include <units/angular_velocity.h>

class ShooterLookupTable {
 public:
  struct Status
  {
    units::meter_t distanceToTarget;
    units::turns_per_second_t shooterVelocity;
    units::second_t timeOfFlight;
  };
  static Status interpolate(units::meter_t iDistance)
  {
    for (unsigned int i = 0; i < LookupTable.size(); i++) {
      if (LookupTable[i].distanceToTarget > iDistance) {
        units::meter_t wDeltaNextPreviousDistance = LookupTable[i].distanceToTarget - LookupTable[i - 1].distanceToTarget;
        units::meter_t wDeltaCurrentPreviousDistance = iDistance - LookupTable[i - 1].distanceToTarget;
        double interpolationRatio = (wDeltaNextPreviousDistance / wDeltaCurrentPreviousDistance);
        units::turns_per_second_t wDeltaNextPreviousVelocity = (LookupTable[i].shooterVelocity - LookupTable[i - 1].shooterVelocity);
        units::turns_per_second_t wDesiredVelocity = wDeltaNextPreviousVelocity * interpolationRatio + LookupTable[i - 1].shooterVelocity;
        units::second_t wDeltaNextPreviousTOF = (LookupTable[i].timeOfFlight - LookupTable[i - 1].timeOfFlight);
        units::second_t wCorrespondentTOF = wDeltaNextPreviousTOF * interpolationRatio + LookupTable[i - 1].timeOfFlight;
        return Status{iDistance, wDesiredVelocity, wCorrespondentTOF};
      }
    }
    return LookupTable[LookupTable.size() - 1]; // return the last value if distance is bigger than last value of array
  }
 private:
    static constexpr std::array<Status, 3> LookupTable = {
    Status{1_m, 30_tps, 1.5_s},
    Status{1.5_m, 50_tps, 2.5_s},
    Status{2_m, 70_tps, 3.5_s}
  };
};