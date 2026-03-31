#pragma once

#include <array>

#include <units/length.h>
#include <units/time.h>
#include <units/angular_velocity.h>

class LookupTable {
 public:
  struct ShooterStatus
  {
    units::meter_t distanceToTarget;
    units::turns_per_second_t shooterVelocity;
    units::second_t timeOfFlight;
  };
  static ShooterStatus interpolate(units::meter_t iDistance)
  {
    for (unsigned int i = 0; i < ShooterLookupTable.size(); i++) {
      if (ShooterLookupTable[i].distanceToTarget >= iDistance) {
        units::meter_t wDeltaNextPreviousDistance = ShooterLookupTable[i].distanceToTarget - ShooterLookupTable[i - 1].distanceToTarget;
        units::meter_t wDeltaCurrentPreviousDistance = iDistance - ShooterLookupTable[i - 1].distanceToTarget;
        double interpolationRatio = (wDeltaCurrentPreviousDistance / wDeltaNextPreviousDistance);
        units::turns_per_second_t wDeltaNextPreviousVelocity = (ShooterLookupTable[i].shooterVelocity - ShooterLookupTable[i - 1].shooterVelocity);
        units::turns_per_second_t wDesiredVelocity = wDeltaNextPreviousVelocity * interpolationRatio + ShooterLookupTable[i - 1].shooterVelocity;
        units::second_t wDeltaNextPreviousTOF = (ShooterLookupTable[i].timeOfFlight - ShooterLookupTable[i - 1].timeOfFlight);
        units::second_t wCorrespondentTOF = wDeltaNextPreviousTOF * interpolationRatio + ShooterLookupTable[i - 1].timeOfFlight;
        return ShooterStatus{iDistance, wDesiredVelocity, wCorrespondentTOF};
      }
    }
    return ShooterLookupTable[ShooterLookupTable.size() - 1]; // return the last value if iDistance is bigger than last value of array
  }
 private:
  // This array needs to be sorted by the distanceToTarget value of the structs
  static constexpr std::array<ShooterStatus, 43> ShooterLookupTable = {
    ShooterStatus{0.5_m, 4.49_tps, 0.71_s},
    ShooterStatus{0.6_m, 5.67_tps, 0.77_s},
    ShooterStatus{0.7_m, 6.92_tps, 0.84_s},
    ShooterStatus{0.8_m, 8.22_tps, 0.89_s},
    ShooterStatus{0.9_m, 9.59_tps, 0.95_s},
    ShooterStatus{1.0_m, 11.0_tps, 1.0_s},
    ShooterStatus{1.1_m, 12.46_tps, 1.05_s},
    ShooterStatus{1.2_m, 13.95_tps, 1.1_s},
    ShooterStatus{1.3_m, 15.48_tps, 1.14_s},
    ShooterStatus{1.4_m, 17.05_tps, 1.18_s},
    ShooterStatus{1.5_m, 18.64_tps, 1.22_s},
    ShooterStatus{1.6_m, 20.27_tps, 1.26_s},
    ShooterStatus{1.7_m, 21.92_tps, 1.3_s},
    ShooterStatus{1.8_m, 23.59_tps, 1.34_s},
    ShooterStatus{1.9_m, 25.3_tps, 1.38_s},
    ShooterStatus{2.0_m, 27.02_tps, 1.41_s},
    ShooterStatus{2.1_m, 28.77_tps, 1.45_s},
    ShooterStatus{2.2_m, 30.53_tps, 1.48_s},
    ShooterStatus{2.3_m, 32.32_tps, 1.52_s},
    ShooterStatus{2.4_m, 34.13_tps, 1.55_s},
    ShooterStatus{2.5_m, 35.95_tps, 1.58_s},
    ShooterStatus{2.6_m, 37.79_tps, 1.61_s},
    ShooterStatus{2.7_m, 39.65_tps, 1.64_s},
    ShooterStatus{2.8_m, 41.52_tps, 1.67_s},
    ShooterStatus{2.9_m, 43.41_tps, 1.7_s},
    ShooterStatus{3.0_m, 45.31_tps, 1.73_s},
    ShooterStatus{3.1_m, 47.23_tps, 1.76_s},
    ShooterStatus{3.2_m, 49.16_tps, 1.79_s},
    ShooterStatus{3.3_m, 51.11_tps, 1.82_s},
    ShooterStatus{3.4_m, 53.07_tps, 1.84_s},
    ShooterStatus{3.5_m, 55.04_tps, 1.87_s},
    ShooterStatus{3.6_m, 57.03_tps, 1.9_s},
    ShooterStatus{3.7_m, 59.02_tps, 1.92_s},
    ShooterStatus{3.8_m, 61.03_tps, 1.95_s},
    ShooterStatus{3.9_m, 63.05_tps, 1.97_s},
    ShooterStatus{4.0_m, 65.08_tps, 2.0_s},
    ShooterStatus{4.1_m, 67.12_tps, 2.02_s},
    ShooterStatus{4.2_m, 69.18_tps, 2.05_s},
    ShooterStatus{4.3_m, 71.24_tps, 2.07_s},
    ShooterStatus{4.4_m, 73.31_tps, 2.1_s},
    ShooterStatus{4.5_m, 75.39_tps, 2.12_s},
    ShooterStatus{4.6_m, 77.49_tps, 2.14_s},
    ShooterStatus{4.7_m, 79.59_tps, 2.17_s}
  };
};

#include "struct/ShooterStatusStruct.h"