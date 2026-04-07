// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>

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
				if (i > 0) {
					units::meter_t wDeltaNextPreviousDistance = ShooterLookupTable[i].distanceToTarget - ShooterLookupTable[i - 1].distanceToTarget;
					units::meter_t wDeltaCurrentPreviousDistance = iDistance - ShooterLookupTable[i - 1].distanceToTarget;
					double interpolationRatio = (wDeltaCurrentPreviousDistance / wDeltaNextPreviousDistance);
					units::turns_per_second_t wDeltaNextPreviousVelocity = (ShooterLookupTable[i].shooterVelocity - ShooterLookupTable[i - 1].shooterVelocity);
					units::turns_per_second_t wDesiredVelocity = wDeltaNextPreviousVelocity * interpolationRatio + ShooterLookupTable[i - 1].shooterVelocity;
					units::second_t wDeltaNextPreviousTOF = (ShooterLookupTable[i].timeOfFlight - ShooterLookupTable[i - 1].timeOfFlight);
					units::second_t wCorrespondentTOF = wDeltaNextPreviousTOF * interpolationRatio + ShooterLookupTable[i - 1].timeOfFlight;
					return {iDistance, wDesiredVelocity, wCorrespondentTOF};
				}
				return {iDistance, 0_tps, 0_s};
			}
		}
		return ShooterLookupTable[ShooterLookupTable.size() - 1];	 // return the last value if iDistance is bigger than last value of array
	}

 private:
	// This array needs to be sorted by the distanceToTarget value of the structs
	static constexpr std::array<ShooterStatus, 5> ShooterLookupTable = {
		ShooterStatus{0_m, 0_tps, 0_s},
		ShooterStatus{2.3013_m, 40_tps, 0_s},
		ShooterStatus{2.3497_m, 42.5_tps, 0_s},
		ShooterStatus{2.522_m, 44.5_tps, 0_s},
		ShooterStatus{3.17149_m, 51_tps, 0_s}};
};

#include "struct/ShooterStatusStruct.h"
