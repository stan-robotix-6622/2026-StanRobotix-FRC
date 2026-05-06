#include "rebuilt/MatchStatus.h"

#include <frc/DriverStation.h>

#include <string>

#include <units/math.h>

Rebuilt::Match::Status Rebuilt::getMatchStatus()
{
	Rebuilt::Match::Status status;
	// mAlliance is of type std::optional<frc::DriverStation::Alliance>
	auto mAlliance = frc::DriverStation::GetAlliance();
	if (mAlliance) {
		std::string gameData = frc::DriverStation::GetGameSpecificMessage();
		units::second_t matchTime = frc::DriverStation::GetMatchTime();
		status.timeLeftInMatch = matchTime;

		if (matchTime <= 30_s) {
			if (gameData[0] != 'B' && gameData[0] != 'R') {
				status.timeLeftInMatch = matchTime + 140_s;
				status.timeLeftInPeriod = matchTime;
				status.matchPeriod = Rebuilt::Match::Period::Autonomous;
				status.matchPeriodName = "Autonomous";
			}
			else {
				status.timeLeftInPeriod = matchTime;
				status.matchPeriod = Rebuilt::Match::Period::Endgame;
				status.matchPeriodName = "Endgame";
			}
		}
		else {
			status.timeLeftInPeriod = units::math::fmod(matchTime - 30_s, 25_s);
		}
		if (matchTime >= 130_s) {
			status.matchPeriod = Rebuilt::Match::Period::TransitionShift;
			status.matchPeriodName = "Transition Shift";
		}
		else if (matchTime >= 105_s) {
			status.matchPeriod = Rebuilt::Match::Period::Shift1;
			status.matchPeriodName = "Shift 1";
		}
		else if (matchTime >= 80_s) {
			status.matchPeriod = Rebuilt::Match::Period::Shift2;
			status.matchPeriodName = "Shift 2";
		}
		else if (matchTime >= 55_s) {
			status.matchPeriod = Rebuilt::Match::Period::Shift3;
			status.matchPeriodName = "Shift 3";
		}
		else if (matchTime >= 30_s) {
			status.matchPeriod = Rebuilt::Match::Period::Shift4;
			status.matchPeriodName = "Shift 4";
		}
		if (status.matchPeriod == Rebuilt::Match::Period::Autonomous || status.matchPeriod == Rebuilt::Match::Period::TransitionShift || status.matchPeriod == Rebuilt::Match::Period::Endgame) {
			status.hubActive = true;
		}
		else {
			if (gameData[0] == 'B') { // Blue starts inactive
				if (status.matchPeriod == Rebuilt::Match::Period::Shift2 || status.matchPeriod == Rebuilt::Match::Period::Shift4) {
					status.hubActive = mAlliance.value() == frc::DriverStation::kBlue;
				}
				else {
					status.hubActive = mAlliance.value() == frc::DriverStation::kRed;
				}
			}
			else if (gameData[0] == 'R') { // Red starts inactive
				if (status.matchPeriod == Rebuilt::Match::Period::Shift2 || status.matchPeriod == Rebuilt::Match::Period::Shift4) {
					status.hubActive = mAlliance.value() == frc::DriverStation::kRed;
				}
				else {
					status.hubActive = mAlliance.value() == frc::DriverStation::kBlue;
				}
			}
			else {
				status.hubActive = false;
			}
		}
	}
	return status;
}
