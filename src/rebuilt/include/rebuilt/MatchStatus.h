#pragma once

#include <string_view>

#include <units/time.h>

namespace Rebuilt
{
	namespace Match
	{
		enum Period {
			Autonomous,
			TransitionShift,
			Shift1,
			Shift2,
			Shift3,
			Shift4,
			Endgame
		};

		struct Status
		{
			bool hubActive;
			Period matchPeriod;
			std::string_view matchPeriodName;
			units::second_t timeLeftInPeriod;
			units::second_t timeLeftInMatch;
		};
	} // namespace Match
	Match::Status getMatchStatus();
} // namespace Rebuilt
