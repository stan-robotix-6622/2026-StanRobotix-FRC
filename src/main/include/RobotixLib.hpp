#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

#include <string>

#include <units/angle.h>
#include <units/length.h>

namespace robotixLib
{
	// Made from the example code at https://www.chiefdelphi.com/uploads/default/original/3X/b/a/ba7ccfd90bac0934e374dd4459d813cee2903942.pdf
	double deadband(double iInput, double iThreshold, bool iSquared = false);

	namespace odometryUtils
	{
		units::degree_t GetAngleToTarget(frc::Translation2d iCurrentTranslation, frc::Translation2d iTargetTranslation);

		units::meter_t GetDistanceToTarget(frc::Translation2d iCurrentTranslation, frc::Translation2d iTargetTranslation);
	} // namespace odometryUtils
	
	namespace pathplannerUtils
	{
		frc::Pose2d getStartingPoseOfAuto(std::string iAutoName);
	} // namespace pathplannerUtils
} // namespace robotixLib
