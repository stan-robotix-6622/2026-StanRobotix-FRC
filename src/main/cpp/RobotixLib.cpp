#include "RobotixLib.hpp"

#include <frc/MathUtil.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <cmath>

frc::Pose2d robotixLib::pathplannerUtils::getStartingPoseOfAuto(std::string iAutoName)
{
	// wPaths is of type std::vector<std::shared_ptr<pathplanner::PathPlannerPath>>
	auto wPaths = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(iAutoName);
	return wPaths.front().get()->getStartingHolonomicPose().value(); // Return the first pose of the first path's poses
}

double robotixLib::deadband(double iInput, double iThreshold, bool iSquared)
{
	if (std::abs(iInput) < iThreshold) {
		return 0.0;
	}
	if (!iSquared) {
		// ((iInput > 0) - (iInput < 0)) gives us the sign of iInput
		// Then we scale the value of iInput over the range [-1; iThreshold] or [iTheshold; 1]
		return (1 / (1 - iThreshold)) * (iInput - (((iInput > 0) - (iInput < 0)) * iThreshold));
	}
	// Same as above but we square the value and keep the sign of the initial iInput
	return frc::CopyDirectionPow((1 / (1 - iThreshold)) * (iInput - (((iInput > 0) - (iInput < 0)) * iThreshold)), 2);
}

units::degree_t robotixLib::odometryUtils::GetAngleToTarget(frc::Translation2d iCurrentTranslation, frc::Translation2d iTargetTranslation)
{
	frc::Translation2d wTranslationToTarget = iTargetTranslation - iCurrentTranslation;
	return wTranslationToTarget.Angle().Degrees();
}

units::meter_t robotixLib::odometryUtils::GetDistanceToTarget(frc::Translation2d iCurrentTranslation, frc::Translation2d iTargetTranslation)
{
	frc::Translation2d wTranslationToTarget = iTargetTranslation - iCurrentTranslation;
	return wTranslationToTarget.Norm();
}
