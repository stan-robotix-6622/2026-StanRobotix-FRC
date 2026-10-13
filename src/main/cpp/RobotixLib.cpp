// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotixLib.hpp"

#include <frc/MathUtil.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <cmath>
#include <string>

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

frc::Alert* robotixLib::getAlertForREVErrorMessage(rev::REVLibError iErrorMessage, std::string iMotorName)
{
	frc::Alert* wAlert;
	switch (iErrorMessage) {
		case rev::REVLibError::kOk:
			wAlert = new frc::Alert{"Configuration", "Le controlleur du moteur " + iMotorName + " a été configuré correctement", frc::Alert::AlertType::kInfo};
			break;
		case rev::REVLibError::kError:
			wAlert = new frc::Alert{"Configuration", "La configuration du controlleur du moteur " + iMotorName + " a résulté en une erreure", frc::Alert::AlertType::kError};
			break;
		case rev::REVLibError::kTimeout:
			wAlert = new frc::Alert{"Configuration", "La configuration du controlleur du moteur " + iMotorName + " ne s'est pas effectué à temps", frc::Alert::AlertType::kWarning};
			break;
		case rev::REVLibError::kCantFindFirmware:
			wAlert = new frc::Alert{"Configuration", "Na pas pu trouver le micrologiciel du controlleur du moteur " + iMotorName + " lors de sa configuration", frc::Alert::AlertType::kError};
			break;
		case rev::REVLibError::kFirmwareTooOld:
			wAlert = new frc::Alert{"Configuration", "Le micrologiciel du controlleur du moteur " + iMotorName + " est trop vieux", frc::Alert::AlertType::kWarning};
			break;
		case rev::REVLibError::kFirmwareTooNew:
			wAlert = new frc::Alert{"Configuration", "Le micrologiciel du controlleur du moteur " + iMotorName + " est trop récent", frc::Alert::AlertType::kWarning};
			break;
		case rev::REVLibError::kFollowConfigMismatch:
			wAlert = new frc::Alert{"Configuration", "La configuration du controlleur du moteur " + iMotorName + " a une incompatibilité pour le Follower", frc::Alert::AlertType::kError};
			break;
		case rev::REVLibError::kCANDisconnected:
			wAlert = new frc::Alert{"Configuration", "Le controlleur du moteur " + iMotorName + " est déconnecté du bus CAN", frc::Alert::AlertType::kError};
			break;
		case rev::REVLibError::kDuplicateCANId:
			wAlert = new frc::Alert{"Configuration", "Le CAN Id du controlleur du moteur " + iMotorName + " est le même qu'un autre controlleur de moteur", frc::Alert::AlertType::kError};
			break;
		case rev::REVLibError::kInvalidCANId:
			wAlert = new frc::Alert{"Configuration", "Le CAN Id du controlleur du moteur " + iMotorName + " est invalide", frc::Alert::AlertType::kError};
			break;
		case rev::REVLibError::kCannotPersistParametersWhileEnabled:
			wAlert = new frc::Alert{"Configuration", "Le peut pas persister la configuration du controlleur du moteur " + iMotorName + " lorsque le robot est activé", frc::Alert::AlertType::kWarning};
			break;

		default:
			wAlert = new frc::Alert{"Configuration", "La configuration du controlleur du moteur" + iMotorName + " a résulté en une erreure inconnue", frc::Alert::AlertType::kWarning};
			break;
	}
	if (iErrorMessage != rev::REVLibError::kOk)
	{
		wAlert->Set(true);
	}
	return wAlert;
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
