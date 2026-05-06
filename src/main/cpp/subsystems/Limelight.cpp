// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Limelight.h"

#include "Constants.h"

Limelight::Limelight(std::string_view iName)
{
	mName = iName;
	mNTLimelightTable = inst.GetTable("SmartDashboard/" + mName);
	mPoseEstimatorPublisher = mNTLimelightTable->GetStructTopic<frc::Pose2d>("Pose Estimator").Publish();
}

std::optional<frc::Pose2d> Limelight::getPoseEstimation(frc::Pose2d iCurrentRobotPose, units::radians_per_second_t iRobotRotationalVelocity, bool iMegaTag2)
{
	// Update la rotation du robot pour la Limelight

	LimelightHelpers::SetRobotOrientation(mName, iCurrentRobotPose.Rotation().Degrees().value(), 0, 0, 0, 0, 0);

	if (iMegaTag2) {
		mLimelightPoseEstimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(mName);
	}
	else {
		mLimelightPoseEstimate = LimelightHelpers::getBotPoseEstimate_wpiBlue(mName);
	}

	// reject the camera update if the PoseEstimate is not valid
	mRejectCameraUpdate = !LimelightHelpers::validPoseEstimate(mLimelightPoseEstimate);

	if (units::math::abs(iRobotRotationalVelocity) > 180_deg_per_s) {
		mRejectCameraUpdate = true;
	}
	else if (mLimelightPoseEstimate.tagCount == 0) {
		mRejectCameraUpdate = true;
	}
	else if (mLimelightPoseEstimate.pose == frc::Pose2d(0_m, 0_m, 0_rad)) {
		mRejectCameraUpdate = true;
	}
	else if (!iMegaTag2 && mLimelightPoseEstimate.tagCount < 2) {
		mRejectCameraUpdate = true;
	}

	if (!mRejectCameraUpdate) {
		// LimelightHelpers::PrintPoseEstimate(mLimelightPoseEstimate);
		mPoseEstimatorPublisher.Set(mLimelightPoseEstimate.pose);
		return mLimelightPoseEstimate.pose;
	}
	return {};
}

void Limelight::setCameraPosition(units::meter_t iForward, units::meter_t iRight, units::meter_t iUp, units::degree_t iRoll, units::degree_t iPitch, units::degree_t iYaw)
{
	LimelightHelpers::setCameraPose_RobotSpace(
			mName,
			iForward.value(),
			iRight.value(),
			iUp.value(),
			iRoll.value(),
			iPitch.value(),
			iYaw.value());
}
