// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>

#include <memory>
#include <string>

#include <units/angular_velocity.h>

#include "LimelightHelpers.h"

class Limelight {
 public:
	explicit Limelight(std::string_view iName);

	std::optional<frc::Pose2d> getPoseEstimation(frc::Pose2d iCurrentRobotPose, units::radians_per_second_t iRobotRotationalVelocity, bool iMegaTag2);

	void setCameraPosition(units::meter_t forward, units::meter_t right, units::meter_t up, units::degree_t roll, units::degree_t pitch, units::degree_t yaw);

 private:
	LimelightHelpers::PoseEstimate mLimelightPoseEstimate;
	bool mRejectCameraUpdate;

	std::string mName;
	nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> mNTLimelightTable;
	nt::StructPublisher<frc::Pose2d> mPoseEstimatorPublisher;
};
