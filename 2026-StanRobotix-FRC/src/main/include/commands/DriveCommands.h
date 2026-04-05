// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/CommandPtr.h>

#include <array>
#include <vector>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "subsystems/SubDrivetrain.h"

// Made based on the drive commands found at
// https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/spark_swerve/src/main/java/frc/robot/commands/DriveCommands.java
class DriveCommands {
 public:
	explicit DriveCommands(SubDrivetrain* iDrivetrain);

	frc2::CommandPtr getMeasureMaxAttainableSpeedCommand();
	frc2::CommandPtr getFeedforwardCharacterizationCommand();
	frc2::CommandPtr getWheelRadiusCharacterizationCommand();

 private:
	struct WheelRadiusCharacterizationState
	{
		std::array<frc::SwerveModulePosition, 4> positions = {
			frc::SwerveModulePosition{0_m, 0_rad},
			frc::SwerveModulePosition{0_m, 0_rad},
			frc::SwerveModulePosition{0_m, 0_rad},
			frc::SwerveModulePosition{0_m, 0_rad}};
		frc::Rotation2d lastAngle = 0.0_rad;
		units::radian_t gyroDelta = 0.0_rad;
	};

	SubDrivetrain* mDrivetrain = nullptr;

	WheelRadiusCharacterizationState* mState;
	frc::SlewRateLimiter<units::radians_per_second>* mRotationLimiter;
	frc::SlewRateLimiter<units::meters_per_second>* mSpeedLimiter;

	std::vector<units::radians_per_second_t>* mVelocitySamples;
	std::vector<units::volt_t>* mVoltageSamples;
	frc::Timer* mTimer;
};
