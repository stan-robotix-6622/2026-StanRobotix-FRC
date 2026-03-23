// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

namespace Configs
{
	using namespace rev::spark;
	class SwerveModule {
	 public:
		static SparkMaxConfig& DrivingConfig(bool iDrivingInverted)
		{
			static SparkMaxConfig drivingConfig{};

			constexpr double drivingFactor = ModuleConstants::kDrivingFactor;
			// units::compound_unit<units::volts, units::inverse<units::meters_per_second>>
			constexpr TemplateUnits::VoltageInverse<units::meters_per_second> drivingVelocityFeedForward = ModuleConstants::kNominalVoltage / ModuleConstants::kDriveWheelMaxSpeed;

			drivingConfig.Inverted(iDrivingInverted);
			drivingConfig.SetIdleMode(ModuleConstants::Config::kDrivingIdleMode);
			drivingConfig.Apply(SparkBaseConfig::Presets::REV_NEO());

			drivingConfig.encoder.VelocityConversionFactor(drivingFactor / ModuleConstants::Config::kRPMtoRPSFactor);
			drivingConfig.encoder.PositionConversionFactor(drivingFactor);

			drivingConfig.closedLoop.SetFeedbackSensor(ModuleConstants::Config::kDrivingClosedLoopFeedbackSensor);
			drivingConfig.closedLoop.Pid(ModuleConstants::kDrivingP, ModuleConstants::kDrivingI, ModuleConstants::kDrivingD);
			drivingConfig.closedLoop.OutputRange(-1, 1);

			drivingConfig.closedLoop.feedForward.kV(drivingVelocityFeedForward.value());

			return drivingConfig;
		}

		static SparkMaxConfig& TurningConfig(bool iEncoderInverted)
		{
			static SparkMaxConfig turningConfig{};

			constexpr double turningFactor = ModuleConstants::kTurningFactor;

			turningConfig.Inverted(ModuleConstants::Config::kTurningMotorInverted);
			turningConfig.SetIdleMode(ModuleConstants::Config::kTurningIdleMode);
			turningConfig.Apply(SparkBaseConfig::Presets::REV_NEO_550());

			turningConfig.absoluteEncoder.VelocityConversionFactor(turningFactor / ModuleConstants::Config::kRPMtoRPSFactor);
			turningConfig.absoluteEncoder.PositionConversionFactor(turningFactor);
			turningConfig.absoluteEncoder.Inverted(iEncoderInverted);
			turningConfig.absoluteEncoder.ZeroCentered(ModuleConstants::Config::kTurningEncoderZeroCentered);
			turningConfig.absoluteEncoder.Apply(AbsoluteEncoderConfig::Presets::REV_ThroughBoreEncoder());

			turningConfig.closedLoop.SetFeedbackSensor(ModuleConstants::Config::kTurningClosedLoopFeedbackSensor);
			turningConfig.closedLoop.Pid(ModuleConstants::kTurningP, ModuleConstants::kTurningI, ModuleConstants::kTurningD);
			turningConfig.closedLoop.OutputRange(-1, 1);
			turningConfig.closedLoop.PositionWrappingEnabled(ModuleConstants::Config::kTurningClosedLoopPositionWrapping);
			turningConfig.closedLoop.PositionWrappingMinInput(ModuleConstants::Config::kTurningClosedLoopMinInput);
			turningConfig.closedLoop.PositionWrappingMaxInput(ModuleConstants::Config::kTurningClosedLoopMaxInput);

			turningConfig.closedLoop.maxMotion.AllowedProfileError(ModuleConstants::Config::kTurningClosedLoopTolerance);
			turningConfig.closedLoop.maxMotion.CruiseVelocity(ModuleConstants::Config::kTurningCruiseVelocity.value());
			turningConfig.closedLoop.maxMotion.MaxAcceleration(ModuleConstants::Config::kTurningMaxAcceleration.value());

			return turningConfig;
		}
	};
}	 // namespace Configs
