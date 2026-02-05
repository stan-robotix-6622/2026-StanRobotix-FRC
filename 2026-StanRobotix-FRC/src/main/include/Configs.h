#pragma once

#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

using namespace rev::spark;

namespace Configs {
class SwerveModule {
 public:
  static SparkMaxConfig& DrivingConfig(bool iDrivingInverted) {
    static SparkMaxConfig drivingConfig{};

    constexpr double drivingFactor = ModuleConstants::kWheelPerimeter.value() / ModuleConstants::kDrivingFactor;
    constexpr double nominalVoltage = 12.0;
    constexpr double drivingVelocityFeedForward = nominalVoltage / ModuleConstants::kDriveWheelFreeSpeedRps;

    drivingConfig.Inverted(iDrivingInverted);
    drivingConfig.SetIdleMode(ModuleConstants::Config::kDrivingIdleMode);
    drivingConfig.Apply(SparkBaseConfig::Presets::REV_NEO());

    drivingConfig.encoder.VelocityConversionFactor(drivingFactor / ModuleConstants::kRPMtoRPSFactor);
    drivingConfig.encoder.PositionConversionFactor(drivingFactor);

    drivingConfig.closedLoop.SetFeedbackSensor(ModuleConstants::Config::kDrivingClosedLoopFeedbackSensor);
    drivingConfig.closedLoop.Pid(ModuleConstants::kDrivingP, ModuleConstants::kDrivingI, ModuleConstants::kDrivingD);
    drivingConfig.closedLoop.OutputRange(-1, 1);

    drivingConfig.closedLoop.feedForward.kV(drivingVelocityFeedForward);

    return drivingConfig;
  }

  static SparkMaxConfig& TurningConfig(bool iEncoderInverted) {
    static SparkMaxConfig turningConfig{};

    constexpr double turningFactor = ModuleConstants::kTurningFactor;

    turningConfig.Inverted(ModuleConstants::Config::kTurningMotorInverted);
    turningConfig.SetIdleMode(ModuleConstants::Config::kTurningIdleMode);
    turningConfig.Apply(SparkBaseConfig::Presets::REV_NEO_550());

    turningConfig.absoluteEncoder.VelocityConversionFactor(turningFactor / ModuleConstants::kRPMtoRPSFactor);
    turningConfig.absoluteEncoder.PositionConversionFactor(turningFactor);
    turningConfig.absoluteEncoder.Inverted(iEncoderInverted);
    turningConfig.absoluteEncoder.Apply(AbsoluteEncoderConfig::Presets::REV_ThroughBoreEncoder());

    turningConfig.closedLoop.SetFeedbackSensor(ModuleConstants::Config::kTurningClosedLoopFeedbackSensor);
    turningConfig.closedLoop.Pid(ModuleConstants::kTurningP, ModuleConstants::kTurningI, ModuleConstants::kTurningD);
    turningConfig.closedLoop.OutputRange(-1, 1);
    turningConfig.closedLoop.PositionWrappingEnabled(ModuleConstants::Config::kTurningClosedLoopPositionWrapping);
    turningConfig.closedLoop.PositionWrappingMinInput(ModuleConstants::Config::kTurningClosedLoopMinInput);
    turningConfig.closedLoop.PositionWrappingMaxInput(ModuleConstants::Config::kTurningClosedLoopMaxInput);

    turningConfig.closedLoop.maxMotion.AllowedProfileError(ModuleConstants::Config::kTurningClosedLoopTolerance);

    return turningConfig;
  }
};
}  // namespace Configs