#pragma once

#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

namespace Configs
{
  using namespace rev::spark;
  class SwerveModule
  {
  public:
    static SparkMaxConfig &DrivingConfig(bool iDrivingInverted)
    {
      static SparkMaxConfig drivingConfig{};

      constexpr double drivingFactor = ModuleConstants::kDrivingFactor;
      constexpr TemplateUnits::VoltageInverse<units::meters_per_second> drivingVelocityFeedForward = ModuleConstants::kNominalVoltage / ModuleConstants::kDriveWheelMaxFreeSpeed;

      drivingConfig.Inverted(iDrivingInverted);
      drivingConfig.SetIdleMode(ModuleConstants::Config::kDrivingIdleMode);

      drivingConfig.encoder.VelocityConversionFactor(drivingFactor / ModuleConstants::Config::kRPMtoRPSFactor);
      drivingConfig.encoder.PositionConversionFactor(drivingFactor);

      drivingConfig.closedLoop.SetFeedbackSensor(ModuleConstants::Config::kDrivingClosedLoopFeedbackSensor);
      drivingConfig.closedLoop.Pid(ModuleConstants::kDrivingP, ModuleConstants::kDrivingI, ModuleConstants::kDrivingD);
      drivingConfig.closedLoop.OutputRange(-1, 1);

      drivingConfig.SmartCurrentLimit(ModuleConstants::kDrivingCurrentLimit.value());

      drivingConfig.closedLoop.feedForward.kV(drivingVelocityFeedForward.value());

      return drivingConfig;
    }

    static SparkMaxConfig &TurningConfig(bool iEncoderInverted)
    {
      static SparkMaxConfig turningConfig{};

      constexpr double turningFactor = ModuleConstants::kTurningFactor;

      turningConfig.Inverted(ModuleConstants::Config::kTurningMotorInverted);
      turningConfig.SetIdleMode(ModuleConstants::Config::kTurningIdleMode);

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
      turningConfig.closedLoop.maxMotion.CruiseVelocity(9999999);
      turningConfig.closedLoop.maxMotion.MaxAcceleration(9999999);

      turningConfig.SmartCurrentLimit(ModuleConstants::kTurningCurrentLimit.value());

      return turningConfig;
    }
  };
  class Shooter
  {
  public:
    static SparkMaxConfig &ShooterLeaderConfig()
    {
      static SparkMaxConfig leaderConfig{};
      constexpr double shootingFactor = 1 / ShooterConstants::kGearRatio;

      leaderConfig.Inverted(ShooterConstants::kInverted);
      leaderConfig.SetIdleMode(ShooterConstants::kIdleMode);

      leaderConfig.encoder.PositionConversionFactor(shootingFactor);
      leaderConfig.encoder.VelocityConversionFactor(shootingFactor / 60); // for rpm to tps

      leaderConfig.closedLoop.SetFeedbackSensor(ShooterConstants::Config::kShooterClosedLoopFeedbackSensor);
      // leaderConfig.closedLoop.Pid(ShooterConstants::PIDConstants::kP, ShooterConstants::PIDConstants::kI, ShooterConstants::PIDConstants::kD);
      // leaderConfig.closedLoop.OutputRange(-1, 1);

      leaderConfig.closedLoop.feedForward.kV(ShooterConstants::kV.value());
      leaderConfig.closedLoop.feedForward.kS(ShooterConstants::kS.value());
      leaderConfig.closedLoop.feedForward.kA(ShooterConstants::kA.value());

      // Configs added according to https://www.chiefdelphi.com/t/psa-rev-spark-default-velocity-filtering-is-still-really-bad-for-flywheels/514567
      // leaderConfig.encoder.UvwAverageDepth(4);
      // leaderConfig.encoder.UvwMeasurementPeriod(16);
      // leaderConfig.encoder.QuadratureAverageDepth(4);
      // leaderConfig.encoder.QuadratureMeasurementPeriod(16);

      leaderConfig.SmartCurrentLimit(ShooterConstants::kCurrentLimit.value());

      return leaderConfig;
    }
    static SparkMaxConfig &ShooterFollowerConfig()
    {
      static SparkMaxConfig followerConfig{};
      followerConfig.Apply(Configs::Shooter::ShooterLeaderConfig());
      followerConfig.Follow(CANid::kLeaderMotorShooterID, ShooterConstants::kFollowerinverted);
      return followerConfig;
    }
  };
  class Feeder
  {
  public:
    static SparkMaxConfig &Config()
    {
      static SparkMaxConfig feederConfig{};
      feederConfig.Inverted(FeederConstants::kInverted);
      feederConfig.SetIdleMode(FeederConstants::kIdleMode);

      feederConfig.SmartCurrentLimit(FeederConstants::kCurrentLimit.value());
      return feederConfig;
    }
  };
  class Intake
  {
  public:
    static SparkMaxConfig &Config()
    {
      static SparkMaxConfig intakeConfig{};
      constexpr double intakeFactor = 1 / IntakeConstants::kGearRatio;

      intakeConfig.Inverted(IntakeConstants::kInverted);
      intakeConfig.SetIdleMode(IntakeConstants::kIdleMode);

      intakeConfig.encoder.PositionConversionFactor(intakeFactor);
      intakeConfig.encoder.VelocityConversionFactor(intakeFactor);

      intakeConfig.SmartCurrentLimit(IntakeConstants::kCurrentLimit.value());

      return intakeConfig;
    }
  };
  class Pivot
  {
  public:
    static SparkMaxConfig &Config()
    {
      static SparkMaxConfig pivotConfig{};
      constexpr double pivotFactor = (2 * std::numbers::pi) / PivotConstants::kGearRatio;

      pivotConfig.Inverted(PivotConstants::kInverted);
      pivotConfig.SetIdleMode(PivotConstants::kIdleMode);

      pivotConfig.encoder.PositionConversionFactor(pivotFactor);
      pivotConfig.encoder.VelocityConversionFactor(pivotFactor);

      pivotConfig.SmartCurrentLimit(PivotConstants::kCurrentLimit.value());

      return pivotConfig;
    }
  };
} // namespace Configs