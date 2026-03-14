// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <rev/SparkBase.h>              // Include Spark variable types
#include <rev/config/SparkBaseConfig.h> // For the spark IdleMode
#include <pathplanner/lib/config/RobotConfig.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace TemplateUnits {
  template <typename Unit>
using VoltageInverse = units::unit_t<units::detail::unit_multiply<units::voltage::volts, units::inverse<Unit>>, double, units::linear_scale>;

}
namespace OperatorConstants
{
  inline constexpr int kDriverControllerPort = 0;

  // Button mappings
  // For XboxController:
  // A = 1; B = 2; X = 3; Y = 4; RightBumper = 5; LeftBumper = 6
  inline constexpr int kPivotDownButton = 2; // B

  inline constexpr int kResetIMUButton = 5;  // RightBumper
  inline constexpr int kResetPoseButton = 6; // LeftBumper

  inline constexpr int kShootButton = 4;       // Y
  inline constexpr int kUnstuckFuelButton = 3; // X
  inline constexpr int kFeedButton = 1;        // A
  // inline constexpr int kIndexButton = 6;       // LeftBumper
} // namespace OperatorConstants

namespace ShooterConstants
{
  inline constexpr units::volt_t kS = 0_V;
  inline constexpr TemplateUnits::VoltageInverse<units::turns_per_second> kV = 8_V / 61.523844_tps;
  inline constexpr TemplateUnits::VoltageInverse<units::turns_per_second_squared> kA = 0_V / 1_tr_per_s_sq;

  inline constexpr bool kInverted = false;
  inline constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  inline constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  inline constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;

  inline constexpr units::turns_per_second_t kVitesseVoulue = 55_tps;

  inline constexpr bool kFollowerinverted = false;

  namespace PIDConstants
  {
    inline constexpr double kP = 2; // T'is be a placeholder :)
    inline constexpr double kI = 0;
    inline constexpr double kD = 0;

    inline constexpr units::turns_per_second_t setpoint = 55_tps; // its NOT(it actually is) a placeholder :)
  }
}

namespace FeederConstants
{
  inline constexpr units::volt_t kDesiredVoltage = 2_V; // placeholder :)

  inline constexpr bool kInverted = true;
  inline constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  inline constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  inline constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;

}

namespace IndexerConstants
{
  inline constexpr units::volt_t kDesiredVoltage = 2_V; // placeholder :)

  inline constexpr bool kInverted = true;
  inline constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  inline constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  inline constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;
}

namespace PathPlannerConstants
{
  inline constexpr double kPTranslation = 5.0;
  inline constexpr double kITranslation = 0.0;
  inline constexpr double kDTranslation = 0.0;
  inline constexpr double kPRotation = 5.0;
  inline constexpr double kIRotation = 0.0;
  inline constexpr double kDRotation = 0.0;

  inline constexpr units::meters_per_second_t kMaxVelocity = .3_mps;
  inline constexpr units::meters_per_second_squared_t kMaxAcceleration = 0.3_mps_sq;
  inline constexpr units::degrees_per_second_t kMaxAngularVelocity = 36.0_deg_per_s;
  inline constexpr units::degrees_per_second_squared_t kMaxAngularAcceleration = 72.0_deg_per_s_sq;
}

namespace DrivetrainConstants
{
  // Left-Right
  inline constexpr units::meter_t kRobotWidth = 28_in;
  // Front-Back
  inline constexpr units::meter_t kRobotLength = 26.875_in;
  // In both directions
  inline constexpr units::meter_t kModuleCornerOffset = 1.75_in;

  // We take for granted a rectangular frame
  inline constexpr frc::Translation2d kFrontLeftTranslation = frc::Translation2d{(kRobotLength / 2 - kModuleCornerOffset), (kRobotWidth / 2 - kModuleCornerOffset)};
  inline constexpr frc::Translation2d kFrontRightTranslation = frc::Translation2d{(kRobotLength / 2 - kModuleCornerOffset), -(kRobotWidth / 2 - kModuleCornerOffset)};
  inline constexpr frc::Translation2d kBackLeftTranslation = frc::Translation2d{-(kRobotLength / 2 - kModuleCornerOffset), (kRobotWidth / 2 - kModuleCornerOffset)};
  inline constexpr frc::Translation2d kBackRightTranslation = frc::Translation2d{-(kRobotLength / 2 - kModuleCornerOffset), -(kRobotWidth / 2 - kModuleCornerOffset)};

  inline constexpr units::meters_per_second_t kSpeedConstant = 2_mps;                            // Temporary value
  inline constexpr units::radians_per_second_t kSpeedConstant0 = std::numbers::pi * 1_rad_per_s; // Temporary value
  namespace Commands
  {
    inline constexpr units::second_t kFeedforwartStartDelay = 2.0_s;
    inline constexpr TemplateUnits::VoltageInverse<units::seconds> kFeedforwardRampRate = 1_V / 1_s;
    inline constexpr units::second_t kWheelRadiusMeasurementStartDelay = 1.0_s;
    inline constexpr units::radians_per_second_t kWheelRadiusMaxVelocity = 0.25_rad_per_s;
    inline constexpr units::radians_per_second_squared_t kWheelRadiusRampRate = 0.05_rad_per_s_sq;
  }
}

namespace ModuleConstants
{
  inline constexpr double kDrivingMotorGearRatio = 4.71;                                       // 5.08 rotations of the motor for 1 rotation of the ouput
  inline constexpr units::volt_t kNominalVoltage = 12_V;                                       // The voltage at which the max speeds are mesured
  inline constexpr units::meter_t kWheelRadius = 1.5_in;                                       // The radius of REV's plastic wheels
  inline constexpr units::meter_t kWheelPerimeter = kWheelRadius * 2 * std::numbers::pi;       // in meters (diametre in inches * convertion to meters * pi)
  inline constexpr units::radians_per_second_t kTurningWheelFreeSpeedRadps = 24.260_rad_per_s; // TODO: Verify?
  inline constexpr units::meters_per_second_t kDriveWheelMaxSpeed = 4.9180_mps;                // TODO: Verify?

  inline constexpr double kDrivingFactor = ModuleConstants::kWheelPerimeter.value() / kDrivingMotorGearRatio;
  inline constexpr double kTurningFactor = 2 * std::numbers::pi;

  inline constexpr rev::spark::SparkLowLevel::ControlType kDrivingClosedLoopControlType = rev::spark::SparkLowLevel::ControlType::kVelocity;
  inline constexpr rev::spark::SparkLowLevel::ControlType kTurningClosedLoopControlType = rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl;

  inline constexpr rev::spark::SparkLowLevel::MotorType kDrivingMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
  inline constexpr rev::spark::SparkLowLevel::MotorType kTurningMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;

  inline constexpr rev::ResetMode kDrivingResetMode = rev::ResetMode::kResetSafeParameters;
  inline constexpr rev::ResetMode kTurningResetMode = rev::ResetMode::kResetSafeParameters;

  inline constexpr rev::PersistMode kDrivingPersistMode = rev::PersistMode::kPersistParameters;
  inline constexpr rev::PersistMode kTurningPersistMode = rev::PersistMode::kPersistParameters;

  inline constexpr double kTurningP = 0.3;
  inline constexpr double kTurningI = 0.0;
  inline constexpr double kTurningD = 0.0;
  inline constexpr double kDrivingP = 0.04;
  inline constexpr double kDrivingI = 0.0;
  inline constexpr double kDrivingD = 0.0;

  namespace Config
  {
    inline constexpr double kRPMtoRPSFactor = 60;

    inline constexpr units::radians_per_second_t kTurningCruiseVelocity = 2_rad_per_s * std::numbers::pi;
    inline constexpr units::radians_per_second_squared_t kTurningMaxAcceleration = 4_rad_per_s_sq * std::numbers::pi;

    inline constexpr rev::spark::SparkBaseConfig::IdleMode kDrivingIdleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;
    inline constexpr rev::spark::SparkBaseConfig::IdleMode kTurningIdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;

    inline constexpr rev::spark::FeedbackSensor kDrivingClosedLoopFeedbackSensor = rev::spark::FeedbackSensor::kPrimaryEncoder;
    inline constexpr rev::spark::FeedbackSensor kTurningClosedLoopFeedbackSensor = rev::spark::FeedbackSensor::kAbsoluteEncoder;

    inline constexpr bool kTurningMotorInverted = false;
    inline constexpr bool kTurningEncoderZeroCentered = false;
    inline constexpr bool kTurningClosedLoopPositionWrapping = true;
    inline constexpr double kTurningClosedLoopMinInput = -ModuleConstants::kTurningFactor / 2;
    inline constexpr double kTurningClosedLoopMaxInput = ModuleConstants::kTurningFactor / 2;
    inline constexpr double kTurningClosedLoopTolerance = 0.01 * ModuleConstants::kTurningFactor;
  }
}

namespace LimelightConstants
{
  inline constexpr bool kUseMegaTag2 = true;

  inline constexpr std::string_view kName = "";

  inline constexpr units::meter_t kForward = -1.125_in;
  inline constexpr units::meter_t kRight = -9.75_in;
  inline constexpr units::meter_t kUp = 20.25_in;

  inline constexpr units::degree_t kRoll = 0_deg;
  inline constexpr units::degree_t kPitch = 0_deg;
  inline constexpr units::degree_t kYaw = -90_deg;

  inline constexpr double kPoseEstimatorStandardDeviationX = 0.7;      // Default/Recommended values
  inline constexpr double kPoseEstimatorStandardDeviationY = 0.7;      // Default/Recommended values
  inline constexpr double kPoseEstimatorStandardDeviationYaw = 999999; // Default/Recommended values
}

// Values found at https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf#page=3
// For the Welded Field
namespace FieldConstants
{
  inline constexpr frc::Translation2d kHubCenterTranslation2d = frc::Translation2d{182.11_in, 158.32_in};   // From the right corner of blue alliance wall
  inline constexpr frc::Pose2d kHubCenterPose2d = frc::Pose2d{kHubCenterTranslation2d, 0_rad};              // From the right corner of blue alliance wall
  inline constexpr frc::Translation2d kFieldCenterTranslation2d = frc::Translation2d{325.61_in, 158.32_in}; // From the right corner of blue alliance wall
  inline constexpr frc::Pose2d kFieldCenterPose2d = frc::Pose2d{kHubCenterTranslation2d, 0_rad};            // From the right corner of blue alliance wall
}

namespace CANid
{
  inline constexpr int kMotorIndexerID = 12;
  inline constexpr int kMotorFeederID = 13;

  inline constexpr int kLeaderMotorShooterID = 16;
  inline constexpr int kFollowerMotorShooterID = 17;

  inline constexpr int kBackRightMotorID = 8;
  inline constexpr int kBackRightMotor550ID = 7;
  inline constexpr int kFrontRightMotorID = 4;
  inline constexpr int kFrontRightMotor550ID = 3;
  inline constexpr int kFrontLeftMotorID = 6;
  inline constexpr int kFrontLeftMotor550ID = 5;
  inline constexpr int kBackLeftMotorID = 2;
  inline constexpr int kBackLeftMotor550ID = 1;

  inline constexpr int kMotorPivotID = 9;
  inline constexpr int kMotorIntakeID = 10;
  inline constexpr int kIMUPigeonID = 0;
}

namespace IntakeConstants
{
  inline constexpr bool kInverted = false;
  inline constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  inline constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  inline constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;

  inline constexpr double kSpeed = 0.8; // a modifier (valeur temporaire)
}

namespace PivotConstants
{
  inline constexpr double kGearRatio = 16;
  inline constexpr double kOffset = 6.2;
  inline constexpr double kP = 1.3;  // en attendant
  inline constexpr double kI = 0.4;  // en attendant
  inline constexpr double kD = 0.15; // en attendant
  inline constexpr units::volt_t kG = 0.80_V;
  inline constexpr double setpointUp = std::numbers::pi / 2;    // 90 deg up
  inline constexpr double setpointDown = std::numbers::pi / 18; // 10 deg up

  inline constexpr bool kInverted = false;
  inline constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  inline constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  inline constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;
}
