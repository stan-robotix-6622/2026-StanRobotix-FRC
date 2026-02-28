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

typedef units::unit_t<units::compound_unit<units::volts, units::inverse<units::turns_per_second_squared>>, double, units::linear_scale> kAunit; // V / turn / s^2
typedef units::unit_t<units::compound_unit<units::volts, units::inverse<units::turns_per_second>>, double, units::linear_scale> kVunit;         // V / turn / s

namespace OperatorConstants
{
  inline constexpr int kDriverControllerPort = 0;

  // Button mappings
  // For XboxController:
  // A = 1; B = 2; X = 3; Y = 4; RightBumper = 5; LeftBumper = 6
  constexpr int kPivotDownButton = 2; // B

  constexpr int kResetIMUButton = 5;  // RightBumper
  constexpr int kResetPoseButton = 6; // LeftBumper

  constexpr int kShootButton = 4;       // Y
  constexpr int kUnstuckFuelButton = 3; // X
  constexpr int kFeedButton = 1;        // A
  // constexpr int kIndexButton = 6;       // LeftBumper
} // namespace OperatorConstants

namespace ShooterConstants
{
  constexpr units::volt_t kS = 0_V;
  constexpr kVunit kV = 8_V / 61.523844_tps;
  // constexpr kAunit kA = 0_V / 1_tr_per_s_sq;

  constexpr bool kInverted = false;
  constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;

  constexpr units::turns_per_second_t kVitesseVoulue = 60_tps;

  constexpr bool kFollowerinverted = false;

  namespace PIDConstants
  {
    constexpr double kP = 2; // T'is be a placeholder :)
    constexpr double kI = 0;
    constexpr double kD = 0;

    constexpr units::turns_per_second_t setpoint = 60_tps; // its NOT(it actually is) a placeholder :)
  }
}

namespace FeederConstants
{
  constexpr units::volt_t kDesiredVoltage = 2_V; // placeholder :)

  constexpr bool kInverted = true;
  constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;

}

namespace IndexerConstants
{
  constexpr units::volt_t kDesiredVoltage = 2_V; // placeholder :)

  constexpr bool kInverted = true;
  constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;
}

namespace PathPlannerConstants
{
  constexpr double kPTranslation = 5.0;
  constexpr double kITranslation = 0.0;
  constexpr double kDTranslation = 0.0;
  constexpr double kPRotation = 5.0;
  constexpr double kIRotation = 0.0;
  constexpr double kDRotation = 0.0;

  constexpr double kPathPlannerSpeedModulation = 1.0;
  constexpr units::meters_per_second_t kMaxVelocity = .3_mps;
  constexpr units::meters_per_second_squared_t kMaxAcceleration = 0.3_mps_sq;
  constexpr units::degrees_per_second_t kMaxAngularVelocity = 36.0_deg_per_s;
  constexpr units::degrees_per_second_squared_t kMaxAngularAcceleration = 72.0_deg_per_s_sq;
}

namespace DrivetrainConstants
{
  // Left-Right
  constexpr units::meter_t kRobotWidth = 28_in;
  // Front-Back
  constexpr units::meter_t kRobotLength = 26.875_in;
  // In both directions
  constexpr units::meter_t kModuleCornerOffset = 1.75_in;

  // We take for granted a rectangular frame
  constexpr frc::Translation2d kFrontLeftTranslation = frc::Translation2d{(kRobotLength / 2 - kModuleCornerOffset), (kRobotWidth / 2 - kModuleCornerOffset)};
  constexpr frc::Translation2d kFrontRightTranslation = frc::Translation2d{(kRobotLength / 2 - kModuleCornerOffset), -(kRobotWidth / 2 - kModuleCornerOffset)};
  constexpr frc::Translation2d kBackLeftTranslation = frc::Translation2d{-(kRobotLength / 2 - kModuleCornerOffset), (kRobotWidth / 2 - kModuleCornerOffset)};
  constexpr frc::Translation2d kBackRightTranslation = frc::Translation2d{-(kRobotLength / 2 - kModuleCornerOffset), -(kRobotWidth / 2 - kModuleCornerOffset)};

  constexpr units::meters_per_second_t kSpeedConstant = 1_mps;                              // Temporary value
  constexpr units::radians_per_second_t kSpeedConstant0 = std::numbers::pi * 0.5_rad_per_s; // Temporary value
}

namespace ModuleConstants
{
  constexpr double kDrivingMotorGearRatio = 4.71;                                       // 5.08 rotations of the motor for 1 rotation of the ouput
  constexpr units::volt_t kNominalVoltage = 12_V;                                       // The voltage at which the max speeds are mesured
  constexpr units::meter_t kWheelRadius = 1.5_in;                                       // The radius of REV's plastic wheels
  constexpr units::meter_t kWheelPerimeter = kWheelRadius * 2 * std::numbers::pi;       // in meters (diametre in inches * convertion to meters * pi)
  constexpr units::radians_per_second_t kTurningWheelFreeSpeedRadps = 24.260_rad_per_s; // TODO: Verify?
  constexpr units::meters_per_second_t kDriveWheelMaxSpeed = 4.9180_mps;                // TODO: Verify?

  constexpr double kDrivingFactor = ModuleConstants::kWheelPerimeter.value() / kDrivingMotorGearRatio;
  constexpr double kTurningFactor = 2 * std::numbers::pi;

  constexpr rev::spark::SparkLowLevel::ControlType kDrivingClosedLoopControlType = rev::spark::SparkLowLevel::ControlType::kVelocity;
  constexpr rev::spark::SparkLowLevel::ControlType kTurningClosedLoopControlType = rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl;

  constexpr rev::spark::SparkLowLevel::MotorType kDrivingMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
  constexpr rev::spark::SparkLowLevel::MotorType kTurningMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;

  constexpr rev::ResetMode kDrivingResetMode = rev::ResetMode::kResetSafeParameters;
  constexpr rev::ResetMode kTurningResetMode = rev::ResetMode::kResetSafeParameters;

  constexpr rev::PersistMode kDrivingPersistMode = rev::PersistMode::kPersistParameters;
  constexpr rev::PersistMode kTurningPersistMode = rev::PersistMode::kPersistParameters;

  constexpr double kTurningP = 0.3;
  constexpr double kTurningI = 0.0;
  constexpr double kTurningD = 0.0;
  constexpr double kDrivingP = 0.04;
  constexpr double kDrivingI = 0.0;
  constexpr double kDrivingD = 0.0;

  namespace Config
  {
    constexpr double kRPMtoRPSFactor = 60;

    constexpr units::radians_per_second_t kTurningCruiseVelocity = 2_rad_per_s * std::numbers::pi;
    constexpr units::radians_per_second_squared_t kTurningMaxAcceleration = 4_rad_per_s_sq * std::numbers::pi;

    constexpr rev::spark::SparkBaseConfig::IdleMode kDrivingIdleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;
    constexpr rev::spark::SparkBaseConfig::IdleMode kTurningIdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;

    constexpr rev::spark::FeedbackSensor kDrivingClosedLoopFeedbackSensor = rev::spark::FeedbackSensor::kPrimaryEncoder;
    constexpr rev::spark::FeedbackSensor kTurningClosedLoopFeedbackSensor = rev::spark::FeedbackSensor::kAbsoluteEncoder;

    constexpr bool kTurningMotorInverted = false;
    constexpr bool kTurningEncoderZeroCentered = false;
    constexpr bool kTurningClosedLoopPositionWrapping = true;
    constexpr double kTurningClosedLoopMinInput = -ModuleConstants::kTurningFactor / 2;
    constexpr double kTurningClosedLoopMaxInput = ModuleConstants::kTurningFactor / 2;
    constexpr double kTurningClosedLoopTolerance = 0.01 * ModuleConstants::kTurningFactor;
  }
}

namespace LimelightConstants
{
  constexpr bool kUseMegaTag2 = true;

  const std::string kName = "";

  constexpr units::meter_t kForward = -1.125_in;
  constexpr units::meter_t kRight = -9.75_in;
  constexpr units::meter_t kUp = 20.25_in;

  constexpr units::degree_t kRoll = 0_deg;
  constexpr units::degree_t kPitch = 0_deg;
  constexpr units::degree_t kYaw = -90_deg;

  constexpr double kPoseEstimatorStandardDeviationX = 0.7;      // Default/Recommended values
  constexpr double kPoseEstimatorStandardDeviationY = 0.7;      // Default/Recommended values
  constexpr double kPoseEstimatorStandardDeviationYaw = 999999; // Default/Recommended values
}

// Values found at https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf#page=3
// For the Welded Field
namespace FieldConstants
{
  constexpr frc::Translation2d kHubCenterTranslation2d = frc::Translation2d{182.11_in, 158.32_in};   // From the right corner of blue alliance wall
  constexpr frc::Pose2d kHubCenterPose2d = frc::Pose2d{kHubCenterTranslation2d, 0_rad};              // From the right corner of blue alliance wall
  constexpr frc::Translation2d kFieldCenterTranslation2d = frc::Translation2d{325.61_in, 158.32_in}; // From the right corner of blue alliance wall
  constexpr frc::Pose2d kFieldCenterPose2d = frc::Pose2d{kHubCenterTranslation2d, 0_rad};            // From the right corner of blue alliance wall
}

namespace CANid
{
  constexpr int kMotorIndexerID = 12;
  constexpr int kMotorFeederID = 13;

  constexpr int kLeaderMotorShooterID = 16;
  constexpr int kFollowerMotorShooterID = 17;

  constexpr int kBackRightMotorID = 8;
  constexpr int kBackRightMotor550ID = 7;
  constexpr int kFrontRightMotorID = 4;
  constexpr int kFrontRightMotor550ID = 3;
  constexpr int kFrontLeftMotorID = 6;
  constexpr int kFrontLeftMotor550ID = 5;
  constexpr int kBackLeftMotorID = 2;
  constexpr int kBackLeftMotor550ID = 1;

  constexpr int kMotorPivotID = 9;
  constexpr int kMotorIntakeID = 10;
  constexpr int kIMUPigeonID = 0;
}

namespace IntakeConstants
{
  constexpr bool kInverted = false;
  constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;

  constexpr double kSpeed = 0.5; // a modifier (valeur temporaire)
}

namespace PivotConstants
{
  constexpr double kGearRatio = 16;
  constexpr double kOffset = 5.66666;
  constexpr double kP = 1.3;  // en attendant
  constexpr double kI = 0.4;  // en attendant
  constexpr double kD = 0.15; // en attendant
  constexpr units::volt_t kG = 0.80_V;
  constexpr double setpointUp = std::numbers::pi / 2;    // 90 deg up
  constexpr double setpointDown = std::numbers::pi / 36; // 5 deg up

  constexpr bool kInverted = false;
  constexpr rev::ResetMode kReset = rev::ResetMode::kResetSafeParameters;
  constexpr rev::PersistMode kPersist = rev::PersistMode::kPersistParameters;
  constexpr rev::spark::SparkBaseConfig::IdleMode kIdleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;
}
