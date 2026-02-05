// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <rev/SparkBase.h> // Include Spark variable types
#include <rev/config/SparkBaseConfig.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {
    inline constexpr int kDriverControllerPort = 0;
} // namespace OperatorConstants

namespace IMUConstants {
    constexpr int kCanID = 0;
}

namespace PathPlannerConstants {
    constexpr double kPTranslation = 5.0;
    constexpr double kITranslation = 0.0;
    constexpr double kDTranslation = 0.0;
    constexpr double kPRotation = 5.0;
    constexpr double kIRotation = 0.0;
    constexpr double kDRotation = 0.0;

    constexpr double kPathPlannerSpeedModulation = 1.0;
}

namespace DriveTrainConstants {
    constexpr int kBackRightMotor550ID = 1;
    constexpr int kBackRightMotorID = 2;
    constexpr int kFrontRightMotor550ID = 3;
    constexpr int kFrontRightMotorID = 4;
    constexpr int kFrontLeftMotor550ID = 5;
    constexpr int kFrontLeftMotorID = 6;
    constexpr int kBackLeftMotor550ID = 7;
    constexpr int kBackLeftMotorID = 8;

    // We take for granted a rectangular frame 
    // TODO: Input the new offset for the frame
    constexpr units::meter_t kSwerveModuleOffsetFront = 0.3683_m;
    constexpr units::meter_t kSwerveModuleOffsetBack = -0.3683_m;
    constexpr units::meter_t kSwerveModuleOffsetRight = -0.3556_m;
    constexpr units::meter_t kSwerveModuleOffsetLeft = 0.3556_m;

    constexpr units::meters_per_second_t kSpeedConstant = 10_mps;                            // Temporary value
    constexpr units::radians_per_second_t kSpeedConstant0 = std::numbers::pi * 2_rad_per_s; // Temporary value

    constexpr double kSecToMinFactor = 60;                            // 60 seconds in a minute (to convert from RPM et rotation per second)
    constexpr double kDriveMotorGearRatio = 5.08;                     // 5.08 rotations of the motor for 1 rotation of the ouput
    constexpr double kWheelPerimeter = 3 * 0.0254 * std::numbers::pi; // in meters (diametre in inches * convertion to meters * pi)
}

namespace SwerveConstants {
    constexpr rev::spark::SparkLowLevel::MotorType kNeoMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
    constexpr rev::spark::SparkBaseConfig::IdleMode kNeoIdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;
    constexpr rev::ResetMode kNeoResetMode = rev::ResetMode::kResetSafeParameters;
    constexpr rev::PersistMode kNeoPersistMode = rev::PersistMode::kPersistParameters;
    
    constexpr rev::spark::SparkLowLevel::MotorType kNeo550MotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
    constexpr rev::spark::SparkBaseConfig::IdleMode kNeo550IdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;
    constexpr rev::ResetMode kNeo550ResetMode = rev::ResetMode::kResetSafeParameters;
    constexpr rev::PersistMode kNeo550PersistMode = rev::PersistMode::kPersistParameters;

    constexpr bool kNeo550AbsoluteEncoderZeroCentered = true;
    
    constexpr double kNeo550VelocityConversionFactor = 2 * std::numbers::pi;
    constexpr double kNeo550PositionConversionFactor = 2 * std::numbers::pi;
    constexpr double kNeoVelocityConversionFactor = 1 / DriveTrainConstants::kDriveMotorGearRatio;
    constexpr double kNeoPositionConversionFactor = 1 / DriveTrainConstants::kDriveMotorGearRatio;
    
    constexpr rev::spark::SparkLowLevel::ControlType kNeo550ClosedLoopControlType = rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl;
    constexpr rev::spark::FeedbackSensor kNeo550ClosedLoopFeedbackSensor = rev::spark::FeedbackSensor::kAbsoluteEncoder;
    constexpr bool kNeo550ClosedLoopPositionWrapping = true;
    constexpr double kNeo550ClosedLoopMinInput = -kNeo550PositionConversionFactor / 2;
    constexpr double kNeo550ClosedLoopMaxInput = kNeo550PositionConversionFactor / 2;
    constexpr double kNeo550ClosedLoopTolerance =  0.01 * kNeo550PositionConversionFactor;

    constexpr units::volt_t kV = 12_V / 40;

    constexpr double kP = 0.3;
    constexpr double kI = 0.00;
    constexpr double kD = 0.00;
}

namespace LimelightConstants {
    constexpr double kPoseEstimatorStandardDeviationX = 0.7;      // Default/Recommended values
    constexpr double kPoseEstimatorStandardDeviationY = 0.7;      // Default/Recommended values
    constexpr double kPoseEstimatorStandardDeviationYaw = 999999; // Default/Recommended values
}