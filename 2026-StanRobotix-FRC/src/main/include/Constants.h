// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <rev/SparkBase.h> // Include Spark variable types
#include <rev/config/SparkBaseConfig.h> // For the spark IdleMode

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

namespace DrivetrainConstants {
    constexpr int kBackRightMotorID = 8;
    constexpr int kBackRightMotor550ID = 7;
    constexpr int kFrontRightMotorID = 4;
    constexpr int kFrontRightMotor550ID = 3;
    constexpr int kFrontLeftMotorID = 6;
    constexpr int kFrontLeftMotor550ID = 5;
    constexpr int kBackLeftMotorID = 2;
    constexpr int kBackLeftMotor550ID = 1;

    // Left-Right
    constexpr units::meter_t kRobotWidth = 28_in;
    // Front-Back
    constexpr units::meter_t kRobotLength = 26.875_in;
    // In both directions
    constexpr units::meter_t kModuleCornerOffset = 1.75_in;
    
    // We take for granted a rectangular frame 
    // TODO: Input the new offset for the frame
    constexpr frc::Translation2d kFrontLeftTranslation  = frc::Translation2d{ (kRobotLength / 2 - kModuleCornerOffset),  (kRobotWidth / 2 - kModuleCornerOffset)};
    constexpr frc::Translation2d kFrontRightTranslation = frc::Translation2d{ (kRobotLength / 2 - kModuleCornerOffset), -(kRobotWidth / 2 - kModuleCornerOffset)};
    constexpr frc::Translation2d kBackLeftTranslation   = frc::Translation2d{-(kRobotLength / 2 - kModuleCornerOffset),  (kRobotWidth / 2 - kModuleCornerOffset)};
    constexpr frc::Translation2d kBackRightTranslation  = frc::Translation2d{-(kRobotLength / 2 - kModuleCornerOffset), -(kRobotWidth / 2 - kModuleCornerOffset)};

    constexpr units::meters_per_second_t kSpeedConstant = 1_mps;                              // Temporary value
    constexpr units::radians_per_second_t kSpeedConstant0 = std::numbers::pi * 0.5_rad_per_s; // Temporary value

}

namespace ModuleConstants {
    constexpr double kDrivingMotorGearRatio = 5.08;                     // 5.08 rotations of the motor for 1 rotation of the ouput
    constexpr double kDriveWheelFreeSpeedRps = 40.0;                    // TODO: Mesure
    constexpr units::meter_t kWheelPerimeter = 3_in * std::numbers::pi; // in meters (diametre in inches * convertion to meters * pi)

    constexpr double kDrivingFactor = 1 / kDrivingMotorGearRatio;
    constexpr double kTurningFactor = 2 * std::numbers::pi;
    
    constexpr rev::spark::SparkLowLevel::ControlType kDrivingClosedLoopControlType = rev::spark::SparkLowLevel::ControlType::kVelocity;
    constexpr rev::spark::SparkLowLevel::ControlType kTurningClosedLoopControlType = rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl;

    constexpr rev::ResetMode kDrivingResetMode = rev::ResetMode::kResetSafeParameters;
    constexpr rev::ResetMode kTurningResetMode = rev::ResetMode::kResetSafeParameters;
    
    constexpr rev::spark::SparkLowLevel::MotorType kDrivingMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
    constexpr rev::spark::SparkLowLevel::MotorType kTurningMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
    
    constexpr rev::PersistMode kDrivingPersistMode = rev::PersistMode::kPersistParameters;
    constexpr rev::PersistMode kTurningPersistMode = rev::PersistMode::kPersistParameters;

    constexpr double kTurningP = 0.3;
    constexpr double kTurningI = 0.0;
    constexpr double kTurningD = 0.0;
    constexpr double kDrivingP = 0.04;
    constexpr double kDrivingI = 0.0;
    constexpr double kDrivingD = 0.0;
    
    constexpr double kRPMtoRPSFactor = 60;

    namespace Config {
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

namespace LimelightConstants {
    constexpr double kPoseEstimatorStandardDeviationX = 0.7;      // Default/Recommended values
    constexpr double kPoseEstimatorStandardDeviationY = 0.7;      // Default/Recommended values
    constexpr double kPoseEstimatorStandardDeviationYaw = 999999; // Default/Recommended values
}