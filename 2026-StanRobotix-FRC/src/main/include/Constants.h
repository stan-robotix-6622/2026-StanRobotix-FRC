// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <numbers>

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
    inline constexpr int kDrivingrControllerPort = 0;
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
    constexpr int kBackRightMotorID = 6;
    constexpr int kBackRightMotor550ID = 5;
    constexpr int kFrontRightMotorID = 8;
    constexpr int kFrontRightMotor550ID = 7;
    constexpr int kFrontLeftMotorID = 2;
    constexpr int kFrontLeftMotor550ID = 1;
    constexpr int kBackLeftMotorID = 4;
    constexpr int kBackLeftMotor550ID = 3;

    // We take for granted a rectangular frame 
    // TODO: Input the new offset for the frame
    constexpr units::meter_t kSwerveModuleOffsetFront = 0.3683_m;
    constexpr units::meter_t kSwerveModuleOffsetBack = -0.3683_m;
    constexpr units::meter_t kSwerveModuleOffsetRight = -0.3556_m;
    constexpr units::meter_t kSwerveModuleOffsetLeft = 0.3556_m;

    constexpr units::meters_per_second_t kSpeedConstant = 5_mps;                            // Temporary value
    constexpr units::radians_per_second_t kSpeedConstant0 = std::numbers::pi * 0.2_rad_per_s; // Temporary value

}

namespace ModuleConstants {
    constexpr double kDrivingMotorGearRatio = 5.08;                     // 5.08 rotations of the motor for 1 rotation of the ouput
    constexpr double kDriveWheelFreeSpeedRps = 40; // TODO: Mesure
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

    constexpr double kTurningP = 0.048;
    constexpr double kTurningI = 0.0016;
    constexpr double kTurningD = 0.0008;
    constexpr double kDrivingP = 0.04;
    constexpr double kDrivingI = 0.0;
    constexpr double kDrivingD = 0.0;
}

namespace LimelightConstants {
    constexpr double kPoseEstimatorStandardDeviationX = 0.7;      // Default/Recommended values
    constexpr double kPoseEstimatorStandardDeviationY = 0.7;      // Default/Recommended values
    constexpr double kPoseEstimatorStandardDeviationYaw = 999999; // Default/Recommended values
}

namespace ConfigConstants {
    namespace Swerve {
        constexpr rev::spark::SparkBaseConfig::IdleMode kDrivingIdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;
        constexpr rev::spark::SparkBaseConfig::IdleMode kTurningIdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;
        
        constexpr rev::spark::FeedbackSensor kDrivingClosedLoopFeedbackSensor = rev::spark::FeedbackSensor::kPrimaryEncoder;
        constexpr rev::spark::FeedbackSensor kTurningClosedLoopFeedbackSensor = rev::spark::FeedbackSensor::kAbsoluteEncoder;

        constexpr bool kTurningMotorInverted = false;
        constexpr bool kTurningClosedLoopPositionWrapping = true;
        constexpr double kTurningClosedLoopMinInput = 0;
        constexpr double kTurningClosedLoopMaxInput = ModuleConstants::kTurningFactor;
        constexpr double kTurningClosedLoopTolerance = 0.01 * ModuleConstants::kTurningFactor;
        
        constexpr double kRPMtoRPSFactor = 60;
    }
}