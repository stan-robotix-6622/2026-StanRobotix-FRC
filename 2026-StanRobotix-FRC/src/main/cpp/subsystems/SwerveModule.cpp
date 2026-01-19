// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int iNeoMotorID, int iNeo550MotorID, bool iInverted)
{
    // Initialization of the motor controllers with the motorID constructor input
    mMotorNeo = new rev::spark::SparkMax{iNeoMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mMotorNeo550 = new rev::spark::SparkMax{iNeo550MotorID, rev::spark::SparkLowLevel::MotorType::kBrushless};

    // Initialization of the PIDController with the P,I and D constants and a continuous input from 0 to 1
    mNeo550PID = new frc::PIDController{SwerveModuleConstants::kP, SwerveModuleConstants::kI, SwerveModuleConstants::kD};
    mNeo550PID->EnableContinuousInput(0, 1);

    setNeoInverted(iInverted);

    // Initialization of the motor's encoders and absolute encoder
    mNeoEncoder = new rev::spark::SparkRelativeEncoder{mMotorNeo->GetEncoder()};
    mNeo550AbsoluteEncoder = new rev::spark::SparkAbsoluteEncoder{mMotorNeo550->GetAbsoluteEncoder()};

    // Initialization of the molule's SwerveModulePosition and SwerveModuleState from the encoder's velocity and position
    mModuleState = new frc::SwerveModuleState{units::meters_per_second_t(mNeoEncoder->GetVelocity() * DriveTrainConstants::kGearRatio * DriveTrainConstants::kWheelPerimeter),
                                               frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - 0.5) * 2 * std::numbers::pi)};
    mModulePosition = new frc::SwerveModulePosition{units::meter_t(mNeoEncoder->GetPosition() * DriveTrainConstants::kGearRatio * DriveTrainConstants::kWheelPerimeter),
                                                     frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - 0.5) * 2 * std::numbers::pi)};
}

frc::SwerveModuleState SwerveModule::optimizeState(frc::SwerveModuleState iDesiredState)
{
    frc::SwerveModuleState OptimizedState = iDesiredState;
    frc::Rotation2d Neo550CurrentAngle(units::degree_t(mNeo550AbsoluteEncoder->GetPosition() - 0.5) * 360);
    OptimizedState.Optimize(Neo550CurrentAngle);
    OptimizedState.CosineScale(Neo550CurrentAngle);
    return OptimizedState;
}

void SwerveModule::setDesiredState(frc::SwerveModuleState iDesiredState, double SpeedModulation)
{
    frc::SwerveModuleState OptimizedState = optimizeState(iDesiredState);
    mNeo550PID->SetSetpoint(double(OptimizedState.angle.Radians() / (2 * std::numbers::pi)) + 0.5);
    mMotorNeo550->Set(mNeo550PID->Calculate(mNeo550AbsoluteEncoder->GetPosition()));
    mMotorNeo->Set(double(OptimizedState.speed * SpeedModulation));
    // std::cout << mNeo550PID->Calculate(mNeo550AbsoluteEncoder->GetPosition()) << std::endl;
}

void SwerveModule::setNeoInverted(bool iInverted)
{
    mMotorNeo->SetInverted(iInverted);
}

frc::SwerveModuleState * SwerveModule::getModuleState()
{
    return mModuleState;
}

frc::SwerveModulePosition * SwerveModule::getModulePosition()
{
    return mModulePosition;
}

void SwerveModule::refreshModule()
{
    *mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mNeoEncoder->GetVelocity() * DriveTrainConstants::kGearRatio * DriveTrainConstants::kWheelPerimeter),
                                            frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - 0.5) * 2 * std::numbers::pi)};
    *mModulePosition = frc::SwerveModulePosition{units::meter_t(mNeoEncoder->GetPosition() * DriveTrainConstants::kGearRatio * DriveTrainConstants::kWheelPerimeter),
                                                  frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - 0.5) * 2 * std::numbers::pi)};
}