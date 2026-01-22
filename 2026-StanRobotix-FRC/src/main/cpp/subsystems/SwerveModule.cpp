// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int iNeoMotorID, int iNeo550MotorID, bool iNeoInverted)
{
    // Initialization of the motor controllers with the motorID constructor input
    mMotorNeo = new rev::spark::SparkMax{iNeoMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mMotorNeo550 = new rev::spark::SparkMax{iNeo550MotorID, rev::spark::SparkLowLevel::MotorType::kBrushless};

    // Initialization of the PIDController with the P,I and D constants and a continuous input from 0 to 2pi
    mNeo550PID = new frc::PIDController{SwerveModuleConstants::kP, SwerveModuleConstants::kI, SwerveModuleConstants::kD};
    mNeo550PID->EnableContinuousInput(0, 2 * std::numbers::pi);

    mNeoConfig = new rev::spark::SparkMaxConfig{};
    mNeoConfig->Inverted(iNeoInverted);
    mNeoConfig->SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
    mNeoConfig->absoluteEncoder.VelocityConversionFactor(1 / DriveTrainConstants::kGearRatio);
    mNeoConfig->absoluteEncoder.PositionConversionFactor(1 / DriveTrainConstants::kGearRatio);

    mNeo550Config = new rev::spark::SparkMaxConfig{};
    mNeo550Config->Inverted(false);
    mNeo550Config->SetIdleMode(rev::spark::SparkBaseConfig::kCoast);
    mNeo550Config->absoluteEncoder.VelocityConversionFactor(2 * std::numbers::pi);
    mNeo550Config->absoluteEncoder.PositionConversionFactor(2 * std::numbers::pi);

    mMotorNeo->Configure(*mNeoConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kNoPersistParameters);
    mMotorNeo550->Configure(*mNeo550Config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kNoPersistParameters);

    // Initialization of the motor's encoders and absolute encoder
    mNeoEncoder = new rev::spark::SparkRelativeEncoder{mMotorNeo->GetEncoder()};
    mNeo550AbsoluteEncoder = new rev::spark::SparkAbsoluteEncoder{mMotorNeo550->GetAbsoluteEncoder()};

    // Initialization of the molule's SwerveModulePosition and SwerveModuleState from the encoder's velocity and position
    mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mNeoEncoder->GetVelocity() * DriveTrainConstants::kWheelPerimeter),
                                              frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - 0.5))};
    mModulePosition = frc::SwerveModulePosition{units::meter_t(mNeoEncoder->GetPosition() * DriveTrainConstants::kWheelPerimeter),
                                                    frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - 0.5))};
}

void SwerveModule::setDesiredState(frc::SwerveModuleState iDesiredState, double SpeedModulation)
{
    mNeo550CurrentAngle = units::radian_t(mNeo550AbsoluteEncoder->GetPosition());
    mOptimizedState = iDesiredState;
    // mOptimizedState.Optimize(mNeo550CurrentAngle);
    // mOptimizedState.CosineScale(mNeo550CurrentAngle);

    mNeo550PID->SetSetpoint(mOptimizedState.angle.Radians().value());
    mMotorNeo550->Set(mNeo550PID->Calculate(mNeo550CurrentAngle.Radians().value()));
    // mMotorNeo->Set(mOptimizedState.speed.value());
}

void SwerveModule::setPIDValues(double kP, double kI, double kD)
{
    mNeo550PID->SetPID(kP, kI, kD);
}

frc::SwerveModuleState SwerveModule::getModuleState()
{
    return mModuleState;
}

frc::SwerveModulePosition SwerveModule::getModulePosition()
{
    return mModulePosition;
}

void SwerveModule::refreshModule()
{
    mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mNeoEncoder->GetVelocity() * DriveTrainConstants::kWheelPerimeter),
                                           frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - 0.5))};
    mModulePosition = frc::SwerveModulePosition{units::meter_t(mNeoEncoder->GetPosition() * DriveTrainConstants::kWheelPerimeter),
                                                 frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - 0.5))};
}