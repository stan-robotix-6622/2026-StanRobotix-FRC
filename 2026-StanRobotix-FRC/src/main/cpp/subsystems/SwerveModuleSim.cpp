// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModuleSim.h"

#include <iostream>

SwerveModuleSim::SwerveModuleSim(int iNeoMotorID, int iNeo550MotorID, bool iNeoInverted, bool iNeo550Inverted)
{
    
    mTurningGearBox = new frc::DCMotor{frc::DCMotor::NEO550()};
    mDriveGearBox = new frc::DCMotor{frc::DCMotor::NEO()};
    
    mTurningMotor = new rev::spark::SparkMax{1, SwerveConstants::kNeoMotorType};
    mDriveMotor = new rev::spark::SparkMax{2, SwerveConstants::kNeo550MotorType};

    mTurningMotorSim = new rev::spark::SparkMaxSim{mTurningMotor, mTurningGearBox};
    mDriveMotorSim = new rev::spark::SparkMaxSim{mDriveMotor, mDriveGearBox};
    
    mNeoConfig = new rev::spark::SparkMaxConfig{};
    mNeoConfig->Inverted(iNeoInverted);
    mNeoConfig->SetIdleMode(SwerveConstants::kNeoIdleMode);
    mNeoConfig->absoluteEncoder.VelocityConversionFactor(SwerveConstants::kNeoVelocityConversionFactor);
    mNeoConfig->absoluteEncoder.PositionConversionFactor(SwerveConstants::kNeoPositionConversionFactor);

    mNeo550Config = new rev::spark::SparkMaxConfig{};
    mNeo550Config->Inverted(iNeo550Inverted);
    mNeo550Config->SetIdleMode(SwerveConstants::kNeo550IdleMode);
    mNeo550Config->absoluteEncoder.VelocityConversionFactor(SwerveConstants::kNeo550VelocityConversionFactor);
    mNeo550Config->absoluteEncoder.PositionConversionFactor(SwerveConstants::kNeo550PositionConversionFactor);
    mNeo550Config->closedLoop.SetFeedbackSensor(SwerveConstants::kNeo550ClosedLoopFeedbackSensor);
    mNeo550Config->closedLoop.Pid(SwerveConstants::kP, SwerveConstants::kI, SwerveConstants::kD);
    mNeo550Config->closedLoop.PositionWrappingEnabled(SwerveConstants::kNeo550ClosedLoopPositionWrapping);
    mNeo550Config->closedLoop.PositionWrappingInputRange(SwerveConstants::kNeo550ClosedLoopMinInput, SwerveConstants::kNeo550ClosedLoopMaxInput);
    mNeo550Config->closedLoop.maxMotion.AllowedClosedLoopError(SwerveConstants::kNeo550ClosedLoopTolerance);

    mDriveMotor->Configure(*mNeoConfig, SwerveConstants::kNeoResetMode, SwerveConstants::kNeoPersistMode);
    mTurningMotor->Configure(*mNeo550Config, SwerveConstants::kNeo550ResetMode, SwerveConstants::kNeo550PersistMode);

    mNeoEncoderSim = new rev::spark::SparkRelativeEncoderSim{mDriveMotorSim->GetRelativeEncoderSim()};
    mNeo550AbsoluteEncoderSim = new rev::spark::SparkAbsoluteEncoderSim{mTurningMotorSim->GetAbsoluteEncoderSim()};

    mNeo550ClosedLoopController = new rev::spark::SparkClosedLoopController{mTurningMotor->GetClosedLoopController()};
}

void SwerveModuleSim::setDesiredState(frc::SwerveModuleState iDesiredState, double iSpeedModulation)
{
    mNeo550CurrentAngle = units::radian_t(mNeo550AbsoluteEncoderSim->GetPosition());
    mOptimizedState = iDesiredState;
    mOptimizedState.Optimize(mNeo550CurrentAngle);
    mOptimizedState.CosineScale(mNeo550CurrentAngle);

    mNeo550ClosedLoopController->SetSetpoint(mOptimizedState.angle.Radians().value(), SwerveConstants::kNeo550ClosedLoopControlType);
    mDriveMotorSim->SetVelocity(mOptimizedState.speed.value() * iSpeedModulation);
}

void SwerveModuleSim::setPIDValues(double iP, double iI, double iD)
{
    bool wUpdateConfig = false;
    if (iP != kP)
    {
        mNeo550Config->closedLoop.P(iP);
        wUpdateConfig = true;
        kP = iP;
    }
    if (iI != kI)
    {
        mNeo550Config->closedLoop.I(iI);
        wUpdateConfig = true;
        kI = iI;
    }
    if (iD != kD)
    {
        mNeo550Config->closedLoop.D(iD);
        wUpdateConfig = true;
        kD = iD;
    }
    if (wUpdateConfig)
    {
        mTurningMotor->Configure(*mNeo550Config, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
    }
}

frc::SwerveModuleState SwerveModuleSim::getModuleState()
{
    return mModuleState;
}

frc::SwerveModulePosition SwerveModuleSim::getModulePosition()
{
    return mModulePosition;
}

void SwerveModuleSim::refreshModule()
{
    mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mNeoEncoderSim->GetVelocity() * DriveTrainConstants::kWheelPerimeter),
                                           frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoderSim->GetPosition() - std::numbers::pi))};
    mModulePosition = frc::SwerveModulePosition{units::meter_t(mNeoEncoderSim->GetPosition() * DriveTrainConstants::kWheelPerimeter),
                                                 frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoderSim->GetPosition() - std::numbers::pi))};
}