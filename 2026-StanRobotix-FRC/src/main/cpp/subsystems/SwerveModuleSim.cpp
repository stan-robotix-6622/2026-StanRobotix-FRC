// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModuleSim.h"

#include <iostream>

SwerveModuleSim::SwerveModuleSim(int iNeoMotorID, int iNeo550MotorID, bool iNeoInverted, bool iNeo550Inverted)
{
    
    mTurningGearBox = new frc::DCMotor{frc::DCMotor::NEO550()};
    mDriveGearBox = new frc::DCMotor{frc::DCMotor::NEO()};
    
    mTurningMotor = new rev::spark::SparkMax{iNeo550MotorID, SwerveConstants::kNeoMotorType};
    mDriveMotor = new rev::spark::SparkMax{iNeoMotorID, SwerveConstants::kNeo550MotorType};
    
    mDriveConfig = new rev::spark::SparkMaxConfig{};
    mDriveConfig->Inverted(iNeoInverted);
    mDriveConfig->SetIdleMode(SwerveConstants::kNeoIdleMode);
    mDriveConfig->absoluteEncoder.VelocityConversionFactor(SwerveConstants::kNeoVelocityConversionFactor);
    mDriveConfig->absoluteEncoder.PositionConversionFactor(SwerveConstants::kNeoPositionConversionFactor);

    mTurningConfig = new rev::spark::SparkMaxConfig{};
    mTurningConfig->Inverted(iNeo550Inverted);
    mTurningConfig->SetIdleMode(SwerveConstants::kNeo550IdleMode);
    mTurningConfig->absoluteEncoder.VelocityConversionFactor(SwerveConstants::kNeo550VelocityConversionFactor);
    mTurningConfig->absoluteEncoder.PositionConversionFactor(SwerveConstants::kNeo550PositionConversionFactor);
    mTurningConfig->closedLoop.SetFeedbackSensor(SwerveConstants::kNeo550ClosedLoopFeedbackSensor);
    mTurningConfig->closedLoop.Pid(SwerveConstants::kP, SwerveConstants::kI, SwerveConstants::kD);
    mTurningConfig->closedLoop.PositionWrappingEnabled(SwerveConstants::kNeo550ClosedLoopPositionWrapping);
    mTurningConfig->closedLoop.PositionWrappingInputRange(SwerveConstants::kNeo550ClosedLoopMinInput, SwerveConstants::kNeo550ClosedLoopMaxInput);
    mTurningConfig->closedLoop.maxMotion.AllowedClosedLoopError(SwerveConstants::kNeo550ClosedLoopTolerance);

    mDriveMotor->Configure(*mDriveConfig, SwerveConstants::kNeoResetMode, SwerveConstants::kNeoPersistMode);
    mTurningMotor->Configure(*mTurningConfig, SwerveConstants::kNeo550ResetMode, SwerveConstants::kNeo550PersistMode);

    mTurningClosedLoopController = new rev::spark::SparkClosedLoopController{mTurningMotor->GetClosedLoopController()};
    
    mTurningMotorSim = new rev::spark::SparkMaxSim{mTurningMotor, mTurningGearBox};
    mDriveMotorSim = new rev::spark::SparkMaxSim{mDriveMotor, mDriveGearBox};

    mDriveEncoderSim = new rev::spark::SparkRelativeEncoderSim{mDriveMotorSim->GetRelativeEncoderSim()};
    mTurningAbsoluteEncoderSim = new rev::spark::SparkAbsoluteEncoderSim{mTurningMotorSim->GetAbsoluteEncoderSim()};
}

void SwerveModuleSim::setDesiredState(frc::SwerveModuleState iDesiredState, double iSpeedModulation)
{
    mTurningCurrentAngle = units::radian_t(mTurningAbsoluteEncoderSim->GetPosition());
    mOptimizedState = iDesiredState;
    mOptimizedState.Optimize(mTurningCurrentAngle);
    mOptimizedState.CosineScale(mTurningCurrentAngle);

    mTurningClosedLoopController->SetSetpoint(mOptimizedState.angle.Radians().value(), SwerveConstants::kNeo550ClosedLoopControlType);
    mDriveMotorSim->SetVelocity(mOptimizedState.speed.value() * iSpeedModulation);

    mDriveEncoderSim->SetVelocity(mOptimizedState.speed.value() * iSpeedModulation);
    mTurningAbsoluteEncoderSim->SetVelocity(mTurningMotorSim->GetVelocity());
}

void SwerveModuleSim::setPIDValues(double iP, double iI, double iD)
{
    bool wUpdateConfig = false;
    if (iP != kP)
    {
        mTurningConfig->closedLoop.P(iP);
        wUpdateConfig = true;
        kP = iP;
    }
    if (iI != kI)
    {
        mTurningConfig->closedLoop.I(iI);
        wUpdateConfig = true;
        kI = iI;
    }
    if (iD != kD)
    {
        mTurningConfig->closedLoop.D(iD);
        wUpdateConfig = true;
        kD = iD;
    }
    if (wUpdateConfig)
    {
        mTurningMotor->Configure(*mTurningConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
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
    mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mDriveEncoderSim->GetVelocity() * DriveTrainConstants::kWheelPerimeter),
                                           frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoderSim->GetPosition() - std::numbers::pi))};
    mModulePosition = frc::SwerveModulePosition{units::meter_t(mDriveEncoderSim->GetPosition() * DriveTrainConstants::kWheelPerimeter),
                                                 frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoderSim->GetPosition() - std::numbers::pi))};
}