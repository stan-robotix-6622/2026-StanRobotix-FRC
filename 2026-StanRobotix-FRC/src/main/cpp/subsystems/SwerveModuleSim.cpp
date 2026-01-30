// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModuleSim.h"

#include <iostream>

SwerveModuleSim::SwerveModuleSim(int iDrivingMotorID, int iTurningMotorID, bool iDrivingInverted, bool iTurningInverted)
{
    
    mTurningGearBox = new frc::DCMotor{frc::DCMotor::Turning()};
    mDrivingGearBox = new frc::DCMotor{frc::DCMotor::Driving()};
    
    mTurningMotor = new rev::spark::SparkMax{iTurningMotorID, SwerveConstants::kDrivingMotorType};
    mDrivingMotor = new rev::spark::SparkMax{iDrivingMotorID, SwerveConstants::kTurningMotorType};
    
    mDrivingConfig = new rev::spark::SparkMaxConfig{};
    mDrivingConfig->Inverted(iDrivingInverted);
    mDrivingConfig->SetIdleMode(SwerveConstants::kDrivingIdleMode);
    mDrivingConfig->absoluteEncoder.VelocityConversionFactor(SwerveConstants::kDrivingVelocityConversionFactor);
    mDrivingConfig->absoluteEncoder.PositionConversionFactor(SwerveConstants::kDrivingPositionConversionFactor);

    mTurningConfig = new rev::spark::SparkMaxConfig{};
    mTurningConfig->Inverted(iTurningInverted);
    mTurningConfig->SetIdleMode(SwerveConstants::kTurningIdleMode);
    mTurningConfig->absoluteEncoder.VelocityConversionFactor(SwerveConstants::kTurningVelocityConversionFactor);
    mTurningConfig->absoluteEncoder.PositionConversionFactor(SwerveConstants::kTurningPositionConversionFactor);
    mTurningConfig->closedLoop.SetFeedbackSensor(SwerveConstants::kTurningClosedLoopFeedbackSensor);
    mTurningConfig->closedLoop.Pid(SwerveConstants::kP, SwerveConstants::kI, SwerveConstants::kD);
    mTurningConfig->closedLoop.PositionWrappingEnabled(SwerveConstants::kTurningClosedLoopPositionWrapping);
    mTurningConfig->closedLoop.PositionWrappingInputRange(SwerveConstants::kTurningClosedLoopMinInput, SwerveConstants::kTurningClosedLoopMaxInput);
    mTurningConfig->closedLoop.maxMotion.AllowedClosedLoopError(SwerveConstants::kTurningClosedLoopTolerance);

    mDrivingMotor->Configure(*mDrivingConfig, SwerveConstants::kDrivingResetMode, SwerveConstants::kDrivingPersistMode);
    mTurningMotor->Configure(*mTurningConfig, SwerveConstants::kTurningResetMode, SwerveConstants::kTurningPersistMode);

    mTurningClosedLoopController = new rev::spark::SparkClosedLoopController{mTurningMotor->GetClosedLoopController()};
    
    mTurningMotorSim = new rev::spark::SparkMaxSim{mTurningMotor, mTurningGearBox};
    mDrivingMotorSim = new rev::spark::SparkMaxSim{mDrivingMotor, mDrivingGearBox};

    mDrivingEncoderSim = new rev::spark::SparkRelativeEncoderSim{mDrivingMotorSim->GetRelativeEncoderSim()};
    mTurningAbsoluteEncoderSim = new rev::spark::SparkAbsoluteEncoderSim{mTurningMotorSim->GetAbsoluteEncoderSim()};
}

void SwerveModuleSim::setDesiredState(frc::SwerveModuleState iDesiredState, double iSpeedModulation)
{
    mTurningCurrentAngle = units::radian_t(mTurningAbsoluteEncoderSim->GetPosition());
    mOptimizedState = iDesiredState;
    mOptimizedState.Optimize(mTurningCurrentAngle);
    mOptimizedState.CosineScale(mTurningCurrentAngle);

    mTurningClosedLoopController->SetSetpoint(mOptimizedState.angle.Radians().value(), SwerveConstants::kTurningClosedLoopControlType);
    mDrivingMotorSim->SetVelocity(mOptimizedState.speed.value() * iSpeedModulation);

    mDrivingEncoderSim->SetVelocity(mOptimizedState.speed.value() * iSpeedModulation);
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
    mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mDrivingEncoderSim->GetVelocity() * DriveTrainConstants::kWheelPerimeter),
                                           frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoderSim->GetPosition() - std::numbers::pi))};
    mModulePosition = frc::SwerveModulePosition{units::meter_t(mDrivingEncoderSim->GetPosition() * DriveTrainConstants::kWheelPerimeter),
                                                 frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoderSim->GetPosition() - std::numbers::pi))};
}