// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModuleSim.h"

#include <iostream>

SwerveModuleSim::SwerveModuleSim(int iDrivingMotorID, int iTurningMotorID, bool iDrivingInverted, bool iTurningInverted)
{
    
    mTurningGearBox = new frc::DCMotor{frc::DCMotor::NEO550()};
    mDrivingGearBox = new frc::DCMotor{frc::DCMotor::NEO()};
    
    mTurningMotor = new rev::spark::SparkMax{iTurningMotorID, SwerveConstants::kDrivingMotorType};
    mDrivingMotor = new rev::spark::SparkMax{iDrivingMotorID, SwerveConstants::kTurningMotorType};
    
    mTurningPID = new frc::PIDController{SwerveConstants::kP, SwerveConstants::kI, SwerveConstants::kD};
    mTurningPID->EnableContinuousInput(SwerveConstants::kTurningClosedLoopMinInput, SwerveConstants::kTurningClosedLoopMaxInput);
    
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

    refreshModule();
}

void SwerveModuleSim::setDesiredState(frc::SwerveModuleState iDesiredState, double iSpeedModulation)
{
    mTurningCurrentAngle = units::radian_t(mTurningAbsoluteEncoderSim->GetPosition());
    mOptimizedState = iDesiredState;
    mOptimizedState.Optimize(mTurningCurrentAngle);
    mOptimizedState.CosineScale(mTurningCurrentAngle);

    mTurningPID->SetSetpoint(mOptimizedState.angle.Radians().value());
    mTurningMotorSim->iterate(mTurningPID->Calculate(mTurningCurrentAngle.Radians().value()), 12, 0.02);
    mDrivingMotorSim->iterate(mOptimizedState.speed.value() * iSpeedModulation, 12, 0.02);
    
    mDrivingEncoderSim->SetPosition(mDrivingEncoderSim->GetPosition() + mDrivingEncoderSim->GetVelocity() * 0.02);
    mTurningAbsoluteEncoderSim->SetPosition(mTurningAbsoluteEncoderSim->GetPosition() + mDrivingEncoderSim->GetVelocity() * 0.02);
    // mDrivingEncoderSim->SetVelocity(mDrivingMotorSim->GetVelocity());
    // mTurningAbsoluteEncoderSim->SetPosition(mOptimizedState.angle.Radians().value());
    // mTurningAbsoluteEncoderSim->SetVelocity(mTurningMotorSim->GetVelocity());
}

void SwerveModuleSim::setPIDValues(double iP, double iI, double iD)
{
    mTurningPID->SetPID(iP, iI, iD);
    // bool wUpdateConfig = false;
    // if (iP != kP)
    // {
    //     mTurningConfig->closedLoop.P(iP);
    //     wUpdateConfig = true;
    //     kP = iP;
    // }
    // if (iI != kI)
    // {
    //     mTurningConfig->closedLoop.I(iI);
    //     wUpdateConfig = true;
    //     kI = iI;
    // }
    // if (iD != kD)
    // {
    //     mTurningConfig->closedLoop.D(iD);
    //     wUpdateConfig = true;
    //     kD = iD;
    // }
    // if (wUpdateConfig)
    // {
    //     mTurningMotor->Configure(*mTurningConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
    // }
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
    mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mDrivingEncoderSim->GetVelocity()),
                                           frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoderSim->GetPosition() - std::numbers::pi))};
    mModulePosition = frc::SwerveModulePosition{units::meter_t(mDrivingEncoderSim->GetPosition()),
                                                 frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoderSim->GetPosition() - std::numbers::pi))};
}