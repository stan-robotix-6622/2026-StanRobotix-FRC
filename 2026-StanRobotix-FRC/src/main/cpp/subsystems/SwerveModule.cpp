// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int iNeoMotorID, int iNeo550MotorID, bool iNeoInverted, bool iNeo550Inverted)
{
    // Initialization of the motor controllers with the motorID constructor input
    mMotorNeo = new rev::spark::SparkMax{iNeoMotorID, SwerveConstants::kNeoMotorType};
    mMotorNeo550 = new rev::spark::SparkMax{iNeo550MotorID, SwerveConstants::kNeo550MotorType};

    // Initialization of the PIDController with the P,I and D constants and a continuous input from 0 to 2pi
    mNeo550PID = new frc::PIDController{SwerveConstants::kP, SwerveConstants::kI, SwerveConstants::kD};
    mNeo550PID->EnableContinuousInput(SwerveConstants::kNeo550ClosedLoopMinInput, SwerveConstants::kNeo550ClosedLoopMaxInput);

    mNeoConfig = new rev::spark::SparkMaxConfig{};
    mNeoConfig->Inverted(iNeoInverted);
    mNeoConfig->SetIdleMode(SwerveConstants::kNeoIdleMode);
    mNeoConfig->absoluteEncoder.VelocityConversionFactor(SwerveConstants::kNeoVelocityConversionFactor);
    mNeoConfig->absoluteEncoder.PositionConversionFactor(SwerveConstants::kNeoPositionConversionFactor);

    mNeo550Config = new rev::spark::SparkMaxConfig{};
    mNeo550Config->Inverted(iNeo550Inverted);
    mNeo550Config->SetIdleMode(SwerveConstants::kNeo550IdleMode);
    mNeo550Config->absoluteEncoder.ZeroCentered(true);
    mNeo550Config->absoluteEncoder.VelocityConversionFactor(SwerveConstants::kNeo550VelocityConversionFactor);
    mNeo550Config->absoluteEncoder.PositionConversionFactor(SwerveConstants::kNeo550PositionConversionFactor);
    mNeo550Config->closedLoop.SetFeedbackSensor(SwerveConstants::kNeo550ClosedLoopFeedbackSensor);
    mNeo550Config->closedLoop.Pid(SwerveConstants::kP, SwerveConstants::kI, SwerveConstants::kD);
    mNeo550Config->closedLoop.PositionWrappingEnabled(SwerveConstants::kNeo550ClosedLoopPositionWrapping);
    mNeo550Config->closedLoop.PositionWrappingInputRange(SwerveConstants::kNeo550ClosedLoopMinInput, SwerveConstants::kNeo550ClosedLoopMaxInput);
    // mNeo550Config->closedLoop.maxMotion.CruiseVelocity(0);
    // mNeo550Config->closedLoop.maxMotion.MaxAcceleration(0);
    mNeo550Config->closedLoop.maxMotion.AllowedProfileError(SwerveConstants::kNeo550ClosedLoopTolerance);

    mMotorNeo->Configure(*mNeoConfig, SwerveConstants::kNeoResetMode, SwerveConstants::kNeoPersistMode);
    mMotorNeo550->Configure(*mNeo550Config, SwerveConstants::kNeo550ResetMode, SwerveConstants::kNeo550PersistMode);
    
    mNeo550ClosedLoopController = new rev::spark::SparkClosedLoopController{mMotorNeo550->GetClosedLoopController()};

    // Initialization of the motor's encoders and absolute encoder
    mNeoEncoder = new rev::spark::SparkRelativeEncoder{mMotorNeo->GetEncoder()};
    mNeo550AbsoluteEncoder = new rev::spark::SparkAbsoluteEncoder{mMotorNeo550->GetAbsoluteEncoder()};

    // Initialization of the molule's SwerveModulePosition and SwerveModuleState from the encoder's velocity and position
    mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mNeoEncoder->GetVelocity() * DriveTrainConstants::kWheelPerimeter),
                                              frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - std::numbers::pi))};
    mModulePosition = frc::SwerveModulePosition{units::meter_t(mNeoEncoder->GetPosition() * DriveTrainConstants::kWheelPerimeter),
                                                    frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - std::numbers::pi))};
}

void SwerveModule::setDesiredState(frc::SwerveModuleState iDesiredState, double iSpeedModulation)
{
    mNeo550CurrentAngle = units::radian_t(mNeo550AbsoluteEncoder->GetPosition());
    mOptimizedState = iDesiredState;
    mOptimizedState.Optimize(mNeo550CurrentAngle);
    mOptimizedState.CosineScale(mNeo550CurrentAngle);

    mNeo550PID->SetSetpoint(mOptimizedState.angle.Radians().value());
    mMotorNeo550->Set(mNeo550PID->Calculate(mNeo550CurrentAngle.Radians().value()));
    mMotorNeo->Set(mOptimizedState.speed.value() * iSpeedModulation);
    // mNeo550ClosedLoopController->SetSetpoint(mOptimizedState.angle.Radians().value() * 10, SwerveConstants::kNeo550ClosedLoopControlType);
}

void SwerveModule::setPIDValues(double kP, double kI, double kD)
{
    if (mNeo550PID->GetP() != kP)
    {
        mNeo550PID->SetP(kP);
    }
    if (mNeo550PID->GetI() != kI)
    {
        mNeo550PID->SetI(kI);
    }
    if (mNeo550PID->GetD() != kD)
    {
        mNeo550PID->SetD(kD);
    }
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
                                           frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - std::numbers::pi))};
    mModulePosition = frc::SwerveModulePosition{units::meter_t(mNeoEncoder->GetPosition() * DriveTrainConstants::kWheelPerimeter),
                                                 frc::Rotation2d(units::radian_t(mNeo550AbsoluteEncoder->GetPosition() - std::numbers::pi))};
}