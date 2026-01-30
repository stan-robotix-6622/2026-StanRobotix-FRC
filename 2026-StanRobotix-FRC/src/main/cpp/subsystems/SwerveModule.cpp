// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int iDrivingMotorID, int iTurningMotorID, bool iDrivingInverted, bool iTurningInverted)
{
    // Initialization of the motor controllers with the motorID constructor input
    mDrivingMotor = new rev::spark::SparkMax{iDrivingMotorID, SwerveConstants::kDrivingMotorType};
    mTurningMotor = new rev::spark::SparkMax{iTurningMotorID, SwerveConstants::kTurningMotorType};

    // Initialization of the PIDController with the P,I and D constants and a continuous input from 0 to 2pi
    mTurningPID = new frc::PIDController{SwerveConstants::kP, SwerveConstants::kI, SwerveConstants::kD};
    mTurningPID->EnableContinuousInput(0, 2 * std::numbers::pi);

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

    // Initialization of the motor's encoders and absolute encoder
    mDrivingEncoder = new rev::spark::SparkRelativeEncoder{mDrivingMotor->GetEncoder()};
    mTurningAbsoluteEncoder = new rev::spark::SparkAbsoluteEncoder{mTurningMotor->GetAbsoluteEncoder()};

    // Initialization of the molule's SwerveModulePosition and SwerveModuleState from the encoder's velocity and position
    mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mDrivingEncoder->GetVelocity() * DriveTrainConstants::kWheelPerimeter),
                                              frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoder->GetPosition() - std::numbers::pi))};
    mModulePosition = frc::SwerveModulePosition{units::meter_t(mDrivingEncoder->GetPosition() * DriveTrainConstants::kWheelPerimeter),
                                                    frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoder->GetPosition() - std::numbers::pi))};
}

void SwerveModule::setDesiredState(frc::SwerveModuleState iDesiredState, double iSpeedModulation)
{
    mTurningCurrentAngle = units::radian_t(mTurningAbsoluteEncoder->GetPosition());
    mOptimizedState = iDesiredState;
    // mOptimizedState.Optimize(mTurningCurrentAngle);
    // mOptimizedState.CosineScale(mTurningCurrentAngle);

    // mTurningPID->SetSetpoint(mOptimizedState.angle.Radians().value());
    // mMotorTurning->Set(mTurningPID->Calculate(mTurningCurrentAngle.Radians().value()));
    mTurningClosedLoopController->SetSetpoint(mOptimizedState.angle.Radians().value(), SwerveConstants::kTurningClosedLoopControlType);
    // mMotorDriving->Set(mOptimizedState.speed.value() * iSpeedModulation);
}

void SwerveModule::setPIDValues(double kP, double kI, double kD)
{
    mTurningPID->SetPID(kP, kI, kD);
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
    mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mDrivingEncoder->GetVelocity() * DriveTrainConstants::kWheelPerimeter),
                                           frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoder->GetPosition() - std::numbers::pi))};
    mModulePosition = frc::SwerveModulePosition{units::meter_t(mDrivingEncoder->GetPosition() * DriveTrainConstants::kWheelPerimeter),
                                                 frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoder->GetPosition() - std::numbers::pi))};
}