// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int iDrivingMotorID, int iTurningMotorID, bool iDrivingInverted, bool iTurningInverted)
{
    // Initialization of the motor controllers with the motorID constructor input
    mDrivingMotor = new rev::spark::SparkMax{iDrivingMotorID, ModuleConstants::kDrivingMotorType};
    mTurningMotor = new rev::spark::SparkMax{iTurningMotorID, ModuleConstants::kTurningMotorType};

    // Initialization of the PIDController with the P,I and D constants and 
    // a continuous input from 0 to 2pi
    mTurningPID = new frc::PIDController{ModuleConstants::kTurningP,
                                         ModuleConstants::kTurningI,
                                         ModuleConstants::kTurningD};
    mTurningPID->EnableContinuousInput(0, 2 * std::numbers::pi);

    // Configure the motors from Configs.h
    mDrivingMotor->Configure(Configs::SwerveModule::DrivingConfig(iDrivingInverted),
                             ModuleConstants::kDrivingResetMode,
                             ModuleConstants::kDrivingPersistMode);
    mTurningMotor->Configure(Configs::SwerveModule::DrivingConfig(iTurningInverted),
                             ModuleConstants::kTurningResetMode,
                             ModuleConstants::kTurningPersistMode);
    
    // Initialization of the motors' ClosedLoopController
    mTurningClosedLoopController = new rev::spark::SparkClosedLoopController{mTurningMotor->GetClosedLoopController()};
    mDrivingClosedLoopController = new rev::spark::SparkClosedLoopController{mDrivingMotor->GetClosedLoopController()};

    // Initialization of the motor's encoders and absolute encoder
    mDrivingEncoder = new rev::spark::SparkRelativeEncoder{mDrivingMotor->GetEncoder()};
    mTurningAbsoluteEncoder = new rev::spark::SparkAbsoluteEncoder{mTurningMotor->GetAbsoluteEncoder()};

    // Initialization of the molule's SwerveModulePosition and SwerveModuleState from the encoder's velocity and position
    refreshModule();
}

void SwerveModule::setDesiredState(frc::SwerveModuleState iDesiredState, double iSpeedModulation)
{
    mTurningCurrentAngle = units::radian_t(mTurningAbsoluteEncoder->GetPosition());
    mOptimizedState = iDesiredState;
    // mOptimizedState.Optimize(mTurningCurrentAngle);
    // mOptimizedState.CosineScale(mTurningCurrentAngle);

    // mTurningPID->SetSetpoint(mOptimizedState.angle.Radians().value());
    // mMotorTurning->Set(mTurningPID->Calculate(mTurningCurrentAngle.Radians().value()));
    mTurningClosedLoopController->SetSetpoint(mOptimizedState.angle.Radians().value(), ModuleConstants::kTurningClosedLoopControlType);
    // mMotorDriving->Set(mOptimizedState.speed.value() * iSpeedModulation);
}

void SwerveModule::setPIDValues(double kP, double kI, double kD)
{
    mTurningPID->SetPID(kP, kI, kD);
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
    mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mDrivingEncoder->GetVelocity()),
                                           frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoder->GetPosition() - std::numbers::pi))};
    mModulePosition = frc::SwerveModulePosition{units::meter_t(mDrivingEncoder->GetPosition()),
                                                 frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoder->GetPosition() - std::numbers::pi))};
}