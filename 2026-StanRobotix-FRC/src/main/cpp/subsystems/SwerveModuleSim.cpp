// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModuleSim.h"

#include <iostream>

SwerveModuleSim::SwerveModuleSim(int iDrivingMotorID, int iTurningMotorID, bool iDrivingInverted, bool iTurningInverted)
{
    // Initialization of the motor's GearBox
    mTurningGearBox = new frc::DCMotor{frc::DCMotor::NEO550()};
    mDrivingGearBox = new frc::DCMotor{frc::DCMotor::NEO()};
    
    // Initialization of the 'real' motor
    mTurningMotor = new rev::spark::SparkMax{iTurningMotorID, ModuleConstants::kDrivingMotorType};
    mDrivingMotor = new rev::spark::SparkMax{iDrivingMotorID, ModuleConstants::kTurningMotorType};
    
    // Initialization of the PIDController with the P,I and D constants and 
    // a continuous input from 0 to 2pi
    mTurningPID = new frc::PIDController{ModuleConstants::kTurningP,
                                         ModuleConstants::kTurningI,
                                         ModuleConstants::kTurningD};
    mTurningPID->EnableContinuousInput(ModuleConstants::Config::kTurningClosedLoopMinInput, ModuleConstants::Config::kTurningClosedLoopMaxInput);
    
    // Configure the motors from Configs.h
    mDrivingMotor->Configure(Configs::SwerveModule::DrivingConfig(iDrivingInverted),
                             ModuleConstants::kDrivingResetMode,
                             ModuleConstants::kDrivingPersistMode);
    mTurningMotor->Configure(Configs::SwerveModule::TurningConfig(iTurningInverted),
                             ModuleConstants::kTurningResetMode,
                             ModuleConstants::kTurningPersistMode);

    // Initialization of the motors' ClosedLoopController
    mTurningClosedLoopController = new rev::spark::SparkClosedLoopController{mTurningMotor->GetClosedLoopController()};
    mDrivingClosedLoopController = new rev::spark::SparkClosedLoopController{mDrivingMotor->GetClosedLoopController()};
    
    // Initialization of the SparkMaxSims
    mTurningMotorSim = new rev::spark::SparkMaxSim{mTurningMotor, mTurningGearBox};
    mDrivingMotorSim = new rev::spark::SparkMaxSim{mDrivingMotor, mDrivingGearBox};

    mDrivingEncoderSim = new rev::spark::SparkRelativeEncoderSim{mDrivingMotorSim->GetRelativeEncoderSim()};
    mTurningAbsoluteEncoderSim = new rev::spark::SparkAbsoluteEncoderSim{mTurningMotorSim->GetAbsoluteEncoderSim()};

    // Initialization of the molule's SwerveModulePosition and SwerveModuleState from the encoder's velocity and position
    refreshModule();
}

void SwerveModuleSim::setDesiredState(frc::SwerveModuleState iDesiredState, double iSpeedModulation)
{
    mTurningCurrentAngle = units::radian_t(mTurningAbsoluteEncoderSim->GetPosition());
    mOptimizedState = iDesiredState;
    mOptimizedState.Optimize(mTurningCurrentAngle);
    mOptimizedState.CosineScale(mTurningCurrentAngle);

    // frc::SmartDashboard::PutNumber("Drivetrain/Turning RPM", mTurningPID->Calculate(mTurningCurrentAngle.Radians().value()) / kTurningVelocityFactor);
    frc::SmartDashboard::PutNumber("Drivetrain/Turning RPM", mOptimizedState.angle.Radians().value() / kTurningVelocityFactor);
    frc::SmartDashboard::PutNumber("Drivetrain/Driving RPM", mOptimizedState.speed.value() * iSpeedModulation / kDrivingVelocityFactor);
    
    // mTurningPID->SetSetpoint(mOptimizedState.angle.Radians().value());
    // mTurningMotorSim->iterate(mTurningPID->Calculate(mTurningCurrentAngle.Radians().value()), 12, 0.02);
    mDrivingMotorSim->iterate(-mOptimizedState.speed.value() * iSpeedModulation, 12, 0.02);
    // mDrivingEncoderSim->SetPosition(mDrivingEncoderSim->GetPosition() + mDrivingEncoderSim->GetVelocity() * 0.020);
    // mDrivingEncoderSim->SetVelocity(-mOptimizedState.speed.value() * iSpeedModulation);
    mTurningAbsoluteEncoderSim->SetPosition(mOptimizedState.angle.Radians().value());
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