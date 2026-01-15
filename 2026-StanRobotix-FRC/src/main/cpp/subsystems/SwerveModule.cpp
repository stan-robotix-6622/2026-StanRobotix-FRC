// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int iNeoMotorID, int iNeo550MotorID, bool iSetInveryed = false)
{
    // Initialization of the motor controllers with the motorID constructor input
    m_MotorNeo = new rev::spark::SparkMax{iNeoMotorID, SwerveModuleConstants::kNeoMotorType};
    m_MotorNeo550 = new rev::spark::SparkMax{iNeo550MotorID, SwerveModuleConstants::kNeo550MotorType};

    m_NeoConfig = new rev::spark::SparkMaxConfig{};
    m_NeoConfig->Inverted(iSetInveryed);
    m_NeoConfig->alternateEncoder.PositionConversionFactor(1 / DriveTrainConstants::kGearRatio);
    m_NeoConfig->alternateEncoder.VelocityConversionFactor(1 / DriveTrainConstants::kGearRatio); // Multiply the results of the encoder by about 0.1968503937

    m_Neo550Config = new rev::spark::SparkMaxConfig{};
    m_Neo550Config->absoluteEncoder.ZeroCentered(true);
    m_Neo550Config->absoluteEncoder.PositionConversionFactor(2 * std::numbers::pi); // To get the position in radians
    m_Neo550Config->absoluteEncoder.VelocityConversionFactor(2 * std::numbers::pi); // To get the velocity in radians per second

    m_MotorNeo->Configure(*m_NeoConfig, SwerveModuleConstants::kNeoResetMode, SwerveModuleConstants::kNeoPersistMode);
    m_MotorNeo550->Configure(*m_Neo550Config, SwerveModuleConstants::kNeo550ResetMode, SwerveModuleConstants::kNeo550PersistMode);

    // Initialization of the PIDController with the P,I and D constants and a continuous input from -pi to pi
    m_Neo550PID = new frc::PIDController{SwerveModuleConstants::kP, SwerveModuleConstants::kI, SwerveModuleConstants::kD};
    m_Neo550PID->EnableContinuousInput(-std::numbers::pi, std::numbers::pi);

    // Initialization of the motor's encoders and absolute encoder
    m_NeoEncoder = new rev::spark::SparkRelativeEncoder{m_MotorNeo->GetEncoder()};
    m_Neo550Encoder = new rev::spark::SparkRelativeEncoder{m_MotorNeo550->GetEncoder()};
    m_Neo550AbsoluteEncoder = new rev::spark::SparkAbsoluteEncoder{m_MotorNeo550->GetAbsoluteEncoder()};

    // Initialization of the molule's SwerveModulePosition and SwerveModuleState from the encoder's velocity and position
    SwerveModule::refreshModule();
    m_ModuleState = getModuleState();
    m_ModulePosition = getModulePosition();
}

frc::SwerveModuleState SwerveModule::optimizeState(frc::SwerveModuleState iDesiredState)
{
    frc::Rotation2d Neo550CurrentAngle(units::radian_t(m_Neo550AbsoluteEncoder->GetPosition()));
    frc::SwerveModuleState OptimizedState = frc::SwerveModuleState::Optimize(iDesiredState, Neo550CurrentAngle);
    OptimizedState.CosineScale(Neo550CurrentAngle);
    return OptimizedState;
}

void SwerveModule::setDesiredState(frc::SwerveModuleState iDesiredState, double SpeedModulation)
{
    frc::SwerveModuleState OptimizedState = optimizeState(iDesiredState);
    m_Neo550PID->SetSetpoint(OptimizedState.angle.Radians().value());
    m_MotorNeo550->Set(m_Neo550PID->Calculate(m_Neo550AbsoluteEncoder->GetPosition()));
    m_MotorNeo->Set(OptimizedState.speed.value() * SpeedModulation);
}

frc::SwerveModuleState * SwerveModule::getModuleState()
{
    return m_ModuleState;
}

frc::SwerveModulePosition * SwerveModule::getModulePosition()
{
    return m_ModulePosition;
}

void SwerveModule::refreshModule()
{
    *m_ModuleState = frc::SwerveModuleState{units::meters_per_second_t(m_NeoEncoder->GetVelocity() * DriveTrainConstants::kWheelPerimeter / DriveTrainConstants::kSecToMinFactor),
                                            frc::Rotation2d(units::radian_t(m_Neo550AbsoluteEncoder->GetPosition()))};
    *m_ModulePosition = frc::SwerveModulePosition{units::meter_t(m_NeoEncoder->GetPosition() * DriveTrainConstants::kWheelPerimeter),
                                                  frc::Rotation2d(units::radian_t(m_Neo550AbsoluteEncoder->GetPosition()))};
}