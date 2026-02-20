// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"

subShooter::subShooter()
{
    // mPIDcontroller = new frc::PIDController{ShooterConstants::PIDConstants::kP, ShooterConstants::PIDConstants::kI, ShooterConstants::PIDConstants::kD};
    mShooterController =  new rev::spark::SparkMax{CANid::kCANidShooter, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mRelativeEncoder = new rev::spark::SparkRelativeEncoder{mShooterController->GetEncoder()};
    mSparkConfigShooter = new rev::spark::SparkBaseConfig;
    Configure();
    // frc::SmartDashboard::PutData(mPIDcontroller);
}

// This method will be called once per scheduler run
void subShooter::Periodic() {
    frc::SmartDashboard::PutNumber("Shooter Velocity", getVelocity().value());
}

void subShooter::setVoltage(units::volt_t iVoltage)
{
    mShooterController->SetVoltage(iVoltage);
};

void subShooter::setVelocity(units::turns_per_second_t nextVelocity)
{
    mShooterController->SetVoltage(m_feedforward.Calculate(nextVelocity));
};

units::turns_per_second_t subShooter::getVelocity()
{
    return units::revolutions_per_minute_t(mRelativeEncoder->GetVelocity());
};

rev::REVLibError subShooter::Configure()
{
    mSparkConfigShooter->Inverted(true);

    return mShooterController->Configure(*mSparkConfigShooter, ShooterConstants::kReset, ShooterConstants::kPersist);
};