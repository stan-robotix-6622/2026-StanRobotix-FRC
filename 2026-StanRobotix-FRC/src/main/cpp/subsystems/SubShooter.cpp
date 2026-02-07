// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"

subShooter::subShooter()
{
    mPIDcontroller = new frc::PIDController{PIDConstants::kP, PIDConstants::kI, PIDConstants::kD};
    mShooterController =  new rev::spark::SparkMax{subShooterConstants::kCANid, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mShooterController->SetInverted(true);
    mRelativeEncoder = new rev::spark::SparkRelativeEncoder{mShooterController->GetEncoder()};
}

// This method will be called once per scheduler run
void subShooter::Periodic() {}

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
