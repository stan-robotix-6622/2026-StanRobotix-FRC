// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"

subShooter::subShooter()
{
    mPIDcontroller = new frc::PIDController{PIDConstants::kP, PIDConstants::kI, PIDConstants::kD};
    mShooterController =  new rev::spark::SparkMax{subShooterConstants::kCANid, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mRelativeEncoder = new rev::spark::SparkRelativeEncoder{mShooterController->GetEncoder()};
    mSparkConfig = new rev::spark::SparkBaseConfig; //Don't forget to put the thingy inside of the other thingy
    frc::SmartDashboard::PutNumber("kP", PIDConstants::kP);
    frc::SmartDashboard::PutNumber("kI", PIDConstants::kI);
    frc::SmartDashboard::PutNumber("kD", PIDConstants::kD);
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
    return units::turns_per_second_t(mRelativeEncoder->GetVelocity());
};

rev::REVLibError subShooter::Configure()
{
    mSparkConfig->Inverted(true);

    return mShooterController->Configure(*mSparkConfig, subShooterConstants::kReset, subShooterConstants::kPersist);
};