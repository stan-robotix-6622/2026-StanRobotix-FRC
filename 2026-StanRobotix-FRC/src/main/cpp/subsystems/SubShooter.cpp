// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"

SubShooter::SubShooter()
{
    // mPIDcontroller = new frc::PIDController{ShooterConstants::PIDConstants::kP, ShooterConstants::PIDConstants::kI, ShooterConstants::PIDConstants::kD};
    mShooterController =  new rev::spark::SparkMax{CANid::kMotorShooter2ID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mRelativeEncoder = new rev::spark::SparkRelativeEncoder{mShooterController->GetEncoder()};
    mSparkConfigShooter = new rev::spark::SparkMaxConfig{};
    Configure();
    // frc::SmartDashboard::PutData(mPIDcontroller);
}

// This method will be called once per scheduler run
void SubShooter::Periodic() {
    frc::SmartDashboard::PutNumber("Shooter Velocity", getVelocity().value());
}

void SubShooter::setVoltage(units::volt_t iVoltage)
{
    mShooterController->SetVoltage(iVoltage);
};

void SubShooter::setVelocity(units::turns_per_second_t iNextVelocity)
{
    mShooterController->SetVoltage(mFeedforward.Calculate(iNextVelocity));
};

units::turns_per_second_t SubShooter::getVelocity()
{
    return units::revolutions_per_minute_t(mRelativeEncoder->GetVelocity());
};

rev::REVLibError SubShooter::Configure()
{
    mSparkConfigShooter->Inverted(true);

    return mShooterController->Configure(*mSparkConfigShooter, ShooterConstants::kReset, ShooterConstants::kPersist);
};