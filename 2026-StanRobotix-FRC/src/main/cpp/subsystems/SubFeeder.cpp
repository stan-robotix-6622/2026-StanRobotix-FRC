// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubFeeder.h"
#include <frc2/command/Commands.h>

SubFeeder::SubFeeder() 
{
    mFeederController = new rev::spark::SparkMax{CANid::kMotorFeederID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mSparkConfigFeeder = new rev::spark::SparkMaxConfig;
    Configure();
}

// This method will be called once per scheduler run
void SubFeeder::Periodic() {}

void SubFeeder::setVoltage(units::volt_t iOutput)
{
    mFeederController->SetVoltage(iOutput);
};

rev::REVLibError SubFeeder::Configure()
{
    mSparkConfigFeeder->Inverted(FeederConstants::kInverted);
    mSparkConfigFeeder->SetIdleMode(FeederConstants::kIdleMode);

    return mFeederController->Configure(*mSparkConfigFeeder, FeederConstants::kReset, FeederConstants::kPersist);
};

frc2::CommandPtr SubFeeder::getFeedShooterCommand(units::volt_t iVoltage)
{
    return frc2::cmd::RunEnd(
        [this, iVoltage]
        {
            setVoltage(iVoltage);
        },

        [this]
        {
            setVoltage(0_V);
        },
        {}
    );
}

