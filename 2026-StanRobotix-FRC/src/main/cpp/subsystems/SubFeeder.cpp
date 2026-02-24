// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubFeeder.h"
#include <frc2/command/Commands.h>

SubFeeder::SubFeeder() 
{
    mFeederController = new rev::spark::SparkMax{CANid::kMotorFeederID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mSparkConfigFeeder = new rev::spark::SparkBaseConfig;
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
    mSparkConfigFeeder->Inverted(true);

    return mFeederController->Configure(*mSparkConfigFeeder, FeederConstants::kReset, FeederConstants::kPersist);
};

frc2::CommandPtr SubFeeder::getFeedShooterCommand()
{
    return frc2::cmd::RunEnd(
        [this]
        {
            setVoltage(FeederConstants::kDesiredVoltage);
        },

        [this]
        {
            setVoltage(0_V);
        },
        {}
    );
}

