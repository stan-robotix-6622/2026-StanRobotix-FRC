// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIndexer.h"
#include <frc2/command/Commands.h>

SubIndexer::SubIndexer() 
{
    mIndexerController = new rev::spark::SparkMax{CANid::kMotorIndexerID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mSparkConfigIndexer = new rev::spark::SparkBaseConfig;
    Configure();
}

// This method will be called once per scheduler run
void SubIndexer::Periodic() {}

void SubIndexer::setVoltage(units::volt_t iOutput)
{
    mIndexerController->SetVoltage(iOutput);
};

rev::REVLibError SubIndexer::Configure()
{
    mSparkConfigIndexer->Inverted(IndexerConstants::kInverted);
    mSparkConfigIndexer->SetIdleMode(IndexerConstants::kIdleMode);

    return mIndexerController->Configure(*mSparkConfigIndexer, IndexerConstants::kReset, IndexerConstants::kPersist);
};

frc2::CommandPtr SubIndexer::getIndexCommand()
{
    return frc2::cmd::RunEnd(
        [this]
        {
            setVoltage(IndexerConstants::kDesiredVoltage);
        },

        [this] {
            setVoltage(0_V);
        },
        {}
    );
}
