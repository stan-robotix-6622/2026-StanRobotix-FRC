// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIndexer.h"

subIndexer::subIndexer() 
{
    mIndexerController = new rev::spark::SparkMax{subIndexConstants::kCANid, rev::spark::SparkLowLevel::MotorType::kBrushless};
}

// This method will be called once per scheduler run
void subIndexer::Periodic() {}

void subIndexer::setVoltage(units::volt_t iOutput)
{
    mIndexerController->SetVoltage(iOutput);
};

