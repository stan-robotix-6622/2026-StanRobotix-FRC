// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubFeeder.h"

SubFeeder::SubFeeder() 
{
    mFeederController = new rev::spark::SparkMax{SubFeederConstants::kCANid, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mFeederController->SetInverted(true);
}

// This method will be called once per scheduler run
void SubFeeder::Periodic() {}

void SubFeeder::setVoltage(units::volt_t iOutput)
{
    mFeederController->SetVoltage(iOutput);
};
