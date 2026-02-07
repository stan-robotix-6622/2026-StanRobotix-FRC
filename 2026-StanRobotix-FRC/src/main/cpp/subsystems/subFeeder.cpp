// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subFeeder.h"

subFeeder::subFeeder() 
{
    mFeederController = new rev::spark::SparkMax{subFeederConstants::kCANid, rev::spark::SparkLowLevel::MotorType::kBrushless};
}

// This method will be called once per scheduler run
void subFeeder::Periodic() {}

void subFeeder::setVoltage(units::volt_t iOutput)
{
    mFeederController->SetVoltage(iOutput);
};

